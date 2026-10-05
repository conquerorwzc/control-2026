#include "dart_shoot.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"

/* 单周期最大累加时间(s): 任务被阻塞时不让位置一次性累加过多 */
#define DART_POS_DT_MAX_S 0.01f

/* 舵机角度按 0~DART_SERVO_ANGLE_RANGE(°) 线性映射到 servo_min_pulse_s ~ servo_max_pulse_s
 * 实车用 270° 舵机(0.5ms~2.5ms 对应 0~270°), 换舵机时改这里和 param 里的脉宽参数 */
#define DART_SERVO_ANGLE_RANGE 270.0f

/*
 * 本文件不写函数前置声明: 私有函数按"被调用者在前"的顺序定义, 对外接口声明在 dart_shoot.h 中。
 */

/*=======私有函数: 依赖在前, 不需要前置声明=======*/
/**
 * @brief 同步带当前位置: 取两个电机总角度的平均值
 */
static float DartShootGetBeltPosition(DartShootInstance *instance) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
    sum += instance->belt_motor[i]->measure.total_angle;
  }
  return sum / (float)DART_SHOOT_BELT_MOTOR_NUM;
}

/**
 * @brief yaw 当前位置(电机总角度)
 */
static float DartShootGetYawPosition(DartShootInstance *instance) {
  return instance->yaw_motor->measure.total_angle;
}

/**
 * @brief 把同步带目标位置同步到当前位置(失能期间机构可能被手推动, 避免使能瞬间跳变)
 */
static void DartShootSyncBeltTarget(DartShootInstance *instance) {
  instance->belt_target = DartShootGetBeltPosition(instance);
}

/**
 * @brief 把 yaw 目标位置同步到当前位置
 */
static void DartShootSyncYawTarget(DartShootInstance *instance) {
  instance->yaw_target = DartShootGetYawPosition(instance);
}

/**
 * @brief 使能同步带电机并清空 PID 状态, 避免重新使能时输出跳变
 */
static void DartShootEnableBelt(DartShootInstance *instance) {
  for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
    DJIMotorOuterLoop(instance->belt_motor[i], ANGLE_LOOP);
    PIDClear(&instance->belt_motor[i]->motor_controller.angle_PID);
    PIDClear(&instance->belt_motor[i]->motor_controller.speed_PID);
    DJIMotorEnable(instance->belt_motor[i]);
    DJIMotorSetPIDRef(instance->belt_motor[i], instance->belt_target);
  }
  instance->belt_enabled = 1;
}

/**
 * @brief 同步带失能(不发电流)
 */
static void DartShootDisableBelt(DartShootInstance *instance) {
  for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
    DJIMotorStop(instance->belt_motor[i]);
  }
  instance->belt_enabled = 0;
}

/**
 * @brief 使能 yaw 电机并清空 PID 状态
 */
static void DartShootEnableYaw(DartShootInstance *instance) {
  DJIMotorOuterLoop(instance->yaw_motor, ANGLE_LOOP);
  PIDClear(&instance->yaw_motor->motor_controller.angle_PID);
  PIDClear(&instance->yaw_motor->motor_controller.speed_PID);
  DJIMotorEnable(instance->yaw_motor);
  DJIMotorSetPIDRef(instance->yaw_motor, instance->yaw_target);
  instance->yaw_enabled = 1;
}

/**
 * @brief yaw 失能(不发电流, 轴可以自由推动)
 */
static void DartShootDisableYaw(DartShootInstance *instance) {
  DJIMotorStop(instance->yaw_motor);
  instance->yaw_enabled = 0;
}

/**
 * @brief 使能扳机位置电机并清空 PID 状态
 */
static void DartShootEnableTrigger(DartShootInstance *instance) {
  DJIMotorOuterLoop(instance->trigger_motor, SPEED_LOOP);
  PIDClear(&instance->trigger_motor->motor_controller.speed_PID);
  DJIMotorEnable(instance->trigger_motor);
  instance->trigger_enabled = 1;
}

/**
 * @brief 扳机位置电机失能(不发电流)
 */
static void DartShootDisableTrigger(DartShootInstance *instance) {
  DJIMotorStop(instance->trigger_motor);
  instance->trigger_enabled = 0;
}

/**
 * @brief 失能档位: 所有电机停止输出
 */
static void DartShootDisabledHandler(DartShootInstance *instance) {
  DartShootDisableBelt(instance);
  DartShootDisableYaw(instance);
  DartShootDisableTrigger(instance);
}

/**
 * @brief TRIGGER 档位: 只驱动扳机位置电机(速度环), 摇杆没有输入直接失能
 */
static void DartShootTriggerHandler(DartShootInstance *instance) {
  if (instance->ctrl_cmd.trigger_dir == 0) {
    if (instance->trigger_enabled) {
      // 摇杆没有输入: 直接失能(不发电流), 而不是速度环给定 0
      DartShootDisableTrigger(instance);
    }
    return;
  }

  if (!instance->trigger_enabled) {
    DartShootEnableTrigger(instance);
  }
  DJIMotorOuterLoop(instance->trigger_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(instance->trigger_motor, (float)instance->ctrl_cmd.trigger_dir * instance->param.trigger_speed);
}

/**
 * @brief 同步带: 位置累加, 没有新指令时位置环顶住最后一个目标位置
 *
 * @note  摇杆只决定累加方向, 速率由 param.belt_pos_rate 决定
 */
static void DartShootBeltHandler(DartShootInstance *instance, float dt) {
  DartShoot_Ctrl_Cmd_s *cmd = &instance->ctrl_cmd;

  if (cmd->belt_dir != 0) {
    if (!instance->belt_enabled) {
      DartShootEnableBelt(instance);
    }
    instance->belt_target += (float)cmd->belt_dir * instance->param.belt_pos_rate * dt;
  }
  if (instance->belt_enabled) {
    // 位置环给定: 无指令时给定值不变, 同步带就停在最后一个目标位置上
    for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
      DJIMotorSetPIDRef(instance->belt_motor[i], instance->belt_target);
    }
  }
}

/**
 * @brief yaw: 位置累加, 没有新指令时直接失能(只有 TRIGGER 档调用)
 *
 * @note  摇杆只决定累加方向, 速率由 param.yaw_pos_rate 决定
 */
static void DartShootYawHandler(DartShootInstance *instance, float dt) {
  DartShoot_Ctrl_Cmd_s *cmd = &instance->ctrl_cmd;

  if (cmd->yaw_dir != 0) {
    if (!instance->yaw_enabled) {
      DartShootSyncYawTarget(instance);  // 失能期间可能被手推动, 从当前位置重新开始累加
      DartShootEnableYaw(instance);
    }
    instance->yaw_target += (float)cmd->yaw_dir * instance->param.yaw_pos_rate * dt;
    DJIMotorSetPIDRef(instance->yaw_motor, instance->yaw_target);
  } else if (instance->yaw_enabled) {
    DartShootDisableYaw(instance);
  }
}

/**
 * @brief 舵机处理: 只在指令变化时动作(整体失能优先)
 */
static void DartShootServoHandler(DartShootInstance *instance) {
  DartServo_Cmd_e want = instance->ctrl_cmd.servo_cmd;
  if (instance->mode == DART_SHOOT_MODE_DISABLED) {
    want = DART_SERVO_CMD_DISABLED;  // 整体失能时舵机一并失能
  }
  if (want == instance->servo_cmd) {
    return;
  }
  instance->servo_cmd = want;

  switch (want) {
    case DART_SERVO_CMD_ANGLE_MID:
      DartShootSetServoAngle(instance, instance->param.servo_angle_mid);
      LOGINFO("[dart_shoot] servo -> mid angle");
      break;
    case DART_SERVO_CMD_ANGLE_UP:
      DartShootSetServoAngle(instance, instance->param.servo_angle_up);
      LOGINFO("[dart_shoot] servo -> up angle");
      break;
    case DART_SERVO_CMD_DISABLED:
    default:
      ServoStop(instance->trigger_servo);
      instance->servo_enabled = 0;
      LOGWARNING("[dart_shoot] servo disabled");
      break;
  }
}

/**
 * @brief 档位切换处理: 进入各档位时立刻把执行器设置到正确状态
 */
static void DartShootModeChangeHandler(DartShootInstance *instance) {
  if (instance->mode == instance->ctrl_cmd.mode) {
    return;
  }
  instance->mode = instance->ctrl_cmd.mode;

  switch (instance->mode) {
    case DART_SHOOT_MODE_AIM:
      DartShootDisableTrigger(instance);
      DartShootSyncBeltTarget(instance);
      DartShootSyncYawTarget(instance);
      DartShootEnableBelt(instance);
      DartShootDisableYaw(instance);  // yaw 等到有摇杆指令时再使能
      LOGINFO("[dart_shoot] mode -> AIM: belt holds position, yaw waits for stick");
      break;

    case DART_SHOOT_MODE_TRIGGER:
      // 同步带保持使能并锁定在上一次的目标位置: 中档拉到哪, 切到上档就锁在哪
      // (之前处于失能档时可能被手推动, 从当前位置重新锁)
      if (!instance->belt_enabled) {
        DartShootSyncBeltTarget(instance);
        DartShootEnableBelt(instance);
      }
      LOGINFO("[dart_shoot] mode -> TRIGGER: belt holds position, yaw + trigger motor enabled");
      break;

    case DART_SHOOT_MODE_DISABLED:
    default:
      DartShootDisabledHandler(instance);
      LOGWARNING("[dart_shoot] mode -> DISABLED: all actuators off");
      break;
  }
}

/**
 * @brief 更新反馈数据
 */
static void DartShootUpdateFeed(DartShootInstance *instance) {
  uint8_t online = 1;
  for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
    if (!DaemonIsOnline(instance->belt_motor[i]->daemon)) {
      online = 0;
    }
  }
  if (!DaemonIsOnline(instance->yaw_motor->daemon) || !DaemonIsOnline(instance->trigger_motor->daemon)) {
    online = 0;
  }

  instance->feed.mode = instance->mode;
  instance->feed.online = online;
  instance->feed.belt_enabled = instance->belt_enabled;
  instance->feed.yaw_enabled = instance->yaw_enabled;
  instance->feed.trigger_enabled = instance->trigger_enabled;
  instance->feed.servo_enabled = instance->servo_enabled;
  instance->feed.belt_target = instance->belt_target;
  instance->feed.belt_position = DartShootGetBeltPosition(instance);
  instance->feed.yaw_target = instance->yaw_target;
  instance->feed.yaw_position = DartShootGetYawPosition(instance);
  instance->feed.trigger_speed = instance->trigger_motor->measure.speed_aps;
  instance->feed.servo_angle = instance->servo_angle;
}

/*=======对外接口: 原型见 dart_shoot.h=======*/
/**
 * @brief 初始化 dart_shoot
 */
DartShootInstance *DartShootInit(DartShoot_Init_Config_s *init_config) {
  DartShootInstance *instance = (DartShootInstance *)zmalloc(sizeof(DartShootInstance));
  instance->param = init_config->param;

  for (uint8_t i = 0; i < DART_SHOOT_BELT_MOTOR_NUM; i++) {
    instance->belt_motor[i] = DJIMotorInit(&init_config->belt_motor_config[i]);
  }
  instance->yaw_motor = DJIMotorInit(&init_config->yaw_motor_config);
  instance->trigger_motor = DJIMotorInit(&init_config->trigger_motor_config);
  instance->trigger_servo = ServoInit(&init_config->trigger_servo_config);

  instance->ctrl_cmd.mode = DART_SHOOT_MODE_DISABLED;
  instance->ctrl_cmd.belt_dir = 0;
  instance->ctrl_cmd.yaw_dir = 0;
  instance->ctrl_cmd.trigger_dir = 0;
  instance->ctrl_cmd.servo_cmd = DART_SERVO_CMD_DISABLED;
  instance->mode = DART_SHOOT_MODE_DISABLED;
  instance->servo_cmd = DART_SERVO_CMD_DISABLED;

  // 上电安全状态: 4 个电机不输出电流, 舵机不输出脉冲
  DartShootDisabledHandler(instance);
  ServoStop(instance->trigger_servo);
  DartShootSyncBeltTarget(instance);
  DartShootSyncYawTarget(instance);
  DWT_GetDeltaT(&instance->dt_cnt);  // 丢弃第一次 dt
  DartShootUpdateFeed(instance);

  LOGINFO("[dart_shoot] init done, belt motor num: %d", DART_SHOOT_BELT_MOTOR_NUM);
  return instance;
}

/**
 * @brief dart_shoot 任务, 1kHz 调用
 */
void DartShootTask(DartShootInstance *instance) {
  if (instance == NULL) {
    return;
  }

  float dt = DWT_GetDeltaT(&instance->dt_cnt);
  dt = float_constrain(dt, 0.0f, DART_POS_DT_MAX_S);  // 防止任务被阻塞时位置一次性累加过多

  DartShootModeChangeHandler(instance);

  switch (instance->ctrl_cmd.mode) {
    case DART_SHOOT_MODE_AIM:
      DartShootBeltHandler(instance, dt);  // 中档只有同步带, yaw 不使用
      break;
    case DART_SHOOT_MODE_TRIGGER:
      DartShootBeltHandler(instance, dt);  // belt_dir = 0: 只保持位置环给定, 锁定不动
      DartShootYawHandler(instance, dt);
      DartShootTriggerHandler(instance);
      break;
    case DART_SHOOT_MODE_DISABLED:
    default:
      DartShootDisabledHandler(instance);
      break;
  }

  DartShootServoHandler(instance);
  DartShootUpdateFeed(instance);
}

/**
 * @brief 设置工作档位
 */
void DartShootSetMode(DartShootInstance *instance, DartShoot_Mode_e mode) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.mode = mode;
}

/**
 * @brief 设置同步带位置累加方向
 */
void DartShootSetBeltDir(DartShootInstance *instance, int8_t dir) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.belt_dir = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
}

/**
 * @brief 设置 yaw 位置累加方向
 */
void DartShootSetYawDir(DartShootInstance *instance, int8_t dir) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.yaw_dir = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
}

/**
 * @brief 设置扳机位置电机转向
 */
void DartShootSetTriggerDir(DartShootInstance *instance, int8_t dir) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.trigger_dir = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
}

/**
 * @brief 设置舵机指令
 */
void DartShootSetServo(DartShootInstance *instance, DartServo_Cmd_e servo_cmd) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.servo_cmd = servo_cmd;
}

/**
 * @brief 设置舵机角度(°): 角度 -> 脉宽 -> 占空比, 并立刻使能舵机
 */
void DartShootSetServoAngle(DartShootInstance *instance, float angle) {
  if (instance == NULL || instance->trigger_servo == NULL) {
    return;
  }

  PWMInstance *pwm = instance->trigger_servo->pwm_instance;
  if (instance->trigger_servo->servo_type != PWM_Servo || pwm == NULL || pwm->period <= 0.0f) {
    LOGERROR("[dart_shoot] servo must be a PWM servo with valid period");
    return;
  }

  angle = float_constrain(angle, 0.0f, DART_SERVO_ANGLE_RANGE);
  float pulse = instance->param.servo_min_pulse_s +
                angle / DART_SERVO_ANGLE_RANGE * (instance->param.servo_max_pulse_s - instance->param.servo_min_pulse_s);
  float duty = float_constrain(pulse / pwm->period, 0.0f, 1.0f);

  ServoStart(instance->trigger_servo);  // 失能状态下调用本函数时自动恢复输出
  ServoSetAngle(instance->trigger_servo, duty);
  instance->servo_enabled = 1;
  instance->servo_angle = angle;
}
