#include "dart_shoot.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"

/* Private function prototypes -----------------------------------------------*/
static void DartShootModeChangeHandler(DartShootInstance *instance);
static void DartShootStoppedHandler(DartShootInstance *instance);
static void DartShootManualHandler(DartShootInstance *instance);
static void DartShootAutoHandler(DartShootInstance *instance);
static void DartShootEnterState(DartShootInstance *instance, DartShoot_State_e state);
static void DartShootEnableAll(DartShootInstance *instance);
static void DartShootStopAll(DartShootInstance *instance);
static void DartShootSetPullSpeed(DartShootInstance *instance, float speed);
static void DartShootSetTriggerSpeed(DartShootInstance *instance, float speed);
static float DartShootGetPullPosition(DartShootInstance *instance);
static float DartShootGetPullSpeed(DartShootInstance *instance);
static uint8_t DartShootPullStalled(DartShootInstance *instance);
static uint8_t DartShootPullArrived(DartShootInstance *instance);
static uint8_t DartShootAllMotorOnline(DartShootInstance *instance);
static void DartShootRaiseError(DartShootInstance *instance, DartShoot_Error_e error);
static void DartShootUpdateFeed(DartShootInstance *instance);

/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化 dart_shoot
 */
DartShootInstance *DartShootInit(DartShoot_Init_Config_s *init_config) {
  DartShootInstance *instance = (DartShootInstance *)zmalloc(sizeof(DartShootInstance));
  instance->param = init_config->param;

  // 同步带电机 x2: 全部使用速度环, 上电先给 0 速度, 防止乱转
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    instance->pull_motor[i] = DJIMotorInit(&init_config->pull_motor_config[i]);
    DJIMotorOuterLoop(instance->pull_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(instance->pull_motor[i], 0.0f);
  }

  // 扳机位置调整电机: 速度环
  instance->trigger_motor = DJIMotorInit(&init_config->trigger_motor_config);
  DJIMotorOuterLoop(instance->trigger_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(instance->trigger_motor, 0.0f);

  // 拉扳机的 PWM 舵机
  instance->trigger_servo = ServoInit(&init_config->trigger_servo_config);

  instance->ctrl_cmd.mode = DART_SHOOT_MODE_STOPPED;
  instance->ctrl_cmd.cmd = DART_SHOOT_CMD_NONE;
  instance->ctrl_cmd.servo_angle = instance->param.servo_lock_angle;
  instance->mode = DART_SHOOT_MODE_STOPPED;
  instance->state = DART_SHOOT_STATE_STOPPED;
  instance->error = DART_SHOOT_ERROR_NONE;

  // 上电安全状态: 电机不输出, 舵机回锁止位(扳机不动作)
  DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
  DartShootStopAll(instance);
  DartShootUpdateFeed(instance);

  LOGINFO("[dart_shoot] init done, pull motor num: %d", DART_SHOOT_PULL_MOTOR_NUM);
  return instance;
}

/**
 * @brief dart_shoot 任务, 1kHz 调用
 */
void DartShootTask(DartShootInstance *instance) {
  if (instance == NULL) {
    return;
  }

  DartShootModeChangeHandler(instance);

  switch (instance->ctrl_cmd.mode) {
    case DART_SHOOT_MODE_MANUAL:
      DartShootManualHandler(instance);
      break;
    case DART_SHOOT_MODE_AUTO:
      DartShootAutoHandler(instance);
      break;
    case DART_SHOOT_MODE_STOPPED:
    default:
      DartShootStoppedHandler(instance);
      break;
  }

  DartShootUpdateFeed(instance);
}

/**
 * @brief 设置工作模式
 */
void DartShootSetMode(DartShootInstance *instance, DartShoot_Mode_e mode) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.mode = mode;
}

/**
 * @brief 下发一次性指令
 */
void DartShootSendCmd(DartShootInstance *instance, DartShoot_Cmd_e cmd) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.cmd = cmd;
}

/**
 * @brief 手动设置输出, 仅在 DART_SHOOT_MODE_MANUAL 下生效
 */
void DartShootSetManualOutput(DartShootInstance *instance, float pull_speed, float trigger_speed, float servo_angle) {
  if (instance == NULL) {
    return;
  }
  instance->ctrl_cmd.pull_speed = pull_speed;
  instance->ctrl_cmd.trigger_speed = trigger_speed;
  instance->ctrl_cmd.servo_angle = servo_angle;
}

/* Private function ----------------------------------------------------------*/
/**
 * @brief 模式切换处理: 进入停机时立刻停止输出, 进入其他模式时使能电机
 */
static void DartShootModeChangeHandler(DartShootInstance *instance) {
  if (instance->mode == instance->ctrl_cmd.mode) {
    return;
  }
  instance->mode = instance->ctrl_cmd.mode;

  switch (instance->mode) {
    case DART_SHOOT_MODE_MANUAL:
      DartShootEnableAll(instance);
      DartShootEnterState(instance, DART_SHOOT_STATE_MANUAL);
      break;
    case DART_SHOOT_MODE_AUTO:
      DartShootEnableAll(instance);
      DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
      DartShootEnterState(instance, DART_SHOOT_STATE_IDLE);
      break;
    case DART_SHOOT_MODE_STOPPED:
    default:
      DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
      DartShootStopAll(instance);
      DartShootEnterState(instance, DART_SHOOT_STATE_STOPPED);
      break;
  }
}

/**
 * @brief 停机: 电机不输出, 舵机保持在锁止位
 */
static void DartShootStoppedHandler(DartShootInstance *instance) {
  DartShootStopAll(instance);
  instance->ctrl_cmd.cmd = DART_SHOOT_CMD_NONE;  // 停机状态下丢弃所有指令
}

/**
 * @brief 手动: 直接把 cmd 中的速度/角度下发给执行器
 */
static void DartShootManualHandler(DartShootInstance *instance) {
  DartShoot_Ctrl_Cmd_s *cmd = &instance->ctrl_cmd;

  float pull_speed = float_constrain(cmd->pull_speed, -instance->param.pull_speed_limit, instance->param.pull_speed_limit);
  DartShootSetPullSpeed(instance, pull_speed);
  DartShootSetTriggerSpeed(instance, cmd->trigger_speed);
  DartShootSetServoAngle(instance, cmd->servo_angle);

  instance->state = DART_SHOOT_STATE_MANUAL;
  cmd->cmd = DART_SHOOT_CMD_NONE;
}

/**
 * @brief 自动: 状态机, 完成 回零-拉滑块-击发-复位 流程
 */
static void DartShootAutoHandler(DartShootInstance *instance) {
  DartShoot_Ctrl_Cmd_s *cmd = &instance->ctrl_cmd;
  float now = DWT_GetTimeline_ms();

  // 扳机位置调整电机: 任何时候都跟随 cmd.trigger_speed(默认 0, 可用于调整扳机位置)
  DartShootSetTriggerSpeed(instance, cmd->trigger_speed);

  // 异常状态: 只接受 CMD_RESET
  if (instance->state == DART_SHOOT_STATE_ERROR) {
    DartShootSetPullSpeed(instance, 0.0f);
    if (cmd->cmd == DART_SHOOT_CMD_RESET) {
      instance->error = DART_SHOOT_ERROR_NONE;
      DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
      DartShootEnterState(instance, DART_SHOOT_STATE_IDLE);
    }
    cmd->cmd = DART_SHOOT_CMD_NONE;
    return;
  }

  // 处理一次性指令
  switch (cmd->cmd) {
    case DART_SHOOT_CMD_HOME:
      DartShootEnterState(instance, DART_SHOOT_STATE_HOMING);
      break;
    case DART_SHOOT_CMD_PULL:
      if (instance->state == DART_SHOOT_STATE_IDLE) {
        DartShootEnterState(instance, DART_SHOOT_STATE_PULLING);
      } else {
        LOGWARNING("[dart_shoot] pull ignored, state: %d", instance->state);
      }
      break;
    case DART_SHOOT_CMD_FIRE:
      if (instance->state == DART_SHOOT_STATE_READY) {
        DartShootEnterState(instance, DART_SHOOT_STATE_RELEASING);
      } else {
        LOGWARNING("[dart_shoot] fire ignored, not ready, state: %d", instance->state);
      }
      break;
    case DART_SHOOT_CMD_RELOAD:
      DartShootEnterState(instance, DART_SHOOT_STATE_RELOADING);
      break;
    case DART_SHOOT_CMD_ABORT:
      DartShootSetPullSpeed(instance, 0.0f);
      DartShootEnterState(instance, DART_SHOOT_STATE_IDLE);
      break;
    default:
      break;
  }
  cmd->cmd = DART_SHOOT_CMD_NONE;

  // 运动中若电机离线, 直接进入异常
  if (!DartShootAllMotorOnline(instance) && instance->online_latched &&
      (instance->state == DART_SHOOT_STATE_HOMING || instance->state == DART_SHOOT_STATE_PULLING ||
       instance->state == DART_SHOOT_STATE_RELEASING || instance->state == DART_SHOOT_STATE_RELOADING)) {
    DartShootRaiseError(instance, DART_SHOOT_ERROR_MOTOR_OFFLINE);
    return;
  }

  switch (instance->state) {
    case DART_SHOOT_STATE_IDLE:
      DartShootSetPullSpeed(instance, 0.0f);
      break;

    // 回零: 沿拉滑块的反方向低速运动, 直到堵转(机械限位), 该位置即为零点
    case DART_SHOOT_STATE_HOMING:
      DartShootSetPullSpeed(instance, -instance->param.pull_direction * instance->param.home_speed);
      if (DartShootPullStalled(instance)) {
        DartShootSetPullSpeed(instance, 0.0f);
        instance->pull_zero_angle = DartShootGetPullPosition(instance);
        instance->pull_start_angle = instance->pull_zero_angle;
        instance->is_homed = 1;
        LOGINFO("[dart_shoot] homing done");
        DartShootEnterState(instance, DART_SHOOT_STATE_IDLE);
      } else if (now - instance->state_start_time > instance->param.home_timeout_ms) {
        DartShootRaiseError(instance, DART_SHOOT_ERROR_HOME_TIMEOUT);
      }
      break;

    // 拉滑块: 速度环给定拉滑块速度, 走到扳机位(行程到位或顶到扳机堵转)后停下
    case DART_SHOOT_STATE_PULLING:
      DartShootSetPullSpeed(instance, instance->param.pull_direction * instance->param.pull_speed);
      if (DartShootPullArrived(instance)) {
        DartShootSetPullSpeed(instance, 0.0f);
        instance->pull_time_ms = now - instance->state_start_time;
        DartShootEnterState(instance, DART_SHOOT_STATE_READY);
      } else if (now - instance->state_start_time > instance->param.pull_timeout_ms) {
        DartShootRaiseError(instance, DART_SHOOT_ERROR_PULL_TIMEOUT);
      }
      break;

    // 就位: 滑块由扳机挡住, 同步带不再出力, 等待击发指令
    case DART_SHOOT_STATE_READY:
      DartShootSetPullSpeed(instance, 0.0f);
      break;

    // 击发: 舵机拉下扳机, 滑块飞出; 保持一段时间后进入复位
    case DART_SHOOT_STATE_RELEASING:
      DartShootSetPullSpeed(instance, 0.0f);
      if (now - instance->state_start_time >= instance->param.servo_release_time_ms) {
        instance->fire_time_ms = now - instance->state_start_time;
        DartShootEnterState(instance, DART_SHOOT_STATE_RELOADING);
      }
      break;

    // 复位: 舵机回锁止位, 同步带反向把滑块收回零点
    case DART_SHOOT_STATE_RELOADING:
      DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
      DartShootSetPullSpeed(instance, -instance->param.pull_direction * instance->param.reload_speed);
      if (DartShootPullStalled(instance) ||
          fabsf(DartShootGetPullPosition(instance) - instance->pull_start_angle) <
              instance->param.pull_position_tolerance) {
        DartShootSetPullSpeed(instance, 0.0f);
        DartShootEnterState(instance, DART_SHOOT_STATE_IDLE);
      } else if (now - instance->state_start_time > instance->param.reload_timeout_ms) {
        DartShootRaiseError(instance, DART_SHOOT_ERROR_RELOAD_TIMEOUT);
      }
      break;

    default:
      break;
  }
}

/**
 * @brief 状态切换, 并执行进入该状态时需要立刻完成的动作
 */
static void DartShootEnterState(DartShootInstance *instance, DartShoot_State_e state) {
  instance->state = state;
  instance->state_start_time = DWT_GetTimeline_ms();
  instance->stall_timing = 0;

  switch (state) {
    case DART_SHOOT_STATE_PULLING:
      // 记录本次拉滑块的起点, 用于行程计算(未回零时也可工作)
      instance->pull_start_angle = DartShootGetPullPosition(instance);
      break;
    case DART_SHOOT_STATE_RELEASING:
      // 舵机拉下扳机, 释放滑块
      DartShootSetServoAngle(instance, instance->param.servo_release_angle);
      instance->fire_count++;
      break;
    case DART_SHOOT_STATE_RELOADING:
      DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
      break;
    case DART_SHOOT_STATE_IDLE:
    case DART_SHOOT_STATE_HOMING:
    case DART_SHOOT_STATE_READY:
    case DART_SHOOT_STATE_ERROR:
    case DART_SHOOT_STATE_STOPPED:
    case DART_SHOOT_STATE_MANUAL:
    default:
      break;
  }
}

/**
 * @brief 使能三个 3508
 */
static void DartShootEnableAll(DartShootInstance *instance) {
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    DJIMotorEnable(instance->pull_motor[i]);
  }
  DJIMotorEnable(instance->trigger_motor);
}

/**
 * @brief 停止三个 3508(直接给 0 电流)
 */
static void DartShootStopAll(DartShootInstance *instance) {
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    DJIMotorStop(instance->pull_motor[i]);
  }
  DJIMotorStop(instance->trigger_motor);
}

/**
 * @brief 设置两个同步带电机的速度参考(度/秒), 两个电机使用同一参考值
 */
static void DartShootSetPullSpeed(DartShootInstance *instance, float speed) {
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    DJIMotorOuterLoop(instance->pull_motor[i], SPEED_LOOP);
    DJIMotorSetPIDRef(instance->pull_motor[i], speed);
  }
}

/**
 * @brief 设置扳机调整电机的速度参考(度/秒)
 */
static void DartShootSetTriggerSpeed(DartShootInstance *instance, float speed) {
  float limit = instance->param.trigger_speed_limit;
  speed = float_constrain(speed, -limit, limit);
  DJIMotorOuterLoop(instance->trigger_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(instance->trigger_motor, speed);
}

/**
 * @brief 设置舵机角度(°): 线性映射为脉宽, 再转换为 PWM 占空比
 */
void DartShootSetServoAngle(DartShootInstance *instance, float angle) {
  if (instance->trigger_servo == NULL) {
    return;
  }
  float min_angle = instance->param.servo_angle_min;
  float max_angle = instance->param.servo_angle_max;
  if (max_angle <= min_angle || instance->param.servo_period_s <= 0.0f) {
    LOGERROR("[dart_shoot] servo param error");
    return;
  }

  angle = float_constrain(angle, min_angle, max_angle);

  // 角度 -> 脉宽(s) -> 占空比
  float pulse = instance->param.servo_min_pulse_s +
                (angle - min_angle) / (max_angle - min_angle) *
                    (instance->param.servo_max_pulse_s - instance->param.servo_min_pulse_s);
  float duty = pulse / instance->param.servo_period_s;
  duty = float_constrain(duty, 0.0f, 1.0f);

  ServoSetAngle(instance->trigger_servo, duty);
  instance->servo_angle_cmd = angle;
}

/**
 * @brief 同步带滑块位置: 取两个电机总角度的平均值
 */
static float DartShootGetPullPosition(DartShootInstance *instance) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    sum += instance->pull_motor[i]->measure.total_angle;
  }
  return sum / (float)DART_SHOOT_PULL_MOTOR_NUM;
}

/**
 * @brief 同步带电机平均速度(度/秒)
 */
static float DartShootGetPullSpeed(DartShootInstance *instance) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    sum += instance->pull_motor[i]->measure.speed_aps;
  }
  return sum / (float)DART_SHOOT_PULL_MOTOR_NUM;
}

/**
 * @brief 判断同步带是否已经顶到机械限位(扳机/零点):
 *        电流超过阈值或底层 PID 报堵转, 并连续保持 pull_stall_time_ms
 */
static uint8_t DartShootPullStalled(DartShootInstance *instance) {
  float current = 0.0f;
  uint8_t pid_blocked = 0;

  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    current += fabsf((float)instance->pull_motor[i]->measure.real_current);
    if (instance->pull_motor[i]->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
      instance->pull_motor[i]->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
      instance->pull_motor[i]->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;
      pid_blocked = 1;
    }
  }
  current /= (float)DART_SHOOT_PULL_MOTOR_NUM;

  if (pid_blocked || current > instance->param.pull_current_threshold) {
    float now = DWT_GetTimeline_ms();
    if (!instance->stall_timing) {
      instance->stall_timing = 1;
      instance->stall_start_time = now;
    }
    if (now - instance->stall_start_time >= instance->param.pull_stall_time_ms) {
      return 1;
    }
  } else {
    instance->stall_timing = 0;
  }
  return 0;
}

/**
 * @brief 判断滑块是否已经拉到扳机位: 行程到位, 或者已经顶住扳机(堵转)
 */
static uint8_t DartShootPullArrived(DartShootInstance *instance) {
  float traveled = fabsf(DartShootGetPullPosition(instance) - instance->pull_start_angle);
  if (traveled + instance->param.pull_position_tolerance >= instance->param.pull_travel) {
    return 1;
  }
  return DartShootPullStalled(instance);
}

/**
 * @brief 三个 3508 是否都在线
 */
static uint8_t DartShootAllMotorOnline(DartShootInstance *instance) {
  for (uint8_t i = 0; i < DART_SHOOT_PULL_MOTOR_NUM; i++) {
    if (!DaemonIsOnline(instance->pull_motor[i]->daemon)) {
      return 0;
    }
  }
  return DaemonIsOnline(instance->trigger_motor->daemon);
}

/**
 * @brief 进入异常状态: 停止同步带并让舵机回锁止位
 */
static void DartShootRaiseError(DartShootInstance *instance, DartShoot_Error_e error) {
  instance->error = error;
  DartShootSetPullSpeed(instance, 0.0f);
  DartShootSetServoAngle(instance, instance->param.servo_lock_angle);
  DartShootEnterState(instance, DART_SHOOT_STATE_ERROR);
  LOGERROR("[dart_shoot] error code: %d", error);
}

/**
 * @brief 更新反馈数据
 */
static void DartShootUpdateFeed(DartShootInstance *instance) {
  if (DartShootAllMotorOnline(instance)) {
    instance->online_latched = 1;
  }

  instance->feed.state = instance->state;
  instance->feed.error = instance->error;
  instance->feed.is_homed = instance->is_homed;
  instance->feed.is_ready = (instance->state == DART_SHOOT_STATE_READY);
  instance->feed.is_online = DartShootAllMotorOnline(instance);
  instance->feed.fire_count = instance->fire_count;
  instance->feed.pull_position = DartShootGetPullPosition(instance) - instance->pull_zero_angle;
  instance->feed.pull_traveled = fabsf(DartShootGetPullPosition(instance) - instance->pull_start_angle);
  instance->feed.pull_time_ms = instance->pull_time_ms;
  instance->feed.fire_time_ms = instance->fire_time_ms;
  instance->feed.servo_angle = instance->servo_angle_cmd;
  instance->feed.pull_speed = DartShootGetPullSpeed(instance);
}
