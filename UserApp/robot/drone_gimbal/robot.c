#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
#include <stdint.h>
#include <stdbool.h>
RobotInstance *robot;

// 辅助限幅函数
static void LimitTarget(float *val, float min, float max) {
  if (*val > max) *val = max;
  if (*val < min) *val = min;
}

void RobotInit(void) {
  // 1. 内存清零
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

  // 2. 初始化 IMU 和遥控器
  robot->imu_data = INS_Init();
  robot->rc = RemoteControlInit(&huart3);

  // 3. 初始化电机 (直接用宏，简单粗暴)
  // Yaw (ID 3)
  Motor_Init_Config_s yaw_conf = YAW_CONFIG(&hcan1, YAW_ID);
  yaw_conf.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  yaw_conf.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  // Gyro[2] 通常是 Z 轴角速度
  yaw_conf.controller_param_init_config.other_angle_feedback_ptr = &robot->yaw_imu_feed;
  yaw_conf.controller_param_init_config.other_speed_feedback_ptr = &robot->imu_data->Gyro[2];
  // 参数归零
  yaw_conf.controller_param_init_config.angle_PID.Kp = 0;
  yaw_conf.controller_param_init_config.speed_PID.Kp = 0;
  robot->yaw_motor = DJIMotorInit(&yaw_conf);

  // Pitch (ID 5)
  Motor_Init_Config_s pitch_conf = PITCH_CONFIG(&hcan1, PITCH_ID);
  pitch_conf.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  pitch_conf.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  // Pitch 通常用欧拉角 (Pitch) 而不是累积角
  pitch_conf.controller_param_init_config.other_angle_feedback_ptr = &robot->pitch_imu_feed;
  pitch_conf.controller_param_init_config.other_speed_feedback_ptr = &robot->imu_data->Gyro[0]; // 假设 Gyro[0] 是 X轴
  // 参数归零
  pitch_conf.controller_param_init_config.angle_PID.Kp = 0;
  pitch_conf.controller_param_init_config.speed_PID.Kp = 0;

  robot->pitch_motor = DJIMotorInit(&pitch_conf);

  // 摩擦轮 (左ID1, 右ID2)
  Motor_Init_Config_s fric_l_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_L_ID, MOTOR_DIRECTION_REVERSE);
  robot->fric_l = DJIMotorInit(&fric_l_conf);
  Motor_Init_Config_s fric_r_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_R_ID, MOTOR_DIRECTION_NORMAL);
  robot->fric_r = DJIMotorInit(&fric_r_conf);

  // 4. 初始化目标角度为校准值
  // 必须是这个值，不然上电 PID error 巨大，直接起飞
  robot->target_yaw = YAW_INIT_ANGLE;     // 118.69
  robot->target_pitch = PITCH_INIT_ANGLE; // 98.88

  robot->is_first_loop = true;

  robot->mode = ROBOT_STOP;
}

static void RobotControlLogic(void) {
  if (robot->rc == NULL) return;

  // ============================================================
  // 1. 【数据桥接】把 IMU 数据搬运给 PID 监控变量
  // ============================================================
  if (robot->imu_data) {
    robot->yaw_imu_feed = robot->imu_data->YawTotalAngle * -1.0f;

    float raw_pitch = robot->imu_data->Pitch;
    if (raw_pitch < 0) {
      raw_pitch += 360.0f;
    }
    robot->pitch_imu_feed = (180.0f - raw_pitch);
  }

  // ============================================================
  // 2. 上电初始化同步 (防飞车核心逻辑)
  // ============================================================
  if (robot->is_first_loop) {
    // 【修正】删除了 .init 的检查，只检查电机数据
    // 只要电机回传了非0数据，说明系统已经启动一小会儿了，IMU肯定也好了
    if (robot->yaw_motor->measure.ecd != 0) {

      // 关键：把目标对齐到【IMU当前角度】
      robot->target_yaw = robot->yaw_imu_feed;
      robot->target_pitch = robot->pitch_imu_feed;

      // 同步 PID 内部积分状态，防止瞬间突变
      robot->yaw_motor->motor_controller.pid_ref = robot->target_yaw;
      robot->pitch_motor->motor_controller.pid_ref = robot->target_pitch;

      robot->is_first_loop = false;
    } else {
      return; // 等待电机数据回传
    }
  }

  // ============================================================
  // 3. 急停逻辑
  // ============================================================
  if (switch_is_down(robot->rc->rc.switch_right)) {
    robot->mode = ROBOT_STOP;
    DJIMotorStop(robot->yaw_motor);
    DJIMotorStop(robot->pitch_motor);
    DJIMotorStop(robot->fric_l);
    DJIMotorStop(robot->fric_r);

    // 急停时刻跟随【IMU角度】
    // 这样松手时，云台就锁在当前朝向的世界坐标上
    robot->target_yaw = robot->yaw_imu_feed;
    robot->target_pitch = robot->pitch_imu_feed;

    // 更新 PID Ref
    robot->yaw_motor->motor_controller.pid_ref = robot->target_yaw;
    robot->pitch_motor->motor_controller.pid_ref = robot->target_pitch;
    return;
  }

  // [正常模式]
  robot->mode = ROBOT_RUNNING;
  DJIMotorEnable(robot->yaw_motor);
  DJIMotorEnable(robot->pitch_motor);
  DJIMotorEnable(robot->fric_l);
  DJIMotorEnable(robot->fric_r);

  // --- 1. Yaw 轴控制 ---
  // 注意：摇杆控制的是【世界坐标系的朝向】
  robot->target_yaw -= 0.05f * (float)robot->rc->rc.rocker_l_;
  LimitTarget(&robot->target_yaw, YAW_MIN_ANGLE, YAW_MAX_ANGLE);

  // --- 2. Pitch 轴控制 ---
  robot->target_pitch += 0.02f * (float)robot->rc->rc.rocker_r1;
  LimitTarget(&robot->target_pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
  LimitTarget(&robot->target_pitch, -20.0f, 30.0f);

  // --- 3. 应用目标 ---
  // 现在的 Feedback 是 IMU 数据，Target 是 IMU 角度，单位统一了！
  DJIMotorOuterLoop(robot->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->yaw_motor, robot->target_yaw);

  DJIMotorOuterLoop(robot->pitch_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->pitch_motor, robot->target_pitch);

  // --- 4. 摩擦轮逻辑 (保持不变) ---
  if (switch_is_mid(robot->rc->rc.switch_right)) {
    if (switch_is_down(robot->rc->rc.switch_left)) {
      robot->target_fric_speed = FRIC_SPEED_IDLE;
    } else if (switch_is_mid(robot->rc->rc.switch_left)) {
      robot->target_fric_speed = FRIC_SPEED_NORMAL;
    } else if (switch_is_up(robot->rc->rc.switch_left)) {
      robot->target_fric_speed = FRIC_SPEED_HIGH;
    }
  } else {
    robot->target_fric_speed = 0.0f;
  }

  DJIMotorSetPIDRef(robot->fric_l, robot->target_fric_speed);
  DJIMotorSetPIDRef(robot->fric_r, robot->target_fric_speed);
}

void RobotTask(void) {
  RobotControlLogic();
  DJIMotorTask();
}