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

  // 2. 初始化遥控器 (直接调，防止宏定义坑)
  robot->rc = RemoteControlInit(&huart3);

  // 3. 初始化电机 (直接用宏，简单粗暴)
  // Yaw (ID 3)
  Motor_Init_Config_s yaw_conf = YAW_CONFIG(&hcan1, YAW_ID);
  robot->yaw_motor = DJIMotorInit(&yaw_conf);

  // Pitch (ID 5)
  Motor_Init_Config_s pitch_conf = PITCH_CONFIG(&hcan1, PITCH_ID);
  robot->pitch_motor = DJIMotorInit(&pitch_conf);

  // 摩擦轮 (左ID1, 右ID2)
  Motor_Init_Config_s fric_l_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_L_ID, MOTOR_DIRECTION_NORMAL);
  robot->fric_l = DJIMotorInit(&fric_l_conf);
  Motor_Init_Config_s fric_r_conf = SHOOT_MOTOR_CONFIG(&hcan1, FRIC_R_ID, MOTOR_DIRECTION_REVERSE);
  robot->fric_r = DJIMotorInit(&fric_r_conf);

  // 4. 初始化目标角度为校准值
  // 必须是这个值，不然上电 PID error 巨大，直接起飞
  robot->target_yaw = YAW_INIT_ANGLE;     // 118.69
  robot->target_pitch = PITCH_INIT_ANGLE; // 98.88

  robot->mode = ROBOT_STOP;
}

static void RobotControlLogic(void) {
  if (robot->rc == NULL) return;

  // 如果是上电后的第一次循环，且电机还没准备好数据，就不要进行控制
  if (robot->is_first_loop) {
    // 检查 Yaw 和 Pitch 是否都已收到有效数据 (ECD不为0通常意味着数据来了)
    bool yaw_ready = (robot->yaw_motor->measure.ecd != 0);
    bool pitch_ready = (robot->pitch_motor->measure.ecd != 0);

    if (yaw_ready && pitch_ready) {
      // 核心：把目标值强制设为“当前实际位置”
      // 这样 PID 误差 = 0，电机就不会动
      robot->target_yaw = robot->yaw_motor->measure.total_angle;
      robot->target_pitch = robot->pitch_motor->measure.total_angle;

      // 顺便更新 PID 内部的 Ref，防止积分器问题
      robot->yaw_motor->motor_controller.pid_ref = robot->target_yaw;
      robot->pitch_motor->motor_controller.pid_ref = robot->target_pitch;

      // 初始化完成，关闭标志位
      robot->is_first_loop = false;
    } else {
      // 如果数据还没回来，直接跳过本次控制，等待下一帧
      return;
    }
  }
  // [急停逻辑] 右拨杆在下 -> 停止
  if (switch_is_down(robot->rc->rc.switch_right)) {
    robot->mode = ROBOT_STOP;
    DJIMotorStop(robot->yaw_motor);
    DJIMotorStop(robot->pitch_motor);
    DJIMotorStop(robot->fric_l);
    DJIMotorStop(robot->fric_r);

    // 在急停(失能)状态下，让目标值实时跟随电机当前的实际位置。
    // 这样，当你手掰动了云台，或者切回正常模式的那一瞬间，
    // 目标值 == 实际值，云台会从当前位置平滑开始控制，而不是猛甩回 INIT_ANGLE。
    robot->target_yaw = robot->yaw_motor->measure.total_angle;
    robot->target_pitch = robot->pitch_motor->measure.total_angle;

    // 同时也更新 PID Ref
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

  // 1. Yaw 轴控制 (左摇杆左右)
  // 逻辑：你向左转(摇杆+)，ECD需要变小(去596/29.8度)，所以用减法
  robot->target_yaw -= 0.005f * (float)robot->rc->rc.rocker_l_;
  // 限位：[29.80, 208.69]
  LimitTarget(&robot->target_yaw, YAW_MIN_ANGLE, YAW_MAX_ANGLE);

  // 2. Pitch 轴控制 (右摇杆上下)
  // 逻辑：你向上推(摇杆+)，需要抬头(去水平/98.88度/ECD变大)，所以用加法
  robot->target_pitch += 0.0002f * (float)robot->rc->rc.rocker_r1;
  // 限位：[54.93, 98.88]
  LimitTarget(&robot->target_pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

  // 3. 应用目标到电机
  DJIMotorOuterLoop(robot->yaw_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->yaw_motor, robot->target_yaw);

  DJIMotorOuterLoop(robot->pitch_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(robot->pitch_motor, robot->target_pitch);

  // 4. 摩擦轮 (右拨杆中/上 -> 开启)
  if (!switch_is_down(robot->rc->rc.switch_right)) {
    robot->target_fric_speed = 0.0f;
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