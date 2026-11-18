#include "robot.h"

/**
 ******************************************************************************
 * @file    robot.c
 * @brief   云台 Demo
 *          - 左摇杆左右 -> 云台 yaw 目标角(速度式: 摇杆大小 = 目标角速度)
 *          - 左摇杆上下 -> 云台 pitch 目标角(速度式, 带限位)
 *          - 右拨杆[中]/[上] 使能, [下] 或遥控器掉线 失能
 * @note    只初始化云台, 不涉及底盘/发射/视觉/双板通信
 ******************************************************************************
 * 操作说明:
 *   右拨杆 [中]/[上] : 云台使能     [下] : 云台失能(电机断电)
 *   左摇杆 左右      : yaw 转动     左摇杆 上下 : pitch 转动
******************************************************************************
 */

#include <stdlib.h>

#include "bsp_dwt.h"
#include "daemon.h"
#include "robot_config.h"
#include "user_lib.h"

#define RC_ROCKER_MAX 660.0f      // 摇杆单侧满量程, 用于把原始值归一化
#define YAW_STICK_SPEED 100.0f    // 摇杆打满时 yaw 的目标角速度, deg/s
#define PITCH_STICK_SPEED 100.0f  // 摇杆打满时 pitch 的目标角速度, deg/s
#define STICK_DEADBAND 30         // 摇杆死区(原始 count), 防止回中不到位

static RobotInstance *robot;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static RC_ctrl_t *rc_data;
static uint8_t gimbal_enabled;  // 上一周期的使能状态, 用来检测使能上升沿


static float CalcDt(void) {
  static float last_time;
  float now = DWT_GetTimeline_s();
  float dt = now - last_time;
  last_time = now;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.001f;  // 首周期或异常时退化为标称值(1kHz), 避免大跳变
  }
  return dt;
}

static void PitchLimit(void) {
  if (gimbal_ctrl_cmd->pitch > PITCH_MAX_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MAX_ANGLE;
  } else if (gimbal_ctrl_cmd->pitch < PITCH_MIN_ANGLE) {
    gimbal_ctrl_cmd->pitch = PITCH_MIN_ANGLE;
  }
}

static void RobotCMDTask(float dt) {
    static uint16_t init_hold = 0;
    if (init_hold < 100) {
        init_hold++;
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        gimbal_enabled = 0;
        return;
    }
  /*失能（保护）: 右拨杆[下] 或遥控器掉线 -> 失能, 电机断电 */
  if (!RemoteControlIsOnline() ||switch_is_down(rc_data[TEMP].rc.switch_right) || switch_is_off(rc_data[TEMP].rc.switch_right)) {
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
      gimbal_enabled = 0;
    return;
  }
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;

  /* 把 yaw和pitch 目标对齐当前姿态 */
  if (!gimbal_enabled) {
      gimbal_ctrl_cmd->yaw = robot->gimbal->gimbal_IMU_data->YawTotalAngle;
      gimbal_ctrl_cmd->pitch =
        float_constrain(robot->gimbal->gimbal_IMU_data->Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
      gimbal_enabled = 1;
  }

  /* 摇杆 -> 目标角: 速度式, 摇杆大小就是目标角速度, 松杆即停 */
  int rocker_yaw = rc_data[TEMP].rc.rocker_l_;    // 左摇杆 水平
  int rocker_pitch = rc_data[TEMP].rc.rocker_l1;  // 左摇杆 竖直
  if (abs(rocker_yaw) > STICK_DEADBAND) {
    gimbal_ctrl_cmd->yaw += -YAW_STICK_SPEED / RC_ROCKER_MAX * (float)rocker_yaw * dt;
  }
  if (abs(rocker_pitch) > STICK_DEADBAND) {
    gimbal_ctrl_cmd->pitch += -PITCH_STICK_SPEED / RC_ROCKER_MAX * (float)rocker_pitch * dt;
  }
  /* pitch限位 */
  PitchLimit();
}


void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F407xx
  robot->rc_data = RemoteControlInit(&huart3);
#elif defined(STM32H723XX)
  robot->rc_data = RemoteControlInit(&huart5);
#endif
  rc_data = robot->rc_data;

  /* 只初始化云台: GimbalInit 内部会把 yaw/pitch 的反馈源设成 IMU 并用陀螺仪闭环 */
  robot->gimbal = GimbalInit(&gimbal_init_config);
  gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;

  /* 上电默认失能, 等操作手拨杆使能; 第一次使能时目标角会对齐当前姿态 */
  gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
  gimbal_ctrl_cmd->yaw = 0.0f;
  gimbal_ctrl_cmd->pitch = 0.0f;
    gimbal_enabled = 0;
}


void RobotTask(void) {
  float dt = CalcDt();

  RobotCMDTask(dt);
  GimbalTask();
}