#pragma once

#include "dji_motor.h"

/* ===================== 运动参数 ===================== */

/* 正常运行速度，单位 m/s */
#define CONVEYOR_TARGET_MPS 1.0f

/* 两侧机械挡板之间的实际距离，单位 m。
 * 只测一次；保持 0 时禁止正常往返。 */
#define CONVEYOR_STOP_TO_STOP_M 4.95f

/* 正常运行时距离软限位的最小安全距离，单位 m */
#define CONVEYOR_SAFE_MARGIN_M 0.8f

/* 加、减速共用，单位 m/s² */
#define CONVEYOR_ACCEL_MPS2 1.0f

/* 上电找左右挡板时的低速，单位转子 °/s */
#define CONVEYOR_HOME_SPEED_DPS 7500.0f

/* ===================== M3508 ===================== */

static Motor_Init_Config_s conveyor_motor_config = {
  .controller_param_init_config = {
    .speed_PID = {
      .Kp = 3.0f,
      .Ki = 0.05f,
      .Kd = 0.0f,
      .IntegralLimit = 3000.0f,
      .Improve = PID_Integral_Limit,
      .MaxOut = 8000.0f,
  },
},
.controller_setting_init_config = {
  .angle_feedback_source = MOTOR_FEED,
  .speed_feedback_source = MOTOR_FEED,
  .outer_loop_type = SPEED_LOOP,
  .close_loop_type = SPEED_LOOP,
  .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
  .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
},
.motor_type = M3508,
.can_init_config = {
  .can_handle = &hcan1, // CAN1
  .tx_id = 1,           // C620 拨码 ID 1
},
};