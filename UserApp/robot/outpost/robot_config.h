//
// Created by PC on 2025/12/22.
//
#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

/* ================= 硬件 ID 配置 ================= */
#define OUTPOST_MOTOR_ID 1  // 3508 前哨站电机 (CAN1)

/* ================= 运动参数 ================= */
#define TARGET_SPEED  3700.0f

/* ================= 3508 前哨站电机配置 (速度环 + 位置环) ================= */
#define OUTPOST_3508_CONFIG(can_h, _id, _reverse)                                    \
  {                                                                                  \
      .motor_type = M3508,                                                           \
      .can_init_config = {.can_handle = can_h, .tx_id = _id},                        \
      .controller_setting_init_config = {                                            \
          .angle_feedback_source = MOTOR_FEED,                                       \
          .speed_feedback_source = MOTOR_FEED,                                       \
          .outer_loop_type = SPEED_LOOP,                                             \
          .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                \
          .motor_reverse_flag = _reverse,                                            \
          .feedback_reverse_flag = (_reverse == MOTOR_DIRECTION_REVERSE) ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL, \
      },                                                                             \
      .controller_param_init_config = {                                              \
          .speed_PID = {                                                             \
              .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.0f,                                    \
              .MaxOut = 16000.0f, .IntegralLimit = 8000.0f,                          \
              .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
          },                                                                         \
          .angle_PID = {                                                             \
              .Kp = 5.0f, .Ki = 0.0f, .Kd = 0.0f,                                    \
              .MaxOut = 10000.0f,                                                    \
              .Improve = PID_Integral_Limit,                                         \
          },                                                                         \
      },                                                                             \
  }

#endif