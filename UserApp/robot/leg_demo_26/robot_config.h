#pragma once

#include "general_def.h"
#include "robot.h"

#define JOINT_REF_RATE 0.8f
#define JOINT_LOCK_TOLERANCE 0.08f

#define BALANCE_START_ANGLE (8.0f * DEGREE_2_RAD)
#define BALANCE_STOP_ANGLE (25.0f * DEGREE_2_RAD)

#define JOINT_MOTOR_CONFIG(can, tx, rx)                                                                 \
  {                                                                                                     \
      .controller_param_init_config =                                                                   \
          {                                                                                             \
              .angle_PID =                                                                              \
                  {                                                                                     \
                      .Kp = 8.0f,                                                                      \
                      .Ki = 0.0f,                                                                       \
                      .Kd = 0.08f,                                                                       \
                      .MaxOut = 12.5f,                                                                   \
                      .DeadBand = 0.01f,                                                                \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |                         \
                                 PID_Derivative_On_Measurement,                                         \
                      .IntegralLimit = 6.0f,                                                            \
                  },                                                                                    \
              .speed_PID =                                                                              \
                  {                                                                                     \
                      .Kp = 0.65f,                                                                       \
                      .Ki = 0.1f,                                                                       \
                      .Kd = 0.007f,                                                                      \
                      .MaxOut = 7.0f,                                                                   \
                      .DeadBand = 0.05f,                                                                \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |                         \
                                 PID_Derivative_On_Measurement,                                         \
                      .IntegralLimit = 0.0f,                                                            \
                  },                                                                                    \
          },                                                                                            \
      .controller_setting_init_config =                                                                 \
          {                                                                                             \
              .outer_loop_type = ANGLE_LOOP,                                                            \
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                               \
              .angle_feedback_source = MOTOR_FEED,                                                      \
              .speed_feedback_source = MOTOR_FEED,                                                      \
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,                                             \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,                                       \
          },                                                                                            \
      .motor_type = J4310,                                                                               \
      .can_init_config =                                                                                 \
          {                                                                                             \
              .can_handle = can,                                                                        \
              .tx_id = tx,                                                                              \
              .rx_id = rx,                                                                              \
          },                                                                                            \
  }

#define WHEEL_MOTOR_CONFIG(can, tx, rx)                         \
  {                                                             \
      .controller_setting_init_config =                         \
          {                                                     \
              .outer_loop_type = OPEN_LOOP,                     \
              .close_loop_type = OPEN_LOOP,                     \
              .angle_feedback_source = MOTOR_FEED,              \
              .speed_feedback_source = MOTOR_FEED,              \
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,     \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, \
          },                                                    \
      .motor_type = H6215,                                      \
      .can_init_config =                                        \
          {                                                     \
              .can_handle = can,                                \
              .tx_id = tx,                                      \
              .rx_id = rx,                                      \
          },                                                    \
  }

#define LEG_INIT_CONFIG(joint_can_0, joint_tx_0, joint_rx_0, joint_can_1, joint_tx_1, joint_rx_1, wheel_can, \
                        wheel_tx, wheel_rx)                                                                    \
  {                                                                                                           \
      .cali_mode = LEG_PRE_CALI_MODE,                                                                         \
      .joint_motor_config =                                                                                   \
          {                                                                                                   \
              [0] = JOINT_MOTOR_CONFIG(joint_can_0, joint_tx_0, joint_rx_0),                                  \
              [1] = JOINT_MOTOR_CONFIG(joint_can_1, joint_tx_1, joint_rx_1),                                  \
          },                                                                                                  \
      .wheel_motor_config = WHEEL_MOTOR_CONFIG(wheel_can, wheel_tx, wheel_rx),                                \
  }

static const float joint_target[LEG_COUNT][2] = {
    [LEG_LEFT] = {-0.22f, 0.55f},
    [LEG_RIGHT] = {0.22f, -0.55f},
};

static Chassis_Init_Config_s chassis_init_config = {
    .leg_init_config =
        {
            [LEG_LEFT] = LEG_INIT_CONFIG(&hcan1, 0x05, 0x04, &hcan1, 0x07, 0x06, &hcan1, 0x01, 0x00),
            [LEG_RIGHT] = LEG_INIT_CONFIG(&hcan2, 0x09, 0x08, &hcan2, 0x0B, 0x0A, &hcan2, 0x01, 0x00),
        },
    .imu_init_config =
        {
            .flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = 0.0f,
            .Pitch = 180.0f,
            .Roll = 0.0f,
            .CenterOffset = {0.15413f, 0.04612f, 0.09348f},
            .GyroOffset = {0.00708952406f, 0.00323308632f, 0.00078589347f},
            .offset_flag = 1,
        },
};

#undef LEG_INIT_CONFIG
#undef WHEEL_MOTOR_CONFIG
#undef JOINT_MOTOR_CONFIG
