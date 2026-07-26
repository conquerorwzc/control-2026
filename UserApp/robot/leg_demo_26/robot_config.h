#pragma once

#include "robot.h"

#define DM_COMM_MOTOR_CONFIG(motor, can, tx, rx)             \
  {                                                          \
      .controller_setting_init_config =                      \
          {                                                  \
              .outer_loop_type = OPEN_LOOP,                  \
              .close_loop_type = OPEN_LOOP,                  \
              .angle_feedback_source = MOTOR_FEED,           \
              .speed_feedback_source = MOTOR_FEED,           \
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,  \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL, \
          },                                                 \
      .motor_type = motor,                                   \
      .can_init_config =                                     \
          {                                                  \
              .can_handle = can,                             \
              .tx_id = tx,                                   \
              .rx_id = rx,                                   \
          },                                                 \
  }

#define LEG_INIT_CONFIG(joint_can_0, joint_tx_0, joint_rx_0, joint_can_1, joint_tx_1, joint_rx_1, wheel_can, \
                        wheel_tx, wheel_rx)                                                                    \
  {                                                                                                           \
      .cali_mode = LEG_PRE_CALI_MODE,                                                                         \
      .joint_motor_config =                                                                                   \
          {                                                                                                   \
              [0] = DM_COMM_MOTOR_CONFIG(J4310, joint_can_0, joint_tx_0, joint_rx_0),                         \
              [1] = DM_COMM_MOTOR_CONFIG(J4310, joint_can_1, joint_tx_1, joint_rx_1),                         \
          },                                                                                                  \
      .wheel_motor_config = DM_COMM_MOTOR_CONFIG(H6215, wheel_can, wheel_tx, wheel_rx),                       \
  }

static Chassis_Init_Config_s chassis_init_config = {
    .leg_init_config =
        {
            [LEG_LEFT] = LEG_INIT_CONFIG(&hcan1, 0x05, 0x04, &hcan1, 0x07, 0x06, &hcan1, 0x01, 0x00),
            [LEG_RIGHT] = LEG_INIT_CONFIG(&hcan2, 0x09, 0x08, &hcan2, 0x0B, 0x0A, &hcan2, 0x01, 0x00),
        },
};

#undef LEG_INIT_CONFIG
#undef DM_COMM_MOTOR_CONFIG
