#pragma once

#include "robot.h"

/* H7 hardware mapping. The DM master ID is usually 0 and the motor ID is 1. */
#define TEST_MOTOR_CAN_HANDLE (&hcan2)
#define TEST_MOTOR_TX_ID 0x01u
#define TEST_MOTOR_RX_ID 0x00u

#define RC_ROCKER_MAX 660.0f
#define RC_ROCKER_DEADBAND 20
#define TEST_MOTOR_MAX_SPEED 5.0f   // rad/s
#define TEST_MOTOR_MAX_TORQUE 2.0f  // N*m

static Motor_Init_Config_s test_motor_config = {
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 0.35f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = TEST_MOTOR_MAX_TORQUE,
                    .DeadBand = 0.05f,
                    .Improve = PID_Derivative_On_Measurement,
                },
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = J4310,
    .can_init_config =
        {
            .can_handle = TEST_MOTOR_CAN_HANDLE,
            .tx_id = TEST_MOTOR_TX_ID,
            .rx_id = TEST_MOTOR_RX_ID,
        },
};
