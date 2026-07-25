#pragma once

#include "robot.h"

/* H7 hardware mapping. The DM master ID is usually 0 and the motor ID is 1. */


static Motor_Init_Config_s test_motor_config = {
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 0.7f,
                    .Ki = 0.1f,
                    .Kd = 0.0f,
                    .MaxOut = 7.0f,
                    .DeadBand = 0.05f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
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
            .can_handle = &hcan1,
            .tx_id = 0x05,
            .rx_id = 0x06,
        },
};
