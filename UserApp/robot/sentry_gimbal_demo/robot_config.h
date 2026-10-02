#pragma once

#include "robot.h"

#define GIMBAL_BOARD
#define ALIGN_DEG (-17.0f)
static Gimbal_Init_Config_s gimbal_init_config = {
    .big_yaw_motor_config =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x4,
                    .rx_id = 0x14,
                },
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 9.0f,
                            .Ki = 0.0f,
                            .Kd = 0.1f,
                            .MaxOut = 25.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.2f,
                            .Kd = 0.00f,
                            .MaxOut = 11.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.5f,
                        },
                },
        },
    .big_pitch_motor_config =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x2,
                    .rx_id = 0x12,
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 0.8f,
                            .IntegralLimit = 0.0f,
                            .Improve = PID_IMPROVE_NONE,
                        },
                    .angle_PID =
                        {
                            .Kp = 8.0f,
                            .Ki = 0.0f,
                            .Kd = 0.1f,
                            .DeadBand = 0.01f,
                            .MaxOut = 5.0f,
                            .IntegralLimit = 0.0f,
                            .Improve = PID_IMPROVE_NONE,
                        },
                },
        },
    .small_pitch_motor_config =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x3,
                    .rx_id = 0x13,
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 0.6f,
                            .Ki = 0.2f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 1.6f,
                            .IntegralLimit = 0.0f,
                            .Improve = PID_IMPROVE_NONE,
                        },
                    .angle_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.005f,
                            .MaxOut = 6.4f,
                            .IntegralLimit = 0.0f,
                            .Improve = PID_IMPROVE_NONE,
                        },
                },
        },
    .small_yaw_motor_config =
        {
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.gm6020_control_mode = GM6020_CURRENT_CONTROL,
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 1000.0f,
                            .Ki = 30.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 15000.0f,
                            .IntegralLimit = 6000.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        },
                    .angle_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.04f,
                            .MaxOut = 15.0f,
                            .IntegralLimit = 1.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        },
                },
        },
    .imu_init_config =
        {
            .flag = 1,
            .offset_flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = -90.0f,
            .Pitch = 0.0f,
            .Roll = 0.0f,
            .GyroOffset = {-0.00184311043f, -0.00351727684f, 0.000647681532f},
        },
    .Yaw_Follow_PID =
        {
            .Kp = 0.003f,
            .Ki = 0.000f,
            .Kd = 0.000f,
            .DeadBand = 0.1f,
            .MaxOut = 10.0f,
            .IntegralLimit = 1.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        },
};