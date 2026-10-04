#pragma once

#include "robot.h"

#define GIMBAL_BOARD
#define ALIGN_DEG (-17.0f)
static Gimbal_Init_Config_s gimbal_init_config = {

    .gravity_config = {
        .big_pitch_scale_nm = 0.35f,    // N*m，
        .small_pitch_scale_nm = 0.03f,  // N*m，末端偏心重力矩系数，同时计入大pitch
        .big_pitch_zero_rad = 2.5f,    // rad，两轴连线水平的大轴反馈角，
        .small_pitch_zero_rad = 3.5f,  // rad，大轴处于上述零位时末端重心方向水平的小轴反馈角
        .big_pitch_direction = -1.0f,    // 正向直连为 1，反向直连为 -1
        .small_pitch_direction = 1.0f,
    },
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
                            .Kp = 20.0f,         // (rad/s)/rad = 1/s
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .MaxOut = 25.0f,     // rad/s，串级环的速度目标限幅
                            .DeadBand = 0.01f,   //
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.05f,  // rad/s，
                        },
                    .speed_PID =
                        {
                            .Kp = 0.81f,         // N*m/(rad/s)
                            .Ki = 4.63f,
                            .Kd = 0.00f,
                            .MaxOut = 3.0f,     // N*m，目前直接采用4310额定
                            .DeadBand = 0.0f,
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                            .IntegralLimit = 0.35f,  // N*m，
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
                            .Kp = 0.7f,  //
                            .Ki = 3.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 3.0f,     //4310额定
                            .IntegralLimit = 0.35f,  // N*m
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral ,
                        },
                    .angle_PID =
                        {
                            .Kp = 18.0f,
                            .Ki = 0.1f,
                            .Kd = 0.6f,
                            .DeadBand = 0.01f,  // rad，约 0.286 deg
                            .MaxOut = 4.0f,  // rad/s 出现抖动可以减小maxout，因为直接切换角度目标会导致速度尖峰
                            .IntegralLimit = 0.5f,
                            .Improve = PID_Integral_Limit |PID_Derivative_On_Measurement ,
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
                            .Kp = 0.5f,  // N*m/(rad/s)
                            .Ki = 6.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 1.5f,  // N*m，4310额定力矩3NM
                            .IntegralLimit = 0.25f,  // N*m，
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral   //如有抖动减小kpki，加入输出低通滤波

                        },
                    .angle_PID =
                        {
                            .Kp = 0.30f,  // (rad/s)/deg，IMU Pitch 反馈为 deg
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.03f,
                            .MaxOut = 3.2f,  // rad/s；0.05 rad、2.5 Hz 的理想速度峰值为 0.785 rad/s
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
                            .Kp = 1600.0f,  // 电流指令码/(rad/s)， 抑制高频比例输出
                            .Ki = 10000.0f,  // 电流指令码/rad；积分使用 dt（秒）
                            .Kd = 0.0f,
                            .DeadBand = 0.0f,
                            .MaxOut = 16384.0f,  // CAN 电流指令码，约 +/-1.6 A；上限 16384 对应 3 A
                            .IntegralLimit = 2000.0f,  // 约 0.366 A；
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_OutputFilter,
                            .Output_LPF_RC = 0.002f,  // 秒；一阶输出低通
                        },
                    .angle_PID =
                        {
                            .Kp = 1.0f,  // (rad/s)/deg，
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.01f,
                            .MaxOut = 6.0f,  // rad/s；
                            .IntegralLimit = 0.0f,
                            .Improve = PID_IMPROVE_NONE,
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
            .Kp = 0.0035f,
            .Ki = 0.000f,
            .Kd = 0.000f,
            .DeadBand = 0.002f,
            .MaxOut = 10.0f,
            .IntegralLimit = 1.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        },
};