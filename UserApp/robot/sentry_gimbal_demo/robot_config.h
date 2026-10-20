#pragma once

#include "robot.h"

#define GIMBAL_BOARD
#define ALIGN_DEG (-17.0f)  // deg，小 Yaw 机械对齐零位
/*
| 对象 | 角度反馈 | 速度反馈 | 速度环输出 |
| 大 Yaw：DM-J4310 | 电机累计角度，rad | 电机速度，rad/s | DM 力矩指令 |
| 大 Pitch：DM-J4310 | 电机累计角度，rad | 电机速度，rad/s | DM 力矩指令 |
| 小 Yaw：GM6020 | IMU 累计 Yaw，° | `Gyro[2]`，rad/s | GM6020 电流指令数值 |
| 小 Pitch：DM-J4310 | IMU Pitch，° | `Gyro[0]`，rad/s | DM 力矩指令 |
 *
 */
static Gimbal_Init_Config_s gimbal_init_config = {
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
                            .Kp = 0.7f,              // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 3.0f,              // N*m/rad，速度误差积分增益
                            .Kd = 0.0f,              // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 3.0f,          // N*m，速度环输出力矩限幅,4310额定3，峰值7
                            .IntegralLimit = 0.35f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 18.0f,            // (rad/s)/rad = 1/s，角度误差到目标速度的比例增益
                            .Ki = 0.1f,             // (rad/s)/(rad*s) = 1/s^2，角度误差积分增益
                            .Kd = 0.6f,             // (rad/s)/(rad/s)，无量纲，角度误差微分增益；启用微分先行
                            .DeadBand = 0.005f,      // rad，约 0.573 deg，角度误差死区
                            .MaxOut = 4.0f,         // rad/s，目标速度限幅；可减小以抑制目标切换时的速度尖峰
                            //todo：lsy要求速度还要再快一点，思考一下怎么调参，因为两个角度直接切换目标突变太大导致会有一个尖峰，建议对速度规划/加一个斜坡去优化
                            .IntegralLimit = 0.5f,  // rad/s，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
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
                            .Kp = 0.5f,              // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 1.5f,              // N*m/rad，速度误差积分增益
                            .Kd = 0.0f,              // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 1.5f,          // N*m，速度环输出力矩限幅
                            .IntegralLimit = 0.25f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 0.50f,        // (rad/s)/deg，IMU 角度误差到目标速度的比例增益
                            .Ki = 0.0f,         // (rad/s)/(deg*s)，角度误差积分增益
                            .Kd = 0.0f,         // (rad/s)/(deg/s)，角度误差微分增益
                            .DeadBand = 0.01f,  // deg，IMU 角度误差死区
                            .MaxOut = 3.2f,     // rad/s，目标速度限幅；0.05 rad、2.5 Hz 的理想速度峰值为 0.785 rad/s
                            .IntegralLimit = 0.0f,        // rad/s，积分输出限幅
                            .Improve = PID_IMPROVE_NONE,  // 无量纲，优化功能位标志
                        },
                },
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
                    .speed_PID =
                        {
                            .Kp = 0.81f,             // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 4.63f,             // N*m/rad，速度误差积分增益
                            .Kd = 0.00f,             // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 3.0f,          // N*m，速度环输出力矩限幅
                            .IntegralLimit = 0.35f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 20.0f,                    // (rad/s)/rad = 1/s，角度误差到目标速度的比例增益
                            .Kd = 0.0f,                     // (rad/s)/(rad/s)，无量纲，角度误差微分增益
                            .Ki = 0.0f,                     // (rad/s)/(rad*s) = 1/s^2，角度误差积分增益
                            .DeadBand = 0.0f,              // rad，角度误差死区
                            .MaxOut = 25.0f,                // rad/s，目标速度限幅
                            .IntegralLimit = 0.05f,         // rad/s，积分输出限幅
                            .Improve = PID_Integral_Limit,  // 无量纲，优化功能位标志
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
                            .Kp = 1600.0f,             // CAN 电流指令码/(rad/s)，速度误差比例增益
                            .Ki = 10000.0f,            // CAN 电流指令码/rad，速度误差积分增益
                            .Kd = 0.0f,                // CAN 电流指令码/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,          // rad/s，速度误差死区
                            .MaxOut = 12000.0f,        // CAN 电流指令码，速度环输出限幅（电机上限16384对应±3A）
                            .IntegralLimit = 2000.0f,  // CAN 电流指令码，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral |
                                       PID_OutputFilter,  // 无量纲，优化功能位标志
                            .Output_LPF_RC = 0.002f,      // s，一阶输出低通时间常数
                        },
                    .angle_PID =
                        {
                            .Kp = 1.0f,                   // (rad/s)/deg，IMU 角度误差到目标速度的比例增益
                            .Ki = 0.0f,                   // (rad/s)/(deg*s)，角度误差积分增益
                            .Kd = 0.0f,                   // (rad/s)/(deg/s)，角度误差微分增益
                            .DeadBand = 0.15f,            // deg，IMU 角度误差死区
                            .MaxOut = 6.0f,               // rad/s，目标速度限幅
                            .IntegralLimit = 0.0f,        // rad/s，积分输出限幅
                            .Improve = PID_IMPROVE_NONE,  // 无量纲，优化功能位标志
                        },
                },
        },
    .gravity_config =
        {
            .big_pitch_scale_nm = 0.35f,    // N*m，大轴近端重力矩系数，不含小轴末端偏心项
            .small_pitch_scale_nm = 0.03f,  // N*m，末端偏心重力矩系数，同时计入大 Pitch
            .big_pitch_zero_rad = 3.5f,     // rad，两轴连线水平时的大轴反馈角
            .small_pitch_zero_rad = 2.5f,   // rad，大轴处于上述零位时末端重心方向水平的小轴反馈角
            .big_pitch_direction = -1.0f,   // 无量纲，正向直连为 1，反向直连为 -1
            .small_pitch_direction = 1.0f,  // 无量纲，正向直连为 1，反向直连为 -1
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
    // Follow 输出直接累加到大 Yaw 目标，单位 deg/次，当前实现未在累加处乘 dt。
    .Yaw_Follow_PID =
        {
            .Kp = 0.0035f,          // (deg/次)/deg，偏角到每次目标角度增量的比例增益
            .Ki = 0.000f,           // (deg/次)/(deg*s)，偏角积分增益
            .Kd = 0.000f,           // (deg/次)/(deg/s)，偏角微分增益；启用微分先行
            .DeadBand = 0.3f,     // deg，小 Yaw 相对偏角死区
            //todo：陀螺仪零飘有点大没擦好导致follow环死区不开大大yaw会1hz左右来回0.几度动，建议重新擦一下零飘或者在线擦？
            .MaxOut = 10.0f,        // deg/次，每次目标角度增量限幅
            .IntegralLimit = 1.0f,  // deg/次，积分输出限幅
            .Improve =
                PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,  // 无量纲，优化功能位标志
        },
};
