/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"

// static Motor_Init_Config_s J8009P_config = {
//     .controller_param_init_config =
//         {
//             .angle_PID = {.Kp = 20.0f,
//                           .Ki = 0.0f,
//                           .Kd = 0.0f,
//                           .IntegralLimit = 5.0f,
//                           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                           .MaxOut = 20.0f},
//             .speed_PID = {.Kp = 5.0f,
//                           .Ki = 0.0f,
//                           .Kd = 0.05f,
//                           .IntegralLimit = 5.0f,
//                           .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                           .MaxOut = 20.0f},
//         },
//     .controller_setting_init_config =
//         {
//             .angle_feedback_source = MOTOR_FEED,
//             .speed_feedback_source = MOTOR_FEED,
//             .outer_loop_type = SPEED_LOOP,
//             .close_loop_type = SPEED_LOOP,
//         },
//     .motor_type = J8009P,
//     .can_init_config.can_handle = &hcan1,
//     .can_init_config.tx_id = 1,
//     .can_init_config.rx_id = 0,
//     .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
// };

static Motor_Init_Config_s M3508_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 800.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 1000.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 1900.0f},
            .speed_PID = {.Kp = 5.0f,
                          .Ki = 5.1f,
                          .Kd = 0.0f,
                          .IntegralLimit = 12000,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 24000},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
    .motor_type = GM6020,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 2,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};
