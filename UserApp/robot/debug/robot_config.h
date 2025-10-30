/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"

static Motor_Init_Config_s wheel_motor_config = {
    .controller_param_init_config =
        {
            .speed_PID = {.Kp = 100.0f,
                          .Ki = 0.1f,
                          .Kd = 0.0f,
                          .IntegralLimit = 3000.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 12000.0f},
            .current_PID =
                {
                    .Kp = 0.5f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 15000.0f,
                },
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
};