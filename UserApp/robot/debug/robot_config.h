/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "motor_def.h"
#include "robot.h"

static Motor_Init_Config_s J8009P_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 20.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 5.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 20.0f},
            .speed_PID = {.Kp = 5.0f,
                          .Ki = 0.0f,
                          .Kd = 0.05f,
                          .IntegralLimit = 5.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 20.0f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
    .motor_type = J8009P,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 1,
    .can_init_config.rx_id = 0,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

static Motor_Init_Config_s M3508_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 10.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 2000.0f},
            .speed_PID = {.Kp = 4.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 3000,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 12000},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 1,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
};
static Motor_Init_Config_s M3508_config_2 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 10.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 2000.0f},
            .speed_PID = {.Kp = 4.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 3000,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 12000},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan2,
    .can_init_config.tx_id = 2,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

static Motor_Init_Config_s M2006_config = {
    .controller_param_init_config =
        {
            .angle_PID =
                {
                    .Kp = 3.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = 35000.0f,
                },
            .speed_PID = {.Kp = 2.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .Improve = PID_Integral_Limit | PID_ErrorHandle,
                          .IntegralLimit = 0.0f,
                          .MaxOut = 20000.0},
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .motor_type = M2006,
    .can_init_config =
        {
            .can_handle = &hcan1,
            .tx_id = 3,
        },
};
static Motor_Init_Config_s M2006_config_2 = {
    .controller_param_init_config =
        {
            .angle_PID =
                {
                    .Kp = 3.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = 35000.0f,
                },
            .speed_PID = {.Kp = 2.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .Improve = PID_Integral_Limit | PID_ErrorHandle,
                          .IntegralLimit = 0.0f,
                          .MaxOut = 20000.0},
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .motor_type = M2006,
    .can_init_config =
        {
            .can_handle = &hcan1,
            .tx_id = 4,
        },
};

static Motor_Init_Config_s GM6020_config = {
    .controller_param_init_config =
        {
            .angle_PID =
                {
                    .Kp = 0.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .DeadBand = 0.1f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .IntegralLimit = 5.0f,
                    .MaxOut = 22.0f,
                },
            .speed_PID =
                {
                    .Kp = 0.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .IntegralLimit = 12000.0f,
                    .MaxOut = 25000.0f,
                },

        },
    .controller_setting_init_config =
    {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
    },
    .motor_type = GM6020,
    .can_init_config =
        {
            .can_handle = &hcan1,
            .tx_id = 5,
        },
};