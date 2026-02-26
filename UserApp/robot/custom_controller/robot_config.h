/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "can_comm.h"
#include "crc8.h"

// DM4310电机配置
static Motor_Init_Config_s DM4310_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 0.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 0.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 10.0f},
            .speed_PID = {.Kp = 0.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 8.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 28.0f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
    .motor_type = J4310,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 0x01,
    .can_init_config.rx_id = 0x11,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

// 第一个3508电机配置
static Motor_Init_Config_s M3508_config_1 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 0.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 5.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 20.0f},
            .speed_PID = {.Kp = 0.0f,
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
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 1,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

// 第二个3508电机配置
static Motor_Init_Config_s M3508_config_2 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 0.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 5.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 20.0f},
            .speed_PID = {.Kp = 0.0f,
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
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 2,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

// 第三个3508电机配置
static Motor_Init_Config_s M2006_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 0.0f,
                          .Ki = 0.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 5.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 20.0f},
            .speed_PID = {.Kp = 0.0f,
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
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 4,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

