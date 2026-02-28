/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   自定义控制器机器人配置文件
 ******************************************************************************
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "motor_task.h"
#include "custom_controller.h"

// DM4310电机配置
static Motor_Init_Config_s DM4310_config = {
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

// 2006电机配置
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
    .motor_type = M2006,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 4,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

// 电位器配置 - 3.3V参考电压对应0-360度映射
static Potentiometer_Config_s POT_config = {
    .min_voltage = 0.0f,        // 最小电压 0V
    .max_voltage = 3.3f,        // 最大电压 3.3V (对应360度)
    .min_angle = 0.0f,          // 对应最小角度 0度
    .max_angle = 360.0f,        // 对应最大角度 360度
    .filter_alpha = 0.1f,       // 滤波系数 0.1 (轻微滤波)
};

#endif // ROBOT_CONFIG_H