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

// ==================== 电机配置文件 (基于 debug 已调优参数) ====================
// 默认使用双闭环配置，支持角度跟随；力反馈模式下直接下发力矩指令

// 小roll电机 (DM4310) - ID: 0x01, MasterID: 0x11
static Motor_Init_Config_s DM4310_config_1 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 9.5f,
                          .Ki = 0.00f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.0f},
            .speed_PID = {.Kp = 0.38f,
                          .Ki = 0.05f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.5f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .motor_type = J4310,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 0x01,
    .can_init_config.rx_id = 0x11,
};

// 小pitch电机 (DM4310) - ID: 0x02, MasterID: 0x12
static Motor_Init_Config_s DM4310_config_2 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 9.5f,
                          .Ki = 0.00f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.0f},
            .speed_PID = {.Kp = 0.38f,
                          .Ki = 0.05f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.5f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .motor_type = J4310,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 0x02,
    .can_init_config.rx_id = 0x12,
};

// 大pitch电机 (DM4310) - ID: 0x03, MasterID: 0x13
static Motor_Init_Config_s DM4310_config_3 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 9.5f,
                          .Ki = 0.00f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.0f},
            .speed_PID = {.Kp = 0.38f,
                          .Ki = 0.05f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.5f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = J4310,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 0x03,
    .can_init_config.rx_id = 0x13,
};

// 大roll电机 (DM4310) - ID: 0x04, MasterID: 0x14
static Motor_Init_Config_s DM4310_config_4 = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 9.5f,
                          .Ki = 0.00f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.0f},
            .speed_PID = {.Kp = 0.38f,
                          .Ki = 0.05f,
                          .Kd = 0.00f,
                          .MaxOut = 8.0f,
                          .DeadBand = 0.01f,
                          .Improve = PID_Integral_Limit,
                          .IntegralLimit = 0.5f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = J4310,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 0x04,
    .can_init_config.rx_id = 0x14,
};

// 大yaw电机 (DJI 6020)
static Motor_Init_Config_s M6020_config = {
    .controller_param_init_config =
        {
            .angle_PID = {.Kp = 90.0f,
                          .Ki = 0.0f,
                          .Kd = 0.3f,
                          .IntegralLimit = 0.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 5000.0f},
            .speed_PID = {.Kp = 3.0f,
                          .Ki = 40.0f,
                          .Kd = 0.0f,
                          .IntegralLimit = 4650.0f,
                          .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 13500.0f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = GM6020,
    .can_init_config.can_handle = &hcan1,
    .can_init_config.tx_id = 1,
};

#endif // ROBOT_CONFIG_H
