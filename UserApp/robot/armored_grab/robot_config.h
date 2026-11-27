/**
 ******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
 ******************************************************************************
 */
#pragma once

#include "grab.h"
#include "robot.h"

/* ===== 数量与映射参数 ===== */
#define MOTOR_NUM 7u  // 7台全部握手/锚定;
#define MOTOR_MAX 7u

#if MOTOR_NUM > GRAB_MOTOR_NUM
#error "MOTOR_NUM 超过组件上限 GRAB_MOTOR_NUM (grab.h)"
#endif

#define ROCKER_DEADZONE 10  // 摇杆死区(满量程±660)

/* ===== 摇杆增量映射:目标以 CMD_RATE_x rad/s 移动(满速率),
   左sw上档驱动0~3号, 中档驱动4~6号(DOWN 不映射) ===== */
#define CMD_RATE_0 1.5f       // 左摇杆上下 → 0号(J4340P) 满推杆目标速度 rad/s
#define CMD_RATE_1 1.5f       // 左摇杆左右 → 1号(J8009P)
#define CMD_RATE_2 1.5f       // 右摇杆上下 → 2号(J4340)
#define CMD_RATE_3 1.5f       // 右摇杆左右 → 3号(J4310)
#define CMD_RATE_4 1.5f       // 中档: 左摇杆上下 → 4号(J4310)
#define CMD_RATE_5 1.5f       // 中档: 左摇杆左右 → 5号(J4310)
#define CMD_RATE_6 1.5f       // 中档: 右摇杆上下 → 6号(J4310)


static Grab_Init_Config_s grab_init_config = {
    .grab_param =
        {
            .motor_num = 7,
        },
    .joint =
        {
    /* ID 规划: 控制帧 tx_id = 0x01~0x07, 反馈帧 rx_id(master) = 0x11~0x17
       电机侧须在上位机同步配置 CAN_ID / MASTER_ID, 与本表槽位一一对应 */
    [0] =
        {
            .reduction = 1.0f,  // 0号(J4340P): 带轮比待实测, 符号含方向
            .limit = 1.6f,
            .motor_config = {
                .motor_type = J4340,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x01,
                        .rx_id = 0x11,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID = {.Kp = 5.0f,
                                      .Ki = 10.0f,
                                      .Kd = 0.0f,
                                      .MaxOut = DM_T_MAX_J4340,
                                      .Improve = PID_Integral_Limit,
                                      .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 40,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 150,
                                .MaxOut = 5.0f,
                            },
                    },
            },
        },
    [1] =
        {
            .reduction = 1.0f,  // 1号(J8009P)
            .limit = 1.6f,
            .motor_config = {
                .motor_type = J8009P,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x02,
                        .rx_id = 0x12,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID =
                            {.Kp = 2.0f,
                                .Ki = 20.0f,
                                .Kd = 0.0f,
                                .DeadBand = 0.1f,
                                .MaxOut = DM_T_MAX_J8009P,
                                .Improve = PID_Integral_Limit,
                                .IntegralLimit = 15.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 35.0f,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 5.0f,
                                .MaxOut = 15.0f,
                            },
                    },
            },
        },
    [2] =
        {
            .reduction = 1.0f,  // 2号(J4340)
            .limit = 1.6f,
            .motor_config = {
                .motor_type = J4340,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x03,
                        .rx_id = 0x13,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID =
                            {
                                .Kp = 2.5f,
                                .Ki = 40.0f,
                                .Kd = 0.0f,
                                .DeadBand = 0.05f,
                                .MaxOut = DM_T_MAX_J4340,
                                .Improve = PID_Integral_Limit,
                                .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 35,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 150,
                                .MaxOut = 5.0f,
                            },
                    },
            },
        },
    [3] =
        {
            .reduction = 1.0f,  // 3号(J4310)
            .limit = 3.14f,
            .motor_config = {
                .motor_type = J4310,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x04,
                        .rx_id = 0x14,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID = {.Kp = 0.2f,
                                      .Ki = 0.8f,
                                      .Kd = 0.0f,
                                      .MaxOut = DM_T_MAX_J4310,
                                      .Improve = PID_Integral_Limit,
                                      .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 50,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 5.0f,
                                .MaxOut = 15.0f,
                            },
                    },
            },
        },
    [4] =
        {
            .reduction = 1.0f,  // 4号(J4310)
            .limit = 3.14f,
            .motor_config = {
                .motor_type = J4310,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x05,
                        .rx_id = 0x15,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID = {.Kp = 0.2f,
                                      .Ki = 0.8f,
                                      .Kd = 0.0f,
                                      //.DeadBand = 0.05f,
                                      .MaxOut = DM_T_MAX_J4310,
                                      .Improve = PID_Integral_Limit,
                                      .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 50,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 5.0f,
                                .MaxOut = 15.0f,
                            },
                    },
            },
        },
    [5] =
        {
            .reduction = 1.0f,  // 5号(J4310)
            .limit = 3.14f,
            .motor_config = {
                .motor_type = J4310,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x06,
                        .rx_id = 0x16,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID = {.Kp = 0.2f,
                                      .Ki = 0.8f,
                                      .Kd = 0.0f,
                                      //.DeadBand = 0.05f,
                                      .MaxOut = DM_T_MAX_J4310,
                                      .Improve = PID_Integral_Limit,
                                      .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 50,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 5.0f,
                                .MaxOut = 15.0f,
                            },
                    },
            },
        },
    [6] =
        {
            .reduction = 1.0f,  // 6号(J4310)
            .limit = 3.14f,
            .motor_config = {
                .motor_type = J4310,
                .can_init_config =
                    {
                        .can_handle = &hfdcan1,
                        .tx_id = 0x07,
                        .rx_id = 0x17,
                    },
                .controller_setting_init_config =
                    {
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .outer_loop_type = ANGLE_LOOP,
                        .angle_feedback_source = MOTOR_FEED,
                        .speed_feedback_source = MOTOR_FEED,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 装反方向时改 MOTOR_DIRECTION_REVERSE
                    },
                .controller_param_init_config =
                    {
                        .speed_PID = {.Kp = 0.2f,
                                      .Ki = 0.8f,
                                      .Kd = 0.0f,
                                      //.DeadBand = 0.05f,
                                      .MaxOut = DM_T_MAX_J4310,
                                      .Improve = PID_Integral_Limit,
                                      .IntegralLimit = 5.0f},
                        .angle_PID =
                            {
                                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                .Kp = 50,
                                .Ki = 0,
                                .Kd = 0,
                                .DeadBand = 0,
                                .IntegralLimit = 5.0f,
                                .MaxOut = 15.0f,
                            },
                    },
            },
        },
    },
};
