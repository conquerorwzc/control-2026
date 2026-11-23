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
#define MOTOR_NUM 7u  // 7台全部握手/回零;
#define MOTOR_MAX 7u

#if MOTOR_NUM > GRAB_MOTOR_NUM
#error "MOTOR_NUM 超过组件上限 GRAB_MOTOR_NUM (grab.h)"
#endif

#define ROCKER_DEADZONE 10  // 摇杆死区(满量程±660)

/* ===== 摇杆映射: 回中=目标角度0, 满量程(±660)=±CMD_ANGLE_MAX_x, 单位 rad.
   左sw上档驱动0~3号, 中档驱动4~6号(DOWN 不映射) ===== */
#define CMD_ANGLE_MAX_0 1.6f  // 左摇杆上下 → 0号(J4340)
#define CMD_ANGLE_MAX_1 1.6f  // 左摇杆左右 → 1号(J4310)
#define CMD_ANGLE_MAX_2 1.6f  // 右摇杆上下 → 2号(J8009P)
#define CMD_ANGLE_MAX_3 1.6f  // 右摇杆左右 → 3号(J4310)
#define CMD_ANGLE_MAX_4 1.6f  // 中档: 左摇杆上下 → 4号(J4310)
#define CMD_ANGLE_MAX_5 1.6f  // 中档: 左摇杆左右 → 5号(J4340)
#define CMD_ANGLE_MAX_6 1.6f  // 中档: 右摇杆上下 → 6号(J8009P)

/* ===== Homing(上电/re-arm 后的可控回零) ===== */
#define HOMING_SPEED 0.3f  // 回零参考斜率 rad/s. 最坏|ref0|≈12.5 → 最长约42s
#define HOMING_TOL 0.01f   // 到点阈值 rad(≈3°), 全部电机到点才 HOMING→RUN

/* ===== 单台调参目标: 测试信号(摆动/方波)只驱动这一台, 其余 homing 到 0 后原地保持.
   调完删掉本宏即恢复全台测试 ===== */
//#define TUNE_MOTOR_IDX 6  // 调 [5] J4340 速度环 (tx=0x0A/rx=0x0B), cmd_angle[5] 此时承载速度目标 rad/s

/* ===== 正弦摆动测试===== */
#define SWING_TEST 1
#define SWING_FREQ_HZ 0.75f                                // 摆动频率, 0.5Hz = 周期2s.
#define SWING_AMP_0 0.50f
#define SWING_AMP_1 0.50f
#define SWING_AMP_2 0.50f
#define SWING_AMP_3 0.50f
#define SWING_AMP_4 0.50f
#define SWING_AMP_5 0.50f
#define SWING_AMP_6 0.50f
#define SWING_OMEGA (2.0f * 3.14159265f * SWING_FREQ_HZ)  // 角频率 rad/s

/* ===== 方波阶跃测试 =====
   与 SWING_TEST 互斥, 都置0走遥控 CmdMap.
*/
#define SQUARE_TEST 0
#define SQUARE_FREQ_HZ 0.25f
#define SQUARE_AMP_0 0.5f
#define SQUARE_AMP_1 0.5f
#define SQUARE_AMP_2 0.5f
#define SQUARE_AMP_3 0.5f
#define SQUARE_AMP_4 0.5f
#define SQUARE_AMP_5 0.5f
#define SQUARE_AMP_6 0.5f

#if SWING_TEST && SQUARE_TEST
#error "SWING_TEST 与 SQUARE_TEST 互斥: 二选一, 或都置0走遥控 CmdMap"
#endif


static Grab_Init_Config_s grab_init_config = {
    .motor_num = MOTOR_NUM,
    .grab_param =
        {
            .homing_speed = HOMING_SPEED,
            .homing_tol = HOMING_TOL,
        },
    .motor_config =
        {
    [0] =
        {
            .motor_type = J4340,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x01,
                    .rx_id = 0x00,
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
    [1] =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x02,
                    .rx_id = 0x03,
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
    [2] =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x04,
                    .rx_id = 0x05,
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
    [3] =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x06,
                    .rx_id = 0x07,
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
    [4] =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x08,
                    .rx_id = 0x09,
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
    [5] =
        {
            .motor_type = J4340,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x0A,
                    .rx_id = 0x0B,
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
    [6] =
        {
            .motor_type = J8009P,
            .can_init_config =
                {
                    .can_handle = &hfdcan1,
                    .tx_id = 0x0C,
                    .rx_id = 0x0D,
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
};
