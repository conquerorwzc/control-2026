/**
******************************************************************************
* @file    robot_config.h
* @brief   飞镖机器人的硬件配置与机构参数, 集中管理所有实车相关参数
******************************************************************************
* @attention
* 带 TODO 的参数必须按实车实测后修改, 否则会出现方向相反/顶死/异常停机等问题。
* 硬件(全部挂在 CAN1): yaw 2006 id1, 同步带 3508 id2/id4, 扳机位置 3508 id3, 舵机 TIM1_CH1(PE9)
******************************************************************************
*/
#pragma once

#include "robot.h"

// 飞镖只有一块主控板,单板控制整车(所以不需要多板型的冲突检查)
#define ONE_BOARD

// 遥控器参数(robot.c 使用)
#define DART_RC_DIR_THRESHOLD 250         // 摇杆只看正负:偏出该阈值才算有方向指令
#define DART_STATUS_LOG_PERIOD_MS 1000.0f // 状态日志周期

// 同步带电机参数模板,两个电机在同一条同步带上,参数一致,只有id与装配方向不同
#define DART_BELT_MOTOR_CONFIG(handle, id, reverse)                                                                    \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .angle_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 0.0f,                                                                                   \
                        .Ki = 0.0f,                                                                                    \
                        .Kd = 0.0f,                                                                                    \
                        .IntegralLimit = 20000.0f,                                                                      \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,                                       \
                        .MaxOut = 40000.0f,                                                                             \
                    },                                                                                                 \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 1.0f,                                                                                    \
                        .Ki = 0.1f,                                                                                   \
                        .Kd = 0.0f,                                                                                    \
                        .IntegralLimit = 6000.0f,                                                                      \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,                                       \
                        .MaxOut = 16000.0f,                                                                            \
                    },                                                                                                 \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = ANGLE_AND_SPEED_LOOP,                                                               \
                .motor_reverse_flag = reverse,                                                                         \
                .feedback_reverse_flag = reverse,                                                                      \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
    })

static DartShoot_Init_Config_s dart_shoot_init_config = {
    // 同步带两个 3508 挂同一条同步带,使用同一个位置参考;右电机装配相反,反转标志与反馈标志一起取反
    // TODO 实测:摇杆前推时同步带方向相反,就把两个电机的 reverse 一起改成 MOTOR_DIRECTION_REVERSE
    .belt_motor_config =
        {
            DART_BELT_MOTOR_CONFIG(&hcan1, 2, MOTOR_DIRECTION_NORMAL),  // 同步带 3508(左)
            DART_BELT_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_REVERSE), // 同步带 3508(右)
        },
    // yaw 2006 位置环:角度环输出作为速度环参考,所以角度环 MaxOut 是速度限幅(2006 输出轴 2000/36 ≈ 56°/s)
    // TODO 实测:摇杆右推时 yaw 方向相反,就把下面两个 reverse 标志改成 MOTOR_DIRECTION_REVERSE
    .yaw_motor_config =
        {
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 1, // yaw 2006 id
                },
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 30.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .IntegralLimit = 1000.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                            .MaxOut = 30000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.2f,
                            .Kd = 0.0f,
                            .IntegralLimit = 3000.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                            .MaxOut = 10000.0f, // C610 电流指令上限
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_AND_SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = M2006,
        },
    // 扳机位置电机 3508 单环速度:只用于慢速调整扳机位置,摇杆回死区后组件直接让电机失能
    // TODO 实测:摇杆上推时扳机位置电机方向相反,就把下面的 motor_reverse_flag 改成 MOTOR_DIRECTION_REVERSE
    .trigger_motor_config =
        {
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 3, // 扳机位置 3508 id
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .IntegralLimit = 8000.0f,
                            .Improve = PID_Integral_Limit,
                            .MaxOut = 16000.0f,
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = M3508,
        },
    // 拉扳机的 PWM 舵机:PE9 = TIM1_CH1, 50Hz(舵机失能时 ServoStop() 会停掉整个 TIM1,该定时器只用于本舵机)
    .trigger_servo_config =
        {
            .servo_type = PWM_Servo,
            .pwm_init_config =
                {
                    .htim = &htim1,
                    .channel = TIM_CHANNEL_1,
                    .period = 0.02f,
                    .dutyratio = 0.0f,
                },
            .servo_id = 0, // 注册索引 0~6,不能与其他舵机冲突
        },
    // 机构参数
    .param =
        {
            // 位置累加:摇杆只决定方向,目标位置按下面的速率(单位:电机总角度/秒)乘控制周期累加
            // TODO 实测:速率不要超过对应角度环的 MaxOut(速度限幅),否则目标会一直跑在前面追不上
            .belt_pos_rate = 200.0f,       // 同步带累加速率(3508 输出轴 2000/19.2 ≈ 104°/s)
            .yaw_pos_rate = 150.0f,        // yaw 累加速率(2006 输出轴 1500/36 ≈ 42°/s)
            .trigger_speed = 100.0f,       // 扳机位置电机的速度参考(度/秒),摇杆偏出阈值时按它转
            // 270° 舵机: 行程 0~270° 对应 0.5ms~2.5ms, 135° 刚好是行程中点(1.5ms)
            // TODO 实测:左拨杆中档/上档对应的舵机角度,用 DartShootSetServoAngle() 扫描确定
            .servo_angle_mid = 135.0f,
            .servo_angle_up = 135.0f,
            .servo_min_pulse_s = 0.0005f, // 0° 对应脉宽(0.5ms)
            .servo_max_pulse_s = 0.0025f, // 270° 对应脉宽(2.5ms)
        },
};
