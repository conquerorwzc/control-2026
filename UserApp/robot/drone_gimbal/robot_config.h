/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件 (无人机云台+发射)
******************************************************************************
*/
#pragma once

#include "robot.h"

#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message "Check robot_config.h parameters!"
#endif

#define ONE_BOARD
#define VISION_USE_VCP

// =============================================================
//                       1. 云台配置 (Gimbal)
//          电机: GM6020 x 2 | 逻辑: 串级 PID (英雄风格)
// =============================================================

// 云台机械中值与限位
#define YAW_CHASSIS_ALIGN_ECD 2689
#define PITCH_HORIZON_ECD 4677
#define YAW_ECD_MIN 2100
#define YAW_ECD_MAX 3300
#define RC_ESTOP_THRESHOLD 600    // 定义急停阈值 (摇杆最大值通常是 660)

// 云台初始化结构体
static Gimbal_Init_Config_s gimbal_init_config = {
    // --- Yaw 轴 (ID: 1, GM6020) ---
    .yaw_motor_config = {
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.3f,//1.2
                .Ki = 0.1f,
                .Kd = 0.03f,
                .MaxOut = 30.0f,
            },
            .speed_PID = {
                .Kp = 1200.0f,//1200
                .Ki = 25.0,//25.0
                .Kd = 0.0f,
                .MaxOut =  20000.0f,
                .IntegralLimit = 12000.0f,
            },
        },
        .motor_type = GM6020,
        .can_init_config = { .can_handle = &hcan1, .tx_id = 1 },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = OTHER_FEED, // IMU 反馈
            .speed_feedback_source = MOTOR_FEED, // 电机速度反馈
        },
    },

    // --- Pitch 轴 (ID: 5, GM6020) ---
    .pitch_motor_config = {
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.5f,       // 1.2
                .Ki = 0.0f,
                .Kd = 0.005f,
                .MaxOut = 25.0f, // 限制最大速度
            },
            .speed_PID = {
                .Kp = 2000.0f,    // 2750
                .Ki = 15.0f,     // 15
                .Kd = 0.0f,
                .MaxOut =  20000.0f,
                .IntegralLimit = 12000.0f,
            },
        },
        .motor_type = GM6020,
        .can_init_config = { .can_handle = &hcan1, .tx_id = 5 },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            // 开启串级双环控制
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    },

    .imu_init_config = { .flag = 1, .scale = {1,1,1}, .Yaw=0, .Pitch=0, .Roll=0 }
};

// =============================================================
//                       2. 发射机构配置 (Shoot)
//      摩擦轮: M3508 x 2 | 拨弹: M2006 x 1
// =============================================================

// 通用摩擦轮配置模板
#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
  ((Motor_Init_Config_s){                            \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 1.2f,                    \
                      .Ki = 0.1f,                    \
                      .Kd = 0.0f,                    \
                      .Improve = PID_Integral_Limit, \
                      .IntegralLimit = 10000.0f,     \
                      .MaxOut = 15000.0f,            \
                  },                                 \
          },                                         \
      .controller_setting_init_config =              \
          {                                          \
              .angle_feedback_source = MOTOR_FEED,   \
              .speed_feedback_source = MOTOR_FEED,   \
              .outer_loop_type = SPEED_LOOP,         \
              .close_loop_type = SPEED_LOOP,         \
              .motor_reverse_flag = direction,       \
          },                                         \
      .motor_type = M3508,                           \
      .can_init_config =                             \
          {                                          \
              .can_handle = handle,                  \
              .tx_id = id,                           \
          },                                         \
  })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = 45.0f,  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
            .reduction_ratio_loader = 36.0f,  // M2006拨盘电机的减速比
            .num_per_circle = 8,              // 拨盘一圈的装载量
            .loader_direction = -1,            // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                // 摩擦轮数量
            .friction_speed = 37000.0f,    // 设置目标转速
            .friction_coefficients[0] = -1.0f,
            .friction_coefficients[1] = 1.0f,
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 2, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 5.0f,
                            .Ki = 0.0f,
                            .Kd = 0.1f,
                            .MaxOut = 30000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 6000.0f,
                        },
                },
            .motor_type = M2006,  // 拨盘电机为M2006
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};