/**
******************************************************************************
* @file    robot_config.h
* @brief   重装三摩擦轮发射机构台架配置
*          只配置云台(yaw + pitch)与发射机构(3 摩擦轮 + 1 拨盘)
*          底盘、超级电容、裁判系统一律不配置
*          参数取自 hero_rabbit 的 robot_config.h，改动时请注意同步
******************************************************************************
*/
#pragma once

#include "robot.h"

#ifndef BENCH_ROBOT_CONFIG_PARAM_WARNING
#define BENCH_ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check the parameters in bench robot_config.h, otherwise the bench may not behave as expected!"
#endif

// 云台 pitch 软限位，单位: 度（IMU 反馈值），按台架实际机械结构改
#define PITCH_MAX_ANGLE 14.0f
#define PITCH_MIN_ANGLE -35.0f

// ============================ 云台 ============================
static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 2.3f,
                            .Ki = 0.0f,
                            .Kd = 0.055f,
                            // dev 的 PID_Init_Config_s 已经没有 Kf 字段(前馈改由电机设置层处理),
                            // 旧分支这里填的 Kf = 2000 只能删掉
                            .DeadBand = 0.1f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 22.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5500.0f,
                            .Ki = -70.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 26000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 2.0f,
                            .Ki = 0.0f,
                            .Kd = 0.03f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 8000.0f,
                            .Ki = 120.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 26000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 1,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    // 注意: GyroOffset 是英雄原车的陀螺零偏标定值, 换板子或换 IMU 必须重标
    .imu_init_config = {
        .flag = 1,
        .scale = {1.0f, 1.0f, 1.0f},
        .offset_flag = 1,
        .GyroOffset = {0.00337256957f, -0.00220403657f, -0.00249944231f},
        .Yaw = 0.0f,
        .Pitch = 0.0f,
        .Roll = 0.0f,
    },
    // 重力补偿前馈系数: gimbal.c 里按 pitch_feedforward_scale * cos(pitch) 输出补偿电流.
    // 7000 是 six_wheel / sjtu 同型标准云台的起始值, 台架配重不同要重调; 填 0 = 完全没有重力补偿
    .pitch_feedforward_scale = 7000.0f,
};

// ============================ 发射机构 ============================
#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
  ((Motor_Init_Config_s){                            \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 2.0f,                    \
                      .Ki = 0.0f,                    \
                      .Kd = 0.03f,                   \
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
              .feedback_reverse_flag = direction,    \
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
            .one_bullet_delta_angle = 60.0f,               // 一发弹丸拨盘转动的角度(度)
            .reduction_ratio_loader = 100.0f,              // 3508 拨盘减速比
            .num_per_circle = 6,                           // 拨盘一圈装载量
            .loader_direction = 1,                        // 拨盘旋转方向
            .friction_num = 3,                             // 摩擦轮数量
            .friction_speed = 25000.0f,                    // 摩擦轮目标转速(速度环量纲是°/s: 25000°/s ≈ 4167rpm)
            // 摩擦轮转速限幅: shoot.c 的 LimitFrictionSpeedAbs 会把目标夹在 [min, max] 之间,
            // 两个都不填就是 0, 目标会被夹成 0 -> 摩擦轮永远不转, 台架第一脚就会踩这个坑
            .friction_speed_min = 20000.0f,                // 下限(°/s)
            .friction_speed_max = 40000.0f,                // 上限(°/s), 别超过电调能到的最高转速
            .friction_coefficients = {1.0f, -1.0f, 1.10f}, // 三个摩擦轮的速度比例系数
            .deadtime_burstfire = 500,                     // 连发间隔(ms)
            .deadtime_onebullet = 700,                     // 单发间隔(ms), 台架调快一点好试手感
            .target_speed = 11.6f,                         // 目标弹速(m/s)
            .bullet_speed_deadband = 0.3f,                 // 弹速死区
            .bullet_speed_adjustment = 300.0f,             // 弹速误差修正系数
            .one_barrel_heat_value = 100,                  // 每发42mm +100 (2027规则5.1.3, 与初速无关)
            .shooter_barrel_cooling_value = 100,           // 每秒冷却100 (2027表5-16 完成三级装配/装好发射机构后)
            .shooter_barrel_heat_limit = 300,              // 热量上限300 (2027表5-16 同上)
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 1, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 2, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[2] = FRICTION_MOTOR_CONFIG(&hcan1, 3, MOTOR_DIRECTION_NORMAL),
    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 30.0f,
                            .Ki = 0.0f,
                            .Kd = 0.008f,
                            .MaxOut = 40000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 2.5f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 7000.0f,
                            .MaxOut = 15000.0f,
                        },
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 4,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};
