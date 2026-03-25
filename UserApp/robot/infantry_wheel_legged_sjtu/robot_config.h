/**
******************************************************************************
* @file    robot_config.h
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief Infantry wheeled-legged robot control module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/
#pragma once

#include "general_def.h"
#include "robot.h"

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,浮点数需要以.0或f结尾,无符号以u结尾 */

// 机器人底盘修改的参数,单位为m(米)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define WHEEL_RADIUS 0.077f          // 轮子半径
#define WHEEL_REDUCTION_RATIO 19.0f  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
// #define PITCH_MAX_ANGLE 26.0f        // 云台竖直向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define TRACK_WIDTH 0.495f
#define ROBOT_MASS 20.0f
#define LEG_MAX_LENGTH 0.370f  // 0.380f
#define LEG_MIN_LENGTH 0.117f  // 0.112f

#define DELTA_LEG_LENGTH (LEG_MAX_LENGTH - LEG_MIN_LENGTH)
// 跳台阶
#define TARGET_JUMP_HEIGHT 0.3f
#define TARGET_JUMP_DISTANCE 0.5f
// 反向飞坡
// #define TARGET_JUMP_HEIGHT 0.8f
// #define TARGET_JUMP_DISTANCE 1.0f
// 目标速度与腿部输出力
#define JUMP_SPEED TARGET_JUMP_DISTANCE / sqrtf(2.0f * TARGET_JUMP_HEIGHT / 9.8f)
#define JUMP_FORCE ROBOT_MASS * 9.8f / 2.0f * (1.0f + (TARGET_JUMP_HEIGHT - DELTA_LEG_LENGTH) / DELTA_LEG_LENGTH)

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5075  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 4215      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 20.0f       // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -40.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36.0f  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 90.0f  // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10             // 拨盘一圈的装载量

// delta_h = 0.380 - 0.112 = 0.268;
// target_h = 0.3
// m = 5
// g = 9.8
// Equation for Jump Loop
// F: 单腿输出力 m: 底盘质量 g: 重力加速度 target_h: 目标跳跃高度 delta_h: 腿部形变量 delta_t：收腿时间
// F = m * g / 2 * (1 + (target_h - delta_h) / delta_h);
// delta_t = sqrt(2 * (target_h - delta_h), g);

// 不用Macro会传不进去
#define JOINT_MOTOR_CONFIG(motor_reverse, feedback_reverse, can, tx, rx)                                       \
  {                                                                                                            \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .angle_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 20.0f,                                                                             \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.0f,                                                                              \
                      .MaxOut = 10.0f,                                                                         \
                      .DeadBand = 0.01f,                                                                       \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .IntegralLimit = 0.0f,                                                                   \
                  },                                                                                           \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 5.0f,                                                                              \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.05f,                                                                             \
                      .MaxOut = 15.0f,                                                                         \
                      .DeadBand = 0.01f,                                                                       \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .IntegralLimit = 0.0f,                                                                   \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .outer_loop_type = ANGLE_LOOP,                                                                   \
              .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                      \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .motor_reverse_flag = motor_reverse,                                                             \
              .feedback_reverse_flag = feedback_reverse,                                                       \
          },                                                                                                   \
      .motor_type = J8009P,                                                                                    \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = can,                                                                               \
              .tx_id = tx,                                                                                     \
              .rx_id = rx,                                                                                     \
          },                                                                                                   \
  }

#define LEG_INIT_CONFIG(joint_motor_reverse, wheel_motor_reverse, joint_can_0, joint_tx_0, joint_rx_0, joint_can_1, \
                        joint_tx_1, joint_rx_1, wheel_can, wheel_tx, wheel_rx)                                      \
  {                                                                                                                 \
      .cali_mode = LEG_PRE_CALI_MODE,                                                                               \
      .param =                                                                                                      \
          {                                                                                                         \
              .rod_length[0] = 0.170,                                                                               \
              .rod_length[1] = 0.285,                                                                               \
              .rod_length[2] = 0.285,                                                                               \
              .rod_length[3] = 0.170,                                                                               \
              .rod_length[4] = 0.160,                                                                               \
              .joint_motor_zero_offset[0] = 9.97 * DEGREE_2_RAD + PI,                                               \
              .joint_motor_zero_offset[1] = -9.97 * DEGREE_2_RAD,                                                   \
              .wheel_radius = WHEEL_RADIUS,                                                                         \
              .wheel_reduction_ratio = 268.0f / 17.0f,                                                              \
              .joint_limit[0] = {.angle_min = -70.56f * DEGREE_2_RAD,                                               \
                                 .angle_max = 0.0f,                                                                 \
                                 .buffer_zone = 0.15f,                                                              \
                                 .kp = 200.0f,                                                                      \
                                 .kd = 3.0f,                                                                        \
                                 .max_barrier_torque = 20.0f},                                                      \
              .joint_limit[1] = {.angle_min = 0.0f,                                                                 \
                                 .angle_max = 70.56f * DEGREE_2_RAD,                                                \
                                 .buffer_zone = 0.15f,                                                              \
                                 .kp = 200.0f,                                                                      \
                                 .kd = 3.0f,                                                                        \
                                 .max_barrier_torque = 10.0f},                                                      \
          },                                                                                                        \
      .joint_motor_config[0] =                                                                                      \
          JOINT_MOTOR_CONFIG(joint_motor_reverse, joint_motor_reverse, joint_can_0, joint_tx_0, joint_rx_0),        \
      .joint_motor_config[1] =                                                                                      \
          JOINT_MOTOR_CONFIG(joint_motor_reverse, joint_motor_reverse, joint_can_1, joint_tx_1, joint_rx_1),        \
      .wheel_motor_config =                                                                                         \
          {                                                                                                         \
              .controller_setting_init_config =                                                                     \
                  {                                                                                                 \
                      .motor_reverse_flag = wheel_motor_reverse,                                                    \
                      .feedback_reverse_flag = wheel_motor_reverse,                                                 \
                  },                                                                                                \
              .motor_type = M3508,                                                                                  \
              .can_init_config =                                                                                    \
                  {                                                                                                 \
                      .can_handle = wheel_can,                                                                      \
                      .tx_id = wheel_tx,                                                                            \
                      .rx_id = wheel_rx,                                                                            \
                  },                                                                                                \
          },                                                                                                        \
      .length_PID_config =                                                                                          \
          {                                                                                                         \
              .Kp = 850.0f,                                                                                         \
              .Ki = 0.0f,                                                                                           \
              .Kd = 120.0f,                                                                                         \
              .MaxOut = 90.0f,                                                                                      \
              .DeadBand = 0.01f,                                                                                    \
              .Improve = PID_IMPROVE_NONE,                                                                          \
              .IntegralLimit = 0.0f,                                                                                \
          },                                                                                                        \
  }

/* SJTU LQR: fill .lqr_param.K and .lqr_param.LQR_K_Coefficients from MATLAB compute_lqr.m output */

static Chassis_Init_Config_s
    chassis_init_config =
        {.param =
             {
                 .track_width = TRACK_WIDTH,
                 .body_mass = ROBOT_MASS,
                 .initial_leg_length = 0.20f,
                 .leg_min_length = LEG_MIN_LENGTH,
                 .leg_max_length = LEG_MAX_LENGTH,
                 .LQR_K_Coefficients =
                     {
                         {-31.315705f, 20.358533f, 56.237515f, -28.528646f, -44.685181f, 8.607669f},    // K[0][0]
                         {-19.982034f, 14.115014f, 39.708045f, -18.799188f, -29.577290f, -3.174559f},   // K[0][1]
                         {-7.164259f, -14.934957f, -42.204304f, 23.522181f, -6.156102f, 78.596205f},    // K[0][2]
                         {-1.412459f, -3.484654f, -10.558359f, 5.401918f, -0.520349f, 18.615169f},      // K[0][3]
                         {-6.993531f, 24.912423f, 51.537538f, -42.582933f, 13.738236f, -66.618961f},    // K[0][4]
                         {0.018729f, -0.403297f, 0.823923f, 0.145487f, 0.957748f, -1.291213f},          // K[0][5]
                         {-11.864189f, -2.458162f, -49.261242f, 6.624416f, -18.854122f, 107.674319f},   // K[0][6]
                         {-1.632155f, 0.173426f, 3.532310f, -0.382393f, 0.563258f, -4.116916f},         // K[0][7]
                         {7.415104f, -15.746537f, 114.598685f, 21.634847f, -27.960305f, -124.413565f},  // K[0][8]
                         {-0.293879f, -2.084732f, 17.454030f, 3.459579f, -6.642049f, -17.399208f},      // K[0][9]
                         {-31.315705f, 56.237515f, 20.358533f, 8.607669f, -44.685181f, -28.528646f},    // K[1][0]
                         {-19.982034f, 39.708045f, 14.115014f, -3.174559f, -29.577290f, -18.799188f},   // K[1][1]
                         {7.164259f, 42.204304f, 14.934957f, -78.596205f, 6.156102f, -23.522181f},      // K[1][2]
                         {1.412459f, 10.558359f, 3.484654f, -18.615169f, 0.520349f, -5.401918f},        // K[1][3]
                         {-11.864189f, -49.261242f, -2.458162f, 107.674319f, -18.854122f, 6.624416f},   // K[1][4]
                         {-1.632155f, 3.532310f, 0.173426f, -4.116916f, 0.563258f, -0.382393f},         // K[1][5]
                         {-6.993531f, 51.537538f, 24.912423f, -66.618961f, 13.738236f, -42.582933f},    // K[1][6]
                         {0.018729f, 0.823923f, -0.403297f, -1.291213f, 0.957748f, 0.145487f},          // K[1][7]
                         {7.415104f, 114.598685f, -15.746537f, -124.413565f, -27.960305f, 21.634847f},  // K[1][8]
                         {-0.293879f, 17.454030f, -2.084732f, -17.399208f, -6.642049f, 3.459579f},      // K[1][9]
                         {3.057600f, -30.538054f, 76.266291f, 40.721110f, 10.041769f, -111.077442f},    // K[2][0]
                         {2.945719f, -18.911689f, 43.582654f, 25.613170f, 8.127509f, -64.363236f},      // K[2][1]
                         {-7.720256f, -4.735808f, 37.652696f, 0.657999f, 19.293656f, -42.279857f},      // K[2][2]
                         {-1.494287f, -1.030463f, 8.547726f, -0.238601f, 4.014616f, -8.896117f},        // K[2][3]
                         {3.416341f, 9.884822f, -4.528203f, 0.869733f, -17.244534f, -5.283146f},        // K[2][4]
                         {-0.034309f, 1.112540f, -0.326737f, -0.193816f, 0.227304f, -0.251122f},        // K[2][5]
                         {1.658601f, -11.076763f, 70.040678f, 10.450033f, 12.276450f, -69.286008f},     // K[2][6]
                         {0.387624f, 0.151474f, 2.103202f, -0.301659f, 0.000566f, -1.114276f},          // K[2][7]
                         {14.247334f, -21.151860f, -14.858558f, 21.979691f, 19.428781f, -9.678476f},    // K[2][8]
                         {2.303709f, -4.718032f, -1.246005f, 5.034668f, 4.249314f, -3.320778f},         // K[2][9]
                         {3.057600f, 76.266291f, -30.538054f, -111.077442f, 10.041769f, 40.721110f},    // K[3][0]
                         {2.945719f, 43.582654f, -18.911689f, -64.363236f, 8.127509f, 25.613170f},      // K[3][1]
                         {7.720256f, -37.652696f, 4.735808f, 42.279857f, -19.293656f, -0.657999f},      // K[3][2]
                         {1.494287f, -8.547726f, 1.030463f, 8.896117f, -4.014616f, 0.238601f},          // K[3][3]
                         {1.658601f, 70.040678f, -11.076763f, -69.286008f, 12.276450f, 10.450033f},     // K[3][4]
                         {0.387624f, 2.103202f, 0.151474f, -1.114276f, 0.000566f, -0.301659f},          // K[3][5]
                         {3.416341f, -4.528203f, 9.884822f, -5.283146f, -17.244534f, 0.869733f},        // K[3][6]
                         {-0.034309f, -0.326737f, 1.112540f, -0.251122f, 0.227304f, -0.193816f},        // K[3][7]
                         {14.247334f, -14.858558f, -21.151860f, -9.678476f, 19.428781f, 21.979691f},    // K[3][8]
                         {2.303709f, -1.246005f, -4.718032f, -3.320778f, 4.249314f, 5.034668f}          // K[3][9]
                     },
             },
         .leg_init_config[0] = LEG_INIT_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE, &hcan2, 0x02, 0x01,
                                               &hcan2, 0x06, 0x03, &hcan1, 0x01, 0x00),
         .leg_init_config[1] = LEG_INIT_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_NORMAL, &hcan2, 0x08, 0x04,
                                               &hcan2, 0x0A, 0x05, &hcan1, 0x02, 0x00),
         .roll_PID_config =
             {
                 .Kp = 300.0f,
                 .Ki = 0.0f,
                 .Kd = 0.0f,
                 .MaxOut = 100.0f,
                 .DeadBand = 0.0f,
                 .Improve = PID_IMPROVE_NONE,
                 .IntegralLimit = 0.0f,
             },

         .imu_init_config = {.flag = 1,
                             .scale = {1.0f, 1.0f, 1.0f},
                             .Yaw = 0.0f,
                             .Pitch = 180.0f,
                             .Roll = 0.0f,
                             .CenterOffset[0] = 0.15413f,
                             .CenterOffset[1] = 0.04612f,
                             .CenterOffset[2] = 0.09348f}};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    // .angle_PID =
                    //     {
                    //         .Kp = 2.0f,
                    //         .Ki = 0.0f,
                    //         .Kd = 0.03f,
                    //         .DeadBand = 0.01f,
                    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    //         .IntegralLimit = 5.0f,
                    //         .MaxOut = 22.0f,
                    //     },
                    .angle_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.0f,
                            .Kd = 0.05f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 22.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5000.0f,
                            .Ki = -100.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 25000.0f,
                        },

                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 6,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.0f,
                            .Kd = 0.02f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5000.0f,
                            .Ki = -200.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 28000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = -90.0f, .Pitch = 0.0f, .Roll = 0.0f},
    .pitch_feedforward_scale = 7000.0f};

#define FRICTION_MOTOR_CONFIG(handle, id, motor_direction, feedback_direction) \
  ((Motor_Init_Config_s){                                                      \
      .controller_param_init_config =                                          \
          {                                                                    \
              .speed_PID =                                                     \
                  {                                                            \
                      .Kp = 1.5f,                                              \
                      .Ki = 0.2f,                                              \
                      .Kd = 0.0f,                                              \
                      .Improve = PID_Integral_Limit,                           \
                      .IntegralLimit = 10000.0f,                               \
                      .MaxOut = 15000.0f,                                      \
                  },                                                           \
          },                                                                   \
      .controller_setting_init_config =                                        \
          {                                                                    \
              .angle_feedback_source = MOTOR_FEED,                             \
              .speed_feedback_source = MOTOR_FEED,                             \
              .outer_loop_type = SPEED_LOOP,                                   \
              .close_loop_type = SPEED_LOOP,                                   \
              .motor_reverse_flag = motor_direction,                           \
              .feedback_reverse_flag = feedback_direction,                     \
          },                                                                   \
      .motor_type = M3508,                                                     \
      .can_init_config =                                                       \
          {                                                                    \
              .can_handle = handle,                                            \
              .tx_id = id,                                                     \
          },                                                                   \
  })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = ONE_BULLET_DELTA_ANGLE,  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
            .reduction_ratio_loader = REDUCTION_RATIO_LOADER,  // M2006拨盘电机的减速比
            .num_per_circle = NUM_PER_CIRCLE,                  // 拨盘一圈的装载量
            .loader_direction = 1,                             // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                                 // 摩擦轮数量
            .friction_speed = 42000.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 50,
            .deadtime_onebullet = 350,
            .target_speed = 25.5f,
            .bullet_speed_adjustment = 200.0f,
            .feedforward = 200.0f,
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 5, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 40.0f,
                            .Ki = 0.0f,
                            .Kd = 1.0f,
                            .MaxOut = 30000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 8000.0f,
                        },
                },
            .motor_type = M2006,  // 拨盘电机为M2006
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

static PID_Init_Config_s chassis_rotate_PID_config = {
    .Kp = 1.0f,
    .Ki = 0.0f,
    .Kd = 0.01f,
    .IntegralLimit = 1.5f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut = 3.0f,
};

static SuperCap_Init_Config_s super_cap_config = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = 0x302,  // 超级电容默认接收id
        .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    }};

#if defined(GIMBAL_BOARD)
static CANComm_Init_Config_s gimbal_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
    .recv_data_len = sizeof(Chassis_Upload_Data_s),
    .send_data_len = sizeof(Chassis_Fetch_Data_s),  // chassis_ctrl_cmd
};

#endif
#if defined(CHASSIS_BOARD)
static CANComm_Init_Config_s chassis_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
    .recv_data_len = sizeof(Chassis_Fetch_Data_s),  // chassis_ctrl_cmd
    .send_data_len = sizeof(Chassis_Upload_Data_s),
};
#endif