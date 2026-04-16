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
#define CENTER_GIMBAL_OFFSET_X 0              // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0              // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define WHEEL_RADIUS 0.077f                   // 轮子半径
#define WHEEL_REDUCTION_RATIO 268.0f / 17.0f  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
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
              .wheel_reduction_ratio = WHEEL_REDUCTION_RATIO,                                                       \
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
                         {-55.912808f, 134.036516f, -53.200170f, -143.340223f, -126.605147f, 265.620964f},  // K[0][0]
                         {-36.644738f, 89.837521f, -23.458681f, -95.029858f, -88.343984f, 153.316240f},     // K[0][1]
                         {-5.736060f, -3.227618f, -81.334072f, 11.575864f, -55.297156f, 129.401497f},       // K[0][2]
                         {-0.805658f, -0.131531f, -20.237312f, 1.702992f, -12.477559f, 31.205634f},         // K[0][3]
                         {-14.464244f, 28.559058f, 50.722362f, -40.187911f, 48.050729f, -34.801869f},       // K[0][4]
                         {0.008160f, -0.389991f, 1.028935f, -0.696575f, 2.658624f, -1.359884f},             // K[0][5]
                         {-20.013172f, 46.736690f, -127.622280f, -48.031263f, -78.004189f, 243.858668f},    // K[0][6]
                         {-3.165467f, 0.179343f, 4.332732f, -0.554883f, 0.746820f, -3.052408f},             // K[0][7]
                         {-4.394869f, 16.806249f, 206.361664f, -13.315594f, -87.217333f, -164.267084f},     // K[0][8]
                         {-2.933983f, 8.029718f, 21.033642f, -7.174715f, -18.980142f, -9.797655f},          // K[0][9]
                         {-55.912808f, -53.200170f, 134.036516f, 265.620964f, -126.605147f, -143.340223f},  // K[1][0]
                         {-36.644738f, -23.458681f, 89.837521f, 153.316240f, -88.343984f, -95.029858f},     // K[1][1]
                         {5.736060f, 81.334072f, 3.227618f, -129.401497f, 55.297156f, -11.575864f},         // K[1][2]
                         {0.805658f, 20.237312f, 0.131531f, -31.205634f, 12.477559f, -1.702992f},           // K[1][3]
                         {-20.013172f, -127.622280f, 46.736690f, 243.858668f, -78.004189f, -48.031263f},    // K[1][4]
                         {-3.165467f, 4.332732f, 0.179343f, -3.052408f, 0.746820f, -0.554883f},             // K[1][5]
                         {-14.464244f, 50.722362f, 28.559058f, -34.801869f, 48.050729f, -40.187911f},       // K[1][6]
                         {0.008160f, 1.028935f, -0.389991f, -1.359884f, 2.658624f, -0.696575f},             // K[1][7]
                         {-4.394869f, 206.361664f, 16.806249f, -164.267084f, -87.217333f, -13.315594f},     // K[1][8]
                         {-2.933983f, 21.033642f, 8.029718f, -9.797655f, -18.980142f, -7.174715f},          // K[1][9]
                         {-1.355961f, -25.425802f, 84.288936f, 27.981916f, 1.189175f, -98.928029f},         // K[2][0]
                         {0.147052f, -16.171387f, 49.773332f, 18.530293f, 2.565141f, -59.146091f},          // K[2][1]
                         {-7.023830f, -13.771109f, 23.072198f, 16.679095f, 5.572243f, -13.996983f},         // K[2][2]
                         {-1.241276f, -3.263720f, 4.941804f, 3.756339f, 0.902230f, -2.185312f},             // K[2][3]
                         {1.076671f, 18.277271f, 9.841190f, -16.648267f, -2.521306f, -25.909060f},          // K[2][4]
                         {-0.051227f, 0.899691f, 0.138742f, 0.111122f, 0.248259f, -0.762879f},              // K[2][5]
                         {1.301943f, -15.998243f, 57.628049f, 20.441103f, -5.241603f, -38.258201f},         // K[2][6]
                         {0.183168f, 0.182092f, 2.475315f, -0.240507f, -0.226585f, -1.257932f},             // K[2][7]
                         {14.821483f, -28.465365f, 5.156137f, 27.490077f, 15.698241f, -38.731020f},         // K[2][8]
                         {2.072163f, -5.385972f, 1.426127f, 5.342236f, 3.260091f, -5.851362f},              // K[2][9]
                         {-1.355961f, 84.288936f, -25.425802f, -98.928029f, 1.189175f, 27.981916f},         // K[3][0]
                         {0.147052f, 49.773332f, -16.171387f, -59.146091f, 2.565141f, 18.530293f},          // K[3][1]
                         {7.023830f, -23.072198f, 13.771109f, 13.996983f, -5.572243f, -16.679095f},         // K[3][2]
                         {1.241276f, -4.941804f, 3.263720f, 2.185312f, -0.902230f, -3.756339f},             // K[3][3]
                         {1.301943f, 57.628049f, -15.998243f, -38.258201f, -5.241603f, 20.441103f},         // K[3][4]
                         {0.183168f, 2.475315f, 0.182092f, -1.257932f, -0.226585f, -0.240507f},             // K[3][5]
                         {1.076671f, 9.841190f, 18.277271f, -25.909060f, -2.521306f, -16.648267f},          // K[3][6]
                         {-0.051227f, 0.138742f, 0.899691f, -0.762879f, 0.248259f, 0.111122f},              // K[3][7]
                         {14.821483f, 5.156137f, -28.465365f, -38.731020f, 15.698241f, 27.490077f},         // K[3][8]
                         {2.072163f, 1.426127f, -5.385972f, -5.851362f, 3.260091f, 5.342236f}               // K[3][9]
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

         .imu_init_config =
             {
                 .flag = 1,
                 .scale = {1.0f, 1.0f, 1.0f},
                 .Yaw = 0.0f,
                 .Pitch = 180.0f,
                 .Roll = 0.0f,
                 .CenterOffset[0] = 0.15413f,
                 .CenterOffset[1] = 0.04612f,
                 .CenterOffset[2] = 0.09348f,
                 .GyroOffset[0] = 0.00708952406,
                 .GyroOffset[1] = 0.00323308632,
                 .GyroOffset[2] = 0.00078589347,
                 .offset_flag = 1,
             },
         .super_cap_config = {.can_config = {
                                  .can_handle = &hcan1,
                                  .tx_id = 0x210,  // 超级电容默认接收id
                                  .rx_id = 0x211,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
                              }}};

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
                            .Kp = 0.8f,
                            .Ki = 0.0f,
                            .Kd = 0.02f,
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
    .imu_init_config =
        {
            .flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = -90.0f,
            .Pitch = 0.0f,
            .Roll = 0.0f,
            .GyroOffset[0] = -0.0014910656f,
            .GyroOffset[1] = -0.00283604953f,
            .GyroOffset[2] = 0.00104337547f,
            .offset_flag = 1,
        },
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
            .friction_speed = 40000.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 100,
            .deadtime_onebullet = 350,
            .target_speed = 22.5f,
            .bullet_speed_adjustment = 200.0f,
            .feedforward = 200.0f,
            .one_barrel_heat_value = 10,         // 一发弹丸所需热量
            .shooter_barrel_cooling_value = 40,  // 每秒冷却回复
            .shooter_barrel_heat_limit = 230,    // 热量上限
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 5, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 60.0f,
                            .Ki = 0.0f,
                            .Kd = 1.0f,
                            .MaxOut = 45000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 10000.0f,
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

static PID_Init_Config_s chassis_vx_PID_config = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .IntegralLimit = 0.5f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut = 1.0f,
};

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