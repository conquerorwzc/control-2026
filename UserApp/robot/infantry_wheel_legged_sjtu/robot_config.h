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
#define ROBOT_MASS 22.0f
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
#define JUMP_FORCE 55 * ROBOT_MASS * 9.8f / 2.0f * (1.0f + (TARGET_JUMP_HEIGHT - DELTA_LEG_LENGTH) / DELTA_LEG_LENGTH)

// mini PC 偏置 roll 设点修剪 (rad). 云台朝前时全量生效, cos(offset_angle) 调制后随云台旋转衰减/翻号.
// 符号: 负值 = 把目标 roll 拉向左, 用以抵消 mini PC 在云台右侧造成的右倾. 实测调.
#define ROLL_FF_TRIM -0.01f

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5075  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 4215      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 20.0f       // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
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

#define LEG_INIT_CONFIG(joint_motor_reverse, wheel_motor_reverse, joint_can_0, joint_tx_0, joint_rx_0, joint_can_1,    \
                        joint_tx_1, joint_rx_1, wheel_can, wheel_tx, wheel_rx)                                         \
  {                                                                                                                    \
      .cali_mode = LEG_PRE_CALI_MODE,                                                                                  \
      .param =                                                                                                         \
          {                                                                                                            \
              .rod_length[0] = 0.170,                                                                                  \
              .rod_length[1] = 0.285,                                                                                  \
              .rod_length[2] = 0.285,                                                                                  \
              .rod_length[3] = 0.170,                                                                                  \
              .rod_length[4] = 0.160,                                                                                  \
              .joint_motor_zero_offset[0] = 9.97 * DEGREE_2_RAD + PI,                                                  \
              .joint_motor_zero_offset[1] = -9.97 * DEGREE_2_RAD,                                                      \
              .wheel_radius = WHEEL_RADIUS,                                                                            \
              .wheel_reduction_ratio = WHEEL_REDUCTION_RATIO,                                                          \
              .joint_limit[0] = {.angle_min = -70.56f * DEGREE_2_RAD,                                                  \
                                 .angle_max = 0.0f,                                                                    \
                                 .buffer_zone = 0.15f,                                                                 \
                                 .kp = 200.0f,                                                                         \
                                 .kd = 3.0f,                                                                           \
                                 .max_barrier_torque = 20.0f},                                                         \
              .joint_limit[1] = {.angle_min = 0.0f,                                                                    \
                                 .angle_max = 70.56f * DEGREE_2_RAD,                                                   \
                                 .buffer_zone = 0.15f,                                                                 \
                                 .kp = 200.0f,                                                                         \
                                 .kd = 3.0f,                                                                           \
                                 .max_barrier_torque = 10.0f},                                                         \
          },                                                                                                           \
      .joint_motor_config[0] =                                                                                         \
          JOINT_MOTOR_CONFIG(joint_motor_reverse, joint_motor_reverse, joint_can_0, joint_tx_0, joint_rx_0),           \
      .joint_motor_config[1] =                                                                                         \
          JOINT_MOTOR_CONFIG(joint_motor_reverse, joint_motor_reverse, joint_can_1, joint_tx_1, joint_rx_1),           \
      .wheel_motor_config =                                                                                            \
          {                                                                                                            \
              .controller_param_init_config =                                                                          \
                  {                                                                                                    \
                      .speed_PID =                                                                                     \
                          {                                                                                            \
                              .Kp = 1.0,                                                                               \
                              .Ki = 0.5,                                                                               \
                              .Kd = 0,                                                                                 \
                              .IntegralLimit = 6000,                                                                   \
                              .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                              .MaxOut = 15000,                                                                         \
                          },                                                                                           \
                  },                                                                                                   \
                                                                                                                       \
              .controller_setting_init_config =                                                                        \
                  {                                                                                                    \
                      .angle_feedback_source = MOTOR_FEED,                                                             \
                      .speed_feedback_source = MOTOR_FEED,                                                             \
                      .motor_reverse_flag = wheel_motor_reverse,                                                       \
                      .feedback_reverse_flag = wheel_motor_reverse,                                                    \
                  },                                                                                                   \
              .motor_type = M3508,                                                                                     \
              .can_init_config =                                                                                       \
                  {                                                                                                    \
                      .can_handle = wheel_can,                                                                         \
                      .tx_id = wheel_tx,                                                                               \
                      .rx_id = wheel_rx,                                                                               \
                  },                                                                                                   \
          },                                                                                                           \
      .length_PID_config =                                                                                             \
          {                                                                                                            \
              .Kp = 850.0f,                                                                                            \
              .Ki = 0.0f,                                                                                              \
              .Kd = 120.0f,                                                                                            \
              .MaxOut = 90.0f,                                                                                         \
              .DeadBand = 0.01f,                                                                                       \
              .Improve = PID_IMPROVE_NONE,                                                                             \
              .IntegralLimit = 0.0f,                                                                                   \
          },                                                                                                           \
  }

/* SJTU LQR: fill .lqr_param.K and .lqr_param.LQR_K_Coefficients from MATLAB compute_lqr.m output */

static Chassis_Init_Config_s
    chassis_init_config =
        {
            .param =
                {
                    .track_width = TRACK_WIDTH,
                    .body_mass = ROBOT_MASS,
                    .initial_leg_length = 0.20f,
                    .leg_min_length = LEG_MIN_LENGTH,
                    .leg_max_length = LEG_MAX_LENGTH,
                    .LQR_K_Coefficients =
                        {
                            {-18.890402f, 47.097889f, 8.874256f, -46.007441f, -50.751155f, 43.617375f},      // K[0][0]
                            {-23.620946f, 61.179482f, 14.647499f, -57.500641f, -70.631345f, 48.827500f},     // K[0][1]
                            {-13.467502f, -58.165371f, -111.913570f, 72.821921f, -50.468362f, 210.683770f},  // K[0][2]
                            {-1.315043f, -7.400233f, -20.436091f, 8.754375f, -8.307123f, 35.063870f},        // K[0][3]
                            {-7.311006f, 43.415144f, 52.516638f, -49.383242f, 42.545555f, -79.929450f},      // K[0][4]
                            {-0.053270f, -0.085246f, 0.120657f, 0.867070f, 0.834000f, 0.055667f},            // K[0][5]
                            {-37.301092f, 11.640885f, 1.192818f, -14.031708f, -31.612002f, 54.688872f},      // K[0][6]
                            {-0.992252f, 0.793421f, -0.030981f, -1.308529f, 0.962340f, -0.327899f},          // K[0][7]
                            {11.068133f, -22.122445f, 412.104918f, 6.050186f, -92.954682f, -462.499964f},    // K[0][8]
                            {-0.656004f, 0.710587f, 28.317716f, -0.783001f, -13.393785f, -25.590488f},       // K[0][9]
                            {-18.890402f, 8.874256f, 47.097889f, 43.617375f, -50.751155f, -46.007441f},      // K[1][0]
                            {-23.620946f, 14.647499f, 61.179482f, 48.827500f, -70.631345f, -57.500641f},     // K[1][1]
                            {13.467502f, 111.913570f, 58.165371f, -210.683770f, 50.468362f, -72.821921f},    // K[1][2]
                            {1.315043f, 20.436091f, 7.400233f, -35.063870f, 8.307123f, -8.754375f},          // K[1][3]
                            {-37.301092f, 1.192818f, 11.640885f, 54.688872f, -31.612002f, -14.031708f},      // K[1][4]
                            {-0.992252f, -0.030981f, 0.793421f, -0.327899f, 0.962340f, -1.308529f},          // K[1][5]
                            {-7.311006f, 52.516638f, 43.415144f, -79.929450f, 42.545555f, -49.383242f},      // K[1][6]
                            {-0.053270f, 0.120657f, -0.085246f, 0.055667f, 0.834000f, 0.867070f},            // K[1][7]
                            {11.068133f, 412.104918f, -22.122445f, -462.499964f, -92.954682f, 6.050186f},    // K[1][8]
                            {-0.656004f, 28.317716f, 0.710587f, -25.590488f, -13.393785f, -0.783001f},       // K[1][9]
                            {0.596756f, -9.422288f, 32.218534f, 13.213131f, -5.165876f, -40.192170f},        // K[2][0]
                            {1.105856f, -12.024309f, 37.311116f, 16.078542f, -3.856195f, -47.543313f},       // K[2][1]
                            {-15.431368f, -13.748293f, 57.805690f, 7.061602f, 46.520482f, -69.235029f},      // K[2][2]
                            {-1.942091f, -3.025670f, 8.268476f, 2.003763f, 6.436204f, -8.097671f},           // K[2][3]
                            {0.697155f, 26.435241f, -0.667403f, -12.854124f, -31.448020f, -7.083043f},       // K[2][4]
                            {-0.092305f, 1.380413f, 0.493542f, -0.508414f, -0.152613f, -0.958780f},          // K[2][5]
                            {3.267770f, -5.057958f, 73.450810f, 4.593950f, -1.992055f, -66.397777f},         // K[2][6]
                            {0.030567f, 0.112315f, 2.307896f, -0.063494f, -0.750395f, -0.433047f},           // K[2][7]
                            {33.579442f, -52.329771f, -50.051024f, 55.911054f, 48.310114f, -4.710868f},      // K[2][8]
                            {2.976192f, -6.225268f, -1.823642f, 6.449602f, 4.877667f, -3.758775f},           // K[2][9]
                            {0.596756f, 32.218534f, -9.422288f, -40.192170f, -5.165876f, 13.213131f},        // K[3][0]
                            {1.105856f, 37.311116f, -12.024309f, -47.543313f, -3.856195f, 16.078542f},       // K[3][1]
                            {15.431368f, -57.805690f, 13.748293f, 69.235029f, -46.520482f, -7.061602f},      // K[3][2]
                            {1.942091f, -8.268476f, 3.025670f, 8.097671f, -6.436204f, -2.003763f},           // K[3][3]
                            {3.267770f, 73.450810f, -5.057958f, -66.397777f, -1.992055f, 4.593950f},         // K[3][4]
                            {0.030567f, 2.307896f, 0.112315f, -0.433047f, -0.750395f, -0.063494f},           // K[3][5]
                            {0.697155f, -0.667403f, 26.435241f, -7.083043f, -31.448020f, -12.854124f},       // K[3][6]
                            {-0.092305f, 0.493542f, 1.380413f, -0.958780f, -0.152613f, -0.508414f},          // K[3][7]
                            {33.579442f, -50.051024f, -52.329771f, -4.710868f, 48.310114f, 55.911054f},      // K[3][8]
                            {2.976192f, -1.823642f, -6.225268f, -3.758775f, 4.877667f, 6.449602f}            // K[3][9]
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
            .super_cap_config = {.can_config =
                                     {
                                         .can_handle = &hcan1,
                                         .tx_id = 0x210,  // 超级电容默认接收id
                                         .rx_id = 0x211,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
                                     }},
            .yaw_prostrate_PID_config =
                {
                    .Kp = 3.0f,
                    .Ki = 0.0f,
                    .Kd = 0.02f,
                    .MaxOut = 6.0f,
                    .DeadBand = 0.01f,
                    .IntegralLimit = 3.0f,
                },
};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
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

// 云台 yaw 角度环参数:手瞄(默认)/自瞄两套,运行时按 gimbal_mode 切换
static PID_Init_Config_s yaw_angle_PID_manual_config = {
    .Kp = 0.8f,
    .Ki = 0.0f,
    .Kd = 0.02f,
    .DeadBand = 0.01f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 5.0f,
    .MaxOut = 22.0f,
};

static PID_Init_Config_s yaw_angle_PID_vision_config = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.03f,
    .DeadBand = 0.01f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 5.0f,
    .MaxOut = 22.0f,
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