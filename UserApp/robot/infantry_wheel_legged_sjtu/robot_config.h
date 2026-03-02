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
#define ROBOT_MASS 22.0f
#define LEG_MAX_LENGTH 0.370f  // 0.380f
#define LEG_MIN_LENGTH 0.120f  // 0.112f

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
#define PITCH_MIN_ANGLE -20.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
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
                      .Kp = 15.0f,                                                                             \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.0f,                                                                              \
                      .MaxOut = 5.0f,                                                                          \
                      .DeadBand = 0.01f,                                                                       \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .IntegralLimit = 0.0f,                                                                   \
                  },                                                                                           \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 5.0f,                                                                              \
                      .Ki = 0.0f,                                                                              \
                      .Kd = 0.05f,                                                                             \
                      .MaxOut = 5.0f,                                                                          \
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
              .wheel_reduction_ratio =                                                                              \
                  268.0f / 17.0f, /* Per-leg LQR/MPC unused in SJTU model (chassis-level 4x10 LQR overwrites) */    \
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
                         {-11.501063f, 10.689876f, 23.456484f, -17.531089f, -17.760805f, 0.557917f},     // K[0][0]
                         {-15.246163f, 14.781859f, 32.832638f, -23.060010f, -23.702189f, -4.087678f},    // K[0][1]
                         {-8.329333f, -18.192712f, -36.088710f, 30.541469f, -14.576875f, 65.225629f},    // K[0][2]
                         {-1.272251f, -4.759513f, -10.659697f, 7.927348f, -4.061357f, 18.327031f},       // K[0][3]
                         {-9.276323f, 38.141641f, 60.799358f, -63.092067f, 29.094156f, -81.161916f},     // K[0][4]
                         {-0.006849f, -0.881146f, 2.068199f, 0.258373f, 1.976334f, -3.399044f},          // K[0][5]
                         {-10.621495f, -2.972415f, -53.305214f, 10.895934f, -40.662995f, 117.546504f},   // K[0][6]
                         {-2.789264f, 0.123542f, 7.469926f, -0.113621f, -0.057467f, -8.103879f},         // K[0][7]
                         {29.157768f, -46.021825f, 172.107557f, 59.731545f, -24.036274f, -209.394761f},  // K[0][8]
                         {1.769940f, -4.897525f, 19.270718f, 6.377070f, -5.502560f, -20.749435f},        // K[0][9]
                         {-11.501063f, 23.456484f, 10.689876f, 0.557917f, -17.760805f, -17.531089f},     // K[1][0]
                         {-15.246163f, 32.832638f, 14.781859f, -4.087678f, -23.702189f, -23.060010f},    // K[1][1]
                         {8.329333f, 36.088710f, 18.192712f, -65.225629f, 14.576875f, -30.541469f},      // K[1][2]
                         {1.272251f, 10.659697f, 4.759513f, -18.327031f, 4.061357f, -7.927348f},         // K[1][3]
                         {-10.621495f, -53.305214f, -2.972415f, 117.546504f, -40.662995f, 10.895934f},   // K[1][4]
                         {-2.789264f, 7.469926f, 0.123542f, -8.103879f, -0.057467f, -0.113621f},         // K[1][5]
                         {-9.276323f, 60.799358f, 38.141641f, -81.161916f, 29.094156f, -63.092067f},     // K[1][6]
                         {-0.006849f, 2.068199f, -0.881146f, -3.399044f, 1.976334f, 0.258373f},          // K[1][7]
                         {29.157768f, 172.107557f, -46.021825f, -209.394761f, -24.036274f, 59.731545f},  // K[1][8]
                         {1.769940f, 19.270718f, -4.897525f, -20.749435f, -5.502560f, 6.377070f},        // K[1][9]
                         {3.193450f, -15.677203f, 30.141139f, 20.420222f, 5.272624f, -45.510602f},       // K[2][0]
                         {4.633729f, -19.684202f, 35.977508f, 25.803867f, 7.856468f, -55.532579f},       // K[2][1]
                         {-8.602141f, -8.380725f, 37.172886f, 4.256748f, 25.152838f, -40.539358f},       // K[2][2]
                         {-1.566520f, -2.279108f, 8.524299f, 0.857142f, 6.130165f, -8.560636f},          // K[2][3]
                         {4.913700f, 14.402155f, -7.608966f, -0.557699f, -30.098708f, -4.643944f},       // K[2][4]
                         {-0.004208f, 1.724385f, -1.123838f, -0.373075f, -0.003609f, 0.038644f},         // K[2][5]
                         {3.107134f, -18.966867f, 77.699459f, 17.811769f, 22.093955f, -81.660250f},      // K[2][6]
                         {0.899606f, 0.124207f, 3.143905f, -0.492742f, 0.531412f, -3.246629f},           // K[2][7]
                         {26.201746f, -28.886584f, -50.918632f, 27.688392f, 32.352103f, 23.095065f},     // K[2][8]
                         {3.286822f, -5.738805f, -4.214499f, 5.726177f, 5.894745f, -0.614695f},          // K[2][9]
                         {3.193450f, 30.141139f, -15.677203f, -45.510602f, 5.272624f, 20.420222f},       // K[3][0]
                         {4.633729f, 35.977508f, -19.684202f, -55.532579f, 7.856468f, 25.803867f},       // K[3][1]
                         {8.602141f, -37.172886f, 8.380725f, 40.539358f, -25.152838f, -4.256748f},       // K[3][2]
                         {1.566520f, -8.524299f, 2.279108f, 8.560636f, -6.130165f, -0.857142f},          // K[3][3]
                         {3.107134f, 77.699459f, -18.966867f, -81.660250f, 22.093955f, 17.811769f},      // K[3][4]
                         {0.899606f, 3.143905f, 0.124207f, -3.246629f, 0.531412f, -0.492742f},           // K[3][5]
                         {4.913700f, -7.608966f, 14.402155f, -4.643944f, -30.098708f, -0.557699f},       // K[3][6]
                         {-0.004208f, -1.123838f, 1.724385f, 0.038644f, -0.003609f, -0.373075f},         // K[3][7]
                         {26.201746f, -50.918632f, -28.886584f, 23.095065f, 32.352103f, 27.688392f},     // K[3][8]
                         {3.286822f, -4.214499f, -5.738805f, -0.614695f, 5.894745f, 5.726177f}           // K[3][9]
                     },
             },
         .leg_init_config[0] = LEG_INIT_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE, &hcan2, 0x02, 0x01,
                                               &hcan2, 0x06, 0x03, &hcan1, 0x01, 0x00),
         .leg_init_config[1] = LEG_INIT_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_NORMAL, &hcan2, 0x08, 0x04,
                                               &hcan2, 0x0A, 0x05, &hcan1, 0x02, 0x00),
         .delta_theta_PID_config =
             {
                 .Kp = 10.0f,
                 .Ki = 0.0f,
                 .Kd = 0.1f,
                 .MaxOut = 2.0f,
                 .DeadBand = 0.01f,
                 .Improve = PID_IMPROVE_NONE,
                 .IntegralLimit = 0.0f,
             },
         .roll_PID_config =
             {
                 .Kp = 100.0f,
                 .Ki = 0.0f,
                 .Kd = 0.0f,
                 .MaxOut = 100.0f,
                 .DeadBand = 0.0f,
                 .Improve = PID_IMPROVE_NONE,
                 .IntegralLimit = 0.0f,
             },
         .length_PID_config =
             {
                 .Kp = 750.0f,
                 .Ki = 0.0f,
                 .Kd = 120.0f,
                 .MaxOut = 90.0f,
                 .DeadBand = 0.01f,
                 .Improve = PID_IMPROVE_NONE,
                 .IntegralLimit = 0.0f,
             },
         .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 180.0f, .Roll = 0.0f}};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 2.5f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -3000.0f,
                            .Ki = -300.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 20000.0f,
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
                            .Kp = 2.2f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -3500.0f,
                            .Ki = -300.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 20000.0f,
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
    .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = -90.0f, .Pitch = 0.0f, .Roll = 0.0f}};

#define FRICTION_MOTOR_CONFIG(handle, id, motor_direction, feedback_direction) \
  ((Motor_Init_Config_s){                                                      \
      .controller_param_init_config =                                          \
          {                                                                    \
              .speed_PID =                                                     \
                  {                                                            \
                      .Kp = 1.5f,                                              \
                      .Ki = 0.3f,                                              \
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
            .friction_speed = 37000.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 500,
            .deadtime_onebullet = 500,
            .target_speed = 0.0f,
            .bullet_speed_adjustment = 10.0f,
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