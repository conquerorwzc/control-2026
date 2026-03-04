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
              .wheel_reduction_ratio = 268.0f / 17.0f,                                                              \
              .joint_limit[0] = {.angle_min = -70.56f * DEGREE_2_RAD,                                               \
                                 .angle_max = 0.0f,                                                                 \
                                 .buffer_zone = 0.15f,                                                              \
                                 .kp = 200.0f,                                                                      \
                                 .kd = 3.0f,                                                                        \
                                 .max_barrier_torque = 25.0f},                                                      \
              .joint_limit[1] = {.angle_min = 0.0f,                                                                 \
                                 .angle_max = 70.56f * DEGREE_2_RAD,                                                \
                                 .buffer_zone = 0.15f,                                                              \
                                 .kp = 200.0f,                                                                      \
                                 .kd = 3.0f,                                                                        \
                                 .max_barrier_torque = 25.0f},                                                      \
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
                         {-21.896266f, 40.965040f, 24.770393f, -49.119100f, -50.386630f, 31.890380f},       // K[0][0]
                         {-28.844557f, 56.800164f, 34.406263f, -65.325939f, -69.551601f, 36.365709f},       // K[0][1]
                         {-18.823012f, -26.200735f, -44.633667f, 46.721147f, -39.319684f, 76.659497f},      // K[0][2]
                         {-1.934352f, -3.347413f, -13.553375f, 6.856065f, -13.302607f, 21.634315f},         // K[0][3]
                         {-17.024963f, 37.159867f, 75.617676f, -51.440891f, 50.717028f, -80.340628f},       // K[0][4]
                         {0.007919f, -2.159246f, 4.286713f, 0.290242f, 4.618314f, -7.034185f},              // K[0][5]
                         {-20.130587f, 46.761952f, -74.613188f, -40.946118f, -110.864545f, 172.274615f},    // K[0][6]
                         {-5.599670f, 0.017358f, 15.449682f, 0.257687f, -0.925681f, -15.889576f},           // K[0][7]
                         {100.295490f, -163.217632f, 523.964636f, 215.586351f, -56.974668f, -656.470117f},  // K[0][8]
                         {3.131970f, -5.899719f, 27.676426f, 8.714046f, -11.363803f, -27.858084f},          // K[0][9]
                         {-21.896266f, 24.770393f, 40.965040f, 31.890380f, -50.386630f, -49.119100f},       // K[1][0]
                         {-28.844557f, 34.406263f, 56.800164f, 36.365709f, -69.551601f, -65.325939f},       // K[1][1]
                         {18.823012f, 44.633667f, 26.200735f, -76.659497f, 39.319684f, -46.721147f},        // K[1][2]
                         {1.934352f, 13.553375f, 3.347413f, -21.634315f, 13.302607f, -6.856065f},           // K[1][3]
                         {-20.130587f, -74.613188f, 46.761952f, 172.274615f, -110.864545f, -40.946118f},    // K[1][4]
                         {-5.599670f, 15.449682f, 0.017358f, -15.889576f, -0.925681f, 0.257687f},           // K[1][5]
                         {-17.024963f, 75.617676f, 37.159867f, -80.340628f, 50.717028f, -51.440891f},       // K[1][6]
                         {0.007919f, 4.286713f, -2.159246f, -7.034185f, 4.618314f, 0.290242f},              // K[1][7]
                         {100.295490f, 523.964636f, -163.217632f, -656.470117f, -56.974668f, 215.586351f},  // K[1][8]
                         {3.131970f, 27.676426f, -5.899719f, -27.858084f, -11.363803f, 8.714046f},          // K[1][9]
                         {6.978228f, -32.643931f, 58.818919f, 38.610307f, 7.553081f, -82.179703f},          // K[2][0]
                         {9.863132f, -41.662769f, 70.045161f, 49.070325f, 13.558982f, -101.442793f},        // K[2][1]
                         {-15.294339f, -21.883826f, 56.715912f, 23.118907f, 17.800658f, -55.134737f},       // K[2][2]
                         {-1.887315f, -6.455549f, 9.954713f, 6.680027f, 3.399560f, -7.261266f},             // K[2][3]
                         {6.491882f, 32.222091f, 1.995027f, -26.512418f, -10.674557f, -37.403572f},         // K[2][4]
                         {0.022414f, 2.941560f, -2.357040f, -0.488140f, 0.065002f, 0.000253f},              // K[2][5]
                         {7.257884f, -48.249471f, 113.936510f, 57.377763f, 8.635212f, -114.043072f},        // K[2][6]
                         {1.825834f, 0.161770f, 5.888214f, -0.923327f, 1.409669f, -7.544588f},              // K[2][7]
                         {79.346546f, -79.366532f, -165.691081f, 72.477417f, 91.667676f, 93.070974f},       // K[2][8]
                         {5.553747f, -10.585985f, -6.813149f, 10.071571f, 9.947674f, -0.198254f},           // K[2][9]
                         {6.978228f, 58.818919f, -32.643931f, -82.179703f, 7.553081f, 38.610307f},          // K[3][0]
                         {9.863132f, 70.045161f, -41.662769f, -101.442793f, 13.558982f, 49.070325f},        // K[3][1]
                         {15.294339f, -56.715912f, 21.883826f, 55.134737f, -17.800658f, -23.118907f},       // K[3][2]
                         {1.887315f, -9.954713f, 6.455549f, 7.261266f, -3.399560f, -6.680027f},             // K[3][3]
                         {7.257884f, 113.936510f, -48.249471f, -114.043072f, 8.635212f, 57.377763f},        // K[3][4]
                         {1.825834f, 5.888214f, 0.161770f, -7.544588f, 1.409669f, -0.923327f},              // K[3][5]
                         {6.491882f, 1.995027f, 32.222091f, -37.403572f, -10.674557f, -26.512418f},         // K[3][6]
                         {0.022414f, -2.357040f, 2.941560f, 0.000253f, 0.065002f, -0.488140f},              // K[3][7]
                         {79.346546f, -165.691081f, -79.366532f, 93.070974f, 91.667676f, 72.477417f},       // K[3][8]
                         {5.553747f, -6.813149f, -10.585985f, -0.198254f, 9.947674f, 10.071571f}            // K[3][9]
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

         .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 180.0f, .Roll = 0.0f}};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 2.0f,
                            .Ki = 0.0f,
                            .Kd = 0.01f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -2000.0f,
                            .Ki = -200.0f,
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
                            .Kp = 1.0f,
                            .Ki = 0.0f,
                            .Kd = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5500.0f,
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
    .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = -90.0f, .Pitch = 0.0f, .Roll = 0.0f}};

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
            .friction_speed = 38000.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 50,
            .deadtime_onebullet = 500,
            .target_speed = 23.0f,
            .bullet_speed_adjustment = 500.0f,
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