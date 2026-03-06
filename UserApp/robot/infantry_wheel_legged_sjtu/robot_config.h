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
      .length_PID_config =                                                                                          \
          {                                                                                                         \
              .Kp = 750.0f,                                                                                         \
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
                         {-15.057636f, 23.881547f, 12.151309f, -31.214730f, -26.607417f, 30.040838f},    // LQR K[0][0]
                         {-18.281509f, 28.851420f, 18.269185f, -37.148030f, -32.157789f, 28.652479f},    // LQR K[0][1]
                         {-5.220131f, -15.086891f, -47.968773f, 26.297328f, -26.383534f, 82.546933f},    // LQR K[0][2]
                         {-0.900019f, -3.946140f, -13.402263f, 6.844374f, -6.788222f, 22.501988f},       // LQR K[0][3]
                         {-11.729307f, 39.723769f, 62.108372f, -63.879575f, 39.289179f, -75.285436f},    // LQR K[0][4]
                         {-0.040714f, -0.530781f, 1.852401f, -0.419274f, 2.286420f, -2.786137f},         // LQR K[0][5]
                         {-12.172993f, 9.024473f, -83.090325f, -3.239080f, -54.860014f, 170.763597f},    // LQR K[0][6]
                         {-3.100418f, 0.211658f, 6.446420f, -0.363200f, 0.087645f, -5.794190f},          // LQR K[0][7]
                         {14.031230f, -28.456005f, 193.392738f, 37.089394f, -43.161152f, -208.561185f},  // LQR K[0][8]
                         {0.094200f, -1.742127f, 19.623022f, 2.734007f, -8.632428f, -17.590435f},        // LQR K[0][9]
                         {-15.057636f, 12.151309f, 23.881547f, 30.040838f, -26.607417f, -31.214730f},    // LQR K[1][0]
                         {-18.281509f, 18.269185f, 28.851420f, 28.652479f, -32.157789f, -37.148030f},    // LQR K[1][1]
                         {5.220131f, 47.968773f, 15.086891f, -82.546933f, 26.383534f, -26.297328f},      // LQR K[1][2]
                         {0.900019f, 13.402263f, 3.946140f, -22.501988f, 6.788222f, -6.844374f},         // LQR K[1][3]
                         {-12.172993f, -83.090325f, 9.024473f, 170.763597f, -54.860014f, -3.239080f},    // LQR K[1][4]
                         {-3.100418f, 6.446420f, 0.211658f, -5.794190f, 0.087645f, -0.363200f},          // LQR K[1][5]
                         {-11.729307f, 62.108372f, 39.723769f, -75.285436f, 39.289179f, -63.879575f},    // LQR K[1][6]
                         {-0.040714f, 1.852401f, -0.530781f, -2.786137f, 2.286420f, -0.419274f},         // LQR K[1][7]
                         {14.031230f, 193.392738f, -28.456005f, -208.561185f, -43.161152f, 37.089394f},  // LQR K[1][8]
                         {0.094200f, 19.623022f, -1.742127f, -17.590435f, -8.632428f, 2.734007f},        // LQR K[1][9]
                         {1.342326f, -12.220511f, 29.047643f, 15.033991f, 2.911118f, -39.762176f},       // LQR K[2][0]
                         {1.961389f, -13.964693f, 32.255942f, 17.475333f, 4.032336f, -44.662069f},       // LQR K[2][1]
                         {-6.409985f, -9.036948f, 24.733452f, 8.389763f, 14.814691f, -23.437270f},       // LQR K[2][2]
                         {-1.281715f, -2.362360f, 5.993427f, 1.916164f, 3.600695f, -5.087867f},          // LQR K[2][3]
                         {2.905583f, 14.687360f, 1.279098f, -7.674317f, -18.369072f, -13.942550f},       // LQR K[2][4]
                         {-0.026177f, 1.107799f, -0.261275f, -0.001836f, 0.084972f, -0.611928f},         // LQR K[2][5]
                         {1.654319f, -16.409962f, 63.764972f, 17.935965f, 9.434772f, -58.211418f},       // LQR K[2][6]
                         {0.431037f, 0.121090f, 2.983025f, -0.269418f, 0.033341f, -2.502989f},           // LQR K[2][7]
                         {19.218392f, -28.535189f, -19.220568f, 28.199543f, 23.475880f, -12.837266f},    // LQR K[2][8]
                         {2.443480f, -5.190914f, -1.208640f, 5.222811f, 4.261002f, -3.201153f},          // LQR K[2][9]
                         {1.342326f, 29.047643f, -12.220511f, -39.762176f, 2.911118f, 15.033991f},       // LQR K[3][0]
                         {1.961389f, 32.255942f, -13.964693f, -44.662069f, 4.032336f, 17.475333f},       // LQR K[3][1]
                         {6.409985f, -24.733452f, 9.036948f, 23.437270f, -14.814691f, -8.389763f},       // LQR K[3][2]
                         {1.281715f, -5.993427f, 2.362360f, 5.087867f, -3.600695f, -1.916164f},          // LQR K[3][3]
                         {1.654319f, 63.764972f, -16.409962f, -58.211418f, 9.434772f, 17.935965f},       // LQR K[3][4]
                         {0.431037f, 2.983025f, 0.121090f, -2.502989f, 0.033341f, -0.269418f},           // LQR K[3][5]
                         {2.905583f, 1.279098f, 14.687360f, -13.942550f, -18.369072f, -7.674317f},       // LQR K[3][6]
                         {-0.026177f, -0.261275f, 1.107799f, -0.611928f, 0.084972f, -0.001836f},         // LQR K[3][7]
                         {19.218392f, -19.220568f, -28.535189f, -12.837266f, 23.475880f, 28.199543f},    // LQR K[3][8]
                         {2.443480f, -1.208640f, -5.190914f, -3.201153f, 4.261002f, 5.222811f}           // LQR K[3][9]
                     },
                 .MPC_K_Coefficients =
                     {
                         {-0.009207f, 0.026752f, -0.001380f, -0.026445f, -0.022767f, 0.026537f},       // MPC K[0][0]
                         {-3.051998f, 10.001189f, -2.761764f, -9.591019f, -8.501896f, 13.203036f},     // MPC K[0][1]
                         {-0.020154f, -0.133207f, 0.058624f, 0.186959f, 0.041157f, -0.003390f},        // MPC K[0][2]
                         {-0.017058f, -0.142169f, 0.042015f, 0.198967f, 0.044197f, 0.025129f},         // MPC K[0][3]
                         {-5.467023f, 55.183226f, 29.667746f, -71.901123f, -26.575825f, -57.680784f},  // MPC K[0][4]
                         {-1.728462f, 5.345694f, 6.789028f, -6.645741f, -1.673018f, -8.411566f},       // MPC K[0][5]
                         {-3.216320f, -28.728647f, -3.906876f, 41.532716f, -10.397406f, 42.414622f},   // MPC K[0][6]
                         {-1.276910f, 4.444052f, -1.989136f, -4.688648f, -10.561971f, 11.832525f},     // MPC K[0][7]
                         {1.166495f, -3.153745f, 15.137691f, 0.624854f, 3.464133f, -23.869842f},       // MPC K[0][8]
                         {1.279265f, -4.227286f, 22.554083f, 0.186852f, 4.717473f, -35.161039f},       // MPC K[0][9]
                         {-0.003391f, 0.009738f, -0.000283f, -0.009652f, -0.008288f, 0.009340f},       // MPC K[0][10]
                         {-1.226046f, 3.887940f, -0.847200f, -3.757463f, -3.306021f, 4.802045f},       // MPC K[0][11]
                         {-0.007396f, -0.047895f, 0.021769f, 0.067243f, 0.014785f, -0.002182f},        // MPC K[0][12]
                         {-0.006780f, -0.053388f, 0.017506f, 0.074765f, 0.016571f, 0.007021f},         // MPC K[0][13]
                         {-0.003693f, 0.024029f, 0.037527f, -0.046166f, 0.010175f, -0.062562f},        // MPC K[0][14]
                         {-0.541042f, 1.768563f, 2.335406f, -2.348995f, -0.376062f, -3.027995f},       // MPC K[0][15]
                         {-0.031290f, 0.010247f, 0.107415f, -0.011430f, -0.052627f, -0.092719f},       // MPC K[0][16]
                         {-0.675726f, 1.442183f, 0.442725f, -1.535248f, -3.633502f, 2.584109f},        // MPC K[0][17]
                         {0.433971f, -1.152166f, 5.479999f, 0.242774f, 1.263874f, -8.650089f},         // MPC K[0][18]
                         {0.531832f, -1.658240f, 8.619519f, 0.138812f, 1.843027f, -13.474048f},        // MPC K[0][19]
                         {-0.009207f, -0.001380f, 0.026752f, 0.026537f, -0.022767f, -0.026445f},       // MPC K[1][0]
                         {-3.051998f, -2.761764f, 10.001189f, 13.203036f, -8.501896f, -9.591019f},     // MPC K[1][1]
                         {0.020154f, -0.058624f, 0.133207f, 0.003390f, -0.041157f, -0.186959f},        // MPC K[1][2]
                         {0.017058f, -0.042015f, 0.142169f, -0.025129f, -0.044197f, -0.198967f},       // MPC K[1][3]
                         {-3.216320f, -3.906876f, -28.728647f, 42.414622f, -10.397406f, 41.532716f},   // MPC K[1][4]
                         {-1.276910f, -1.989136f, 4.444052f, 11.832525f, -10.561971f, -4.688648f},     // MPC K[1][5]
                         {-5.467023f, 29.667746f, 55.183226f, -57.680784f, -26.575825f, -71.901123f},  // MPC K[1][6]
                         {-1.728462f, 6.789028f, 5.345694f, -8.411566f, -1.673018f, -6.645741f},       // MPC K[1][7]
                         {1.166495f, 15.137691f, -3.153745f, -23.869842f, 3.464133f, 0.624854f},       // MPC K[1][8]
                         {1.279265f, 22.554083f, -4.227286f, -35.161039f, 4.717473f, 0.186852f},       // MPC K[1][9]
                         {-0.003391f, -0.000283f, 0.009738f, 0.009340f, -0.008288f, -0.009652f},       // MPC K[1][10]
                         {-1.226046f, -0.847200f, 3.887940f, 4.802045f, -3.306021f, -3.757463f},       // MPC K[1][11]
                         {0.007396f, -0.021769f, 0.047895f, 0.002182f, -0.014785f, -0.067243f},        // MPC K[1][12]
                         {0.006780f, -0.017506f, 0.053388f, -0.007021f, -0.016571f, -0.074765f},       // MPC K[1][13]
                         {-0.031290f, 0.107415f, 0.010247f, -0.092719f, -0.052627f, -0.011430f},       // MPC K[1][14]
                         {-0.675726f, 0.442725f, 1.442183f, 2.584109f, -3.633502f, -1.535248f},        // MPC K[1][15]
                         {-0.003693f, 0.037527f, 0.024029f, -0.062562f, 0.010175f, -0.046166f},        // MPC K[1][16]
                         {-0.541042f, 2.335406f, 1.768563f, -3.027995f, -0.376062f, -2.348995f},       // MPC K[1][17]
                         {0.433971f, 5.479999f, -1.152166f, -8.650089f, 1.263874f, 0.242774f},         // MPC K[1][18]
                         {0.531832f, 8.619519f, -1.658240f, -13.474048f, 1.843027f, 0.138812f},        // MPC K[1][19]
                         {-0.000064f, -0.004502f, 0.006776f, 0.005582f, -0.000560f, -0.010243f},       // MPC K[2][0]
                         {-0.067677f, -1.666490f, 2.799036f, 2.022097f, -0.298401f, -4.142087f},       // MPC K[2][1]
                         {-0.017671f, 0.027037f, 0.017858f, -0.044817f, 0.012978f, -0.043668f},        // MPC K[2][2]
                         {-0.016790f, 0.028765f, 0.020480f, -0.047635f, 0.013872f, -0.048374f},        // MPC K[2][3]
                         {1.730279f, -10.826601f, -9.141495f, 16.740139f, -3.101597f, 16.157705f},     // MPC K[2][4]
                         {0.280822f, -0.905290f, -0.695377f, 1.383335f, -0.748870f, 0.514702f},        // MPC K[2][5]
                         {-0.312641f, 5.212873f, 7.147321f, -9.936787f, 9.863441f, -18.460298f},       // MPC K[2][6]
                         {0.150537f, -1.100835f, 1.320026f, 1.114340f, 2.700183f, -3.547046f},         // MPC K[2][7]
                         {0.295698f, 0.263459f, -2.129241f, 0.105088f, 0.432740f, 2.479644f},          // MPC K[2][8]
                         {0.454630f, 0.287718f, -3.154281f, 0.330760f, 0.724902f, 3.580156f},          // MPC K[2][9]
                         {-0.000019f, -0.001640f, 0.002442f, 0.002038f, -0.000196f, -0.003699f},       // MPC K[2][10]
                         {-0.021931f, -0.649466f, 1.061640f, 0.792449f, -0.107246f, -1.579012f},       // MPC K[2][11]
                         {-0.006431f, 0.009725f, 0.006373f, -0.016121f, 0.004667f, -0.015643f},        // MPC K[2][12]
                         {-0.006484f, 0.010810f, 0.007572f, -0.017905f, 0.005208f, -0.018018f},        // MPC K[2][13]
                         {0.001289f, -0.004609f, -0.008611f, 0.009377f, -0.004818f, 0.008749f},        // MPC K[2][14]
                         {0.093872f, -0.303398f, -0.279791f, 0.485457f, -0.266467f, 0.221893f},        // MPC K[2][15]
                         {0.003617f, -0.004560f, -0.000116f, 0.003832f, 0.018284f, -0.014487f},        // MPC K[2][16]
                         {0.079560f, -0.374643f, 0.383850f, 0.375300f, 0.968597f, -1.174114f},         // MPC K[2][17]
                         {0.106733f, 0.097705f, -0.771210f, 0.034166f, 0.154879f, 0.900200f},          // MPC K[2][18]
                         {0.172473f, 0.119448f, -1.207117f, 0.110631f, 0.269783f, 1.378586f},          // MPC K[2][19]
                         {-0.000064f, 0.006776f, -0.004502f, -0.010243f, -0.000560f, 0.005582f},       // MPC K[3][0]
                         {-0.067677f, 2.799036f, -1.666490f, -4.142087f, -0.298401f, 2.022097f},       // MPC K[3][1]
                         {0.017671f, -0.017858f, -0.027037f, 0.043668f, -0.012978f, 0.044817f},        // MPC K[3][2]
                         {0.016790f, -0.020480f, -0.028765f, 0.048374f, -0.013872f, 0.047635f},        // MPC K[3][3]
                         {-0.312641f, 7.147321f, 5.212873f, -18.460298f, 9.863441f, -9.936787f},       // MPC K[3][4]
                         {0.150537f, 1.320026f, -1.100835f, -3.547046f, 2.700183f, 1.114340f},         // MPC K[3][5]
                         {1.730279f, -9.141495f, -10.826601f, 16.157705f, -3.101597f, 16.740139f},     // MPC K[3][6]
                         {0.280822f, -0.695377f, -0.905290f, 0.514702f, -0.748870f, 1.383335f},        // MPC K[3][7]
                         {0.295698f, -2.129241f, 0.263459f, 2.479644f, 0.432740f, 0.105088f},          // MPC K[3][8]
                         {0.454630f, -3.154281f, 0.287718f, 3.580156f, 0.724902f, 0.330760f},          // MPC K[3][9]
                         {-0.000019f, 0.002442f, -0.001640f, -0.003699f, -0.000196f, 0.002038f},       // MPC K[3][10]
                         {-0.021931f, 1.061640f, -0.649466f, -1.579012f, -0.107246f, 0.792449f},       // MPC K[3][11]
                         {0.006431f, -0.006373f, -0.009725f, 0.015643f, -0.004667f, 0.016121f},        // MPC K[3][12]
                         {0.006484f, -0.007572f, -0.010810f, 0.018018f, -0.005208f, 0.017905f},        // MPC K[3][13]
                         {0.003617f, -0.000116f, -0.004560f, -0.014487f, 0.018284f, 0.003832f},        // MPC K[3][14]
                         {0.079560f, 0.383850f, -0.374643f, -1.174114f, 0.968597f, 0.375300f},         // MPC K[3][15]
                         {0.001289f, -0.008611f, -0.004609f, 0.008749f, -0.004818f, 0.009377f},        // MPC K[3][16]
                         {0.093872f, -0.279791f, -0.303398f, 0.221893f, -0.266467f, 0.485457f},        // MPC K[3][17]
                         {0.106733f, -0.771210f, 0.097705f, 0.900200f, 0.154879f, 0.034166f},          // MPC K[3][18]
                         {0.172473f, -1.207117f, 0.119448f, 1.378586f, 0.269783f, 0.110631f}           // MPC K[3][19]
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

         .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 180.0f, .Roll = 0.0f}};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.0f,
                            .Kd = 0.01f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 22.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -4000.0f,
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
                            .Kp = 50.0f,
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