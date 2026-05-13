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
#define ROBOT_MASS 23.0f
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

// roll 前馈: 底盘恒定偏置 + 云台 CoM 旋转分量.
//   roll_ff = K_0 + A * sin((α_g - offset_angle) * DEG2RAD), offset_angle 俯视 CW 为正.
// 几何: CoM 在 yaw 轴右偏后 48.1° → α_g = -180° + 48.1° = -131.9°.
// 标定 (roll_ff=0 时读 roll iout):
//   θ=0°:    K_0 + A·sin(α_g) = -3.50
//   θ=180°:  K_0 - A·sin(α_g) = -1.15
//   → K_0 = -2.325,  A = -1.175 / sin(-131.9°) ≈ 1.578
#define GIMBAL_COM_ANGLE_DEG (-131.9f)
#define ROLL_FF_BIAS (-2.325f)
#define ROLL_FF_AMP 1.578f

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
                              .Kp = 1.2,                                                                               \
                              .Ki = 0.6,                                                                               \
                              .Kd = 0,                                                                                 \
                              .IntegralLimit = 8000,                                                                   \
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
                            {-24.927217f, 74.264958f, 7.199199f, -69.914888f, -68.944609f, 55.826806f},   // K[0][0]
                            {-28.470859f, 86.157438f, 13.464976f, -79.157981f, -86.301403f, 55.947492f},  // K[0][1]
                            {-48.638734f, -174.550234f, -214.572479f, 246.611426f, -258.644071f,
                             422.740924f},                                                                  // K[0][2]
                            {-2.558756f, -12.323942f, -26.553558f, 17.276999f, -30.973180f, 45.109052f},    // K[0][3]
                            {-11.267621f, 58.916444f, 74.384609f, -63.192636f, 87.281689f, -106.905877f},   // K[0][4]
                            {-0.146336f, 0.603798f, 2.354020f, -0.189421f, 1.900684f, -3.926559f},          // K[0][5]
                            {-38.561955f, 22.533595f, -14.978198f, -16.000980f, -104.611858f, 85.061668f},  // K[0][6]
                            {-3.264673f, 0.551124f, 7.368989f, -0.885127f, -0.056715f, -7.814639f},         // K[0][7]
                            {65.450752f, -132.389970f, 848.545877f, 123.491298f, -166.636256f,
                             -981.182741f},                                                                   // K[0][8]
                            {1.785136f, -3.753902f, 35.237305f, 3.451339f, -14.983509f, -32.587011f},         // K[0][9]
                            {-24.927217f, 7.199199f, 74.264958f, 55.826806f, -68.944609f, -69.914888f},       // K[1][0]
                            {-28.470859f, 13.464976f, 86.157438f, 55.947492f, -86.301403f, -79.157981f},      // K[1][1]
                            {48.638734f, 214.572479f, 174.550234f, -422.740924f, 258.644071f, -246.611426f},  // K[1][2]
                            {2.558756f, 26.553558f, 12.323942f, -45.109052f, 30.973180f, -17.276999f},        // K[1][3]
                            {-38.561955f, -14.978198f, 22.533595f, 85.061668f, -104.611858f, -16.000980f},    // K[1][4]
                            {-3.264673f, 7.368989f, 0.551124f, -7.814639f, -0.056715f, -0.885127f},           // K[1][5]
                            {-11.267621f, 74.384609f, 58.916444f, -106.905877f, 87.281689f, -63.192636f},     // K[1][6]
                            {-0.146336f, 2.354020f, 0.603798f, -3.926559f, 1.900684f, -0.189421f},            // K[1][7]
                            {65.450752f, 848.545877f, -132.389970f, -981.182741f, -166.636256f,
                             123.491298f},                                                                   // K[1][8]
                            {1.785136f, 35.237305f, -3.753902f, -32.587011f, -14.983509f, 3.451339f},        // K[1][9]
                            {2.316752f, -10.288006f, 35.629683f, 17.928298f, -11.096060f, -43.181339f},      // K[2][0]
                            {3.071167f, -12.469043f, 36.931602f, 20.047805f, -8.183719f, -46.638908f},       // K[2][1]
                            {-47.080453f, -55.581708f, 132.054718f, 50.362413f, 113.106963f, -170.975565f},  // K[2][2]
                            {-3.124746f, -7.020090f, 10.679089f, 5.901796f, 9.892657f, -10.359205f},         // K[2][3]
                            {1.906750f, 36.910333f, -3.121184f, -17.575632f, -42.081241f, -8.144915f},       // K[2][4]
                            {-0.037918f, 1.655200f, -0.192563f, 0.024426f, -0.945953f, -0.375463f},          // K[2][5]
                            {5.229857f, -15.385000f, 74.702660f, 15.995171f, 3.305949f, -66.697214f},        // K[2][6]
                            {0.332532f, 0.189566f, 3.197128f, -0.308558f, -0.169458f, -2.512404f},           // K[2][7]
                            {72.228925f, -108.799499f, -132.809533f, 98.825529f, 121.995135f, 30.036899f},   // K[2][8]
                            {4.347463f, -8.441103f, -4.921827f, 7.630427f, 7.930078f, -1.483795f},           // K[2][9]
                            {2.316752f, 35.629683f, -10.288006f, -43.181339f, -11.096060f, 17.928298f},      // K[3][0]
                            {3.071167f, 36.931602f, -12.469043f, -46.638908f, -8.183719f, 20.047805f},       // K[3][1]
                            {47.080453f, -132.054718f, 55.581708f, 170.975565f, -113.106963f, -50.362413f},  // K[3][2]
                            {3.124746f, -10.679089f, 7.020090f, 10.359205f, -9.892657f, -5.901796f},         // K[3][3]
                            {5.229857f, 74.702660f, -15.385000f, -66.697214f, 3.305949f, 15.995171f},        // K[3][4]
                            {0.332532f, 3.197128f, 0.189566f, -2.512404f, -0.169458f, -0.308558f},           // K[3][5]
                            {1.906750f, -3.121184f, 36.910333f, -8.144915f, -42.081241f, -17.575632f},       // K[3][6]
                            {-0.037918f, -0.192563f, 1.655200f, -0.375463f, -0.945953f, 0.024426f},          // K[3][7]
                            {72.228925f, -132.809533f, -108.799499f, 30.036899f, 121.995135f, 98.825529f},   // K[3][8]
                            {4.347463f, -4.921827f, -8.441103f, -1.483795f, 7.930078f, 7.630427f}            // K[3][9]
                        },
                },
            .leg_init_config[0] = LEG_INIT_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE, &hcan2, 0x02, 0x01,
                                                  &hcan2, 0x06, 0x03, &hcan1, 0x01, 0x00),
            .leg_init_config[1] = LEG_INIT_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_NORMAL, &hcan2, 0x08, 0x04,
                                                  &hcan2, 0x0A, 0x05, &hcan1, 0x02, 0x00),
            .roll_PID_config =
                {
                    .Kp = 1000.0f,
                    .Ki = 0.0f,
                    .Kd = 90.0f,
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
                                         .can_handle = &hcan2,
                                         .tx_id = 0x210,  // 超级电容默认接收id
                                         .rx_id = 0x211,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
                                     }},
            .yaw_prostrate_PID_config =
                {
                    .Kp = 4.0f,
                    .Ki = 0.0f,
                    .Kd = 0.38f,
                    .MaxOut = 6.0f,
                    .DeadBand = 0.0f,
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
                            .Kp = 0.4f,
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
            .friction_speed = 38000.0f,                        // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},            // 摩擦轮速度比例系数
            .deadtime_burstfire = 50,
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
    .Kp = 0.4f,
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

#if defined(GIMBAL_BOARD)
static CANComm_Init_Config_s gimbal_main_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x012,
            .rx_id = 0x011,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s*)0)->main),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->main),
};

static CANComm_Init_Config_s gimbal_motion_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x214,
            .rx_id = 0x213,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s*)0)->motion),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->motion),
};

static CANComm_Init_Config_s gimbal_gamestate_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x216,
            .rx_id = 0x215,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s*)0)->gamestate),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->gamestate),
};

#endif
#if defined(CHASSIS_BOARD)
static CANComm_Init_Config_s chassis_main_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x011,
            .rx_id = 0x012,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->main),
    .send_data_len = sizeof(((Chassis_Upload_Data_s*)0)->main),
};

static CANComm_Init_Config_s chassis_motion_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x213,
            .rx_id = 0x214,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->motion),
    .send_data_len = sizeof(((Chassis_Upload_Data_s*)0)->motion),
};

static CANComm_Init_Config_s chassis_gamestate_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x215,
            .rx_id = 0x216,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s*)0)->gamestate),
    .send_data_len = sizeof(((Chassis_Upload_Data_s*)0)->gamestate),
};
#endif
