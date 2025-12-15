/**
******************************************************************************
* @file    robot.h
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

#include "robot.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD  // 单板控制整车

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

// 机器人底盘修改的参数,单位为mm(毫米)
#define TRACK_WIDTH 300              // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define WHEEL_RADIUS 60              // 轮子半径
#define WHEEL_REDUCTION_RATIO 19.0f  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define PITCH_MAX_ANGLE 26.0f        // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define LEG_MAX_LENGTH 0.37f  // 0.380f
#define LEG_MIN_LENGTH 0.13f  // 0.112f

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 4757
#define PITCH_HORIZON_ECD 3494  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 26.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -35.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 发射参数
#define ONE_BULLET_DELTA_ANGLE (36.0f * 45.0f)  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f            // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10                       // 拨盘一圈的装载量

// todo: dmmotor只对j4310做了适配
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
      /* 腿部五连杆长度, 单位是M */                                                                                 \
      .leg_param =                                                                                                  \
          {                                                                                                         \
              .rod_length[0] = 0.170,                                                                               \
              .rod_length[1] = 0.285,                                                                               \
              .rod_length[2] = 0.285,                                                                               \
              .rod_length[3] = 0.170,                                                                               \
              .rod_length[4] = 0.160,                                                                               \
              .joint_motor_zero_offset[0] = 9.97 * DEGREE_2_RAD + PI,                                               \
              .joint_motor_zero_offset[1] = -9.97 * DEGREE_2_RAD,                                                   \
              .wheel_radius = 0.077f,                                                                               \
              .wheel_reduction_ratio = 268.0f / 17.0f,                                                              \
          },                                                                                                        \
      .LQR_K_Coefficient = {{{441.551183998361f, -317.100694010089f, -53.2860355048048f, 1.0214711522511f},         \
                             {93.5746418306052f, -88.7046183868486f, -2.84761011769104f, 0.230147123100247f},       \
                             {112.446285029707f, -57.2575125663828f, -24.8577632209981f, 1.95943652888061f},        \
                             {113.244561314235f, -62.595210948033f, -23.4098939988008f, 1.28451877386627f},         \
                             {-137.916111560284f, 225.955829263684f, -159.077588007052f, 56.2968935152272f},        \
                             {-19.5404161915262f, 21.9672214664995f, -11.2651499943364f, 3.88964852977064f}},       \
                            {{5.23228400555349f, -165.663228396468f, 127.410002451317f, -6.44184925503793f},        \
                             {-19.220141905894f, -11.5718592995644f, 20.9346174463876f, -0.980748386909695f},       \
                             {51.7053107398254f, -92.8306075109487f, 44.961091154552f, -0.902359796170954f},        \
                             {60.3366240178356f, -100.741571780561f, 48.3833222646449f, -1.54959058953633f},        \
                             {619.991461394924f, -698.160707752117f, 298.821820969453f, 61.4544063768443f},         \
                             {17.0573479696088f, -20.6180593973892f, 10.2480958411851f, 4.62769177087231f}}},       \
      .length_PID_config =                                                                                          \
          {                                                                                                         \
              .Kp = 350.0f,                                                                                         \
              .Ki = 0.0f,                                                                                           \
              .Kd = 3000.0f,                                                                                        \
              .MaxOut = 90.0f,                                                                                      \
              .DeadBand = 0.01f,                                                                                    \
              .Improve = PID_IMPROVE_NONE,                                                                          \
              .IntegralLimit = 0.0f,                                                                                \
          },                                                                                                        \
      .length_PID_config =                                                                                          \
          {                                                                                                         \
              .Kp = 350.0f,                                                                                         \
              .Ki = 0.0f,                                                                                           \
              .Kd = 30.0f,                                                                                          \
              .MaxOut = 90.0f,                                                                                      \
              .DeadBand = 0.01f,                                                                                    \
              .Improve = PID_IMPROVE_NONE,                                                                          \
              .IntegralLimit = 0.0f,                                                                                \
          },                                                                                                        \
      .length_d_PID_config =                                                                                        \
          {                                                                                                         \
              .Kp = 10.0f,                                                                                          \
              .Ki = 0.0f,                                                                                           \
              .Kd = 0.0f,                                                                                           \
              .MaxOut = 90.0f,                                                                                      \
              .DeadBand = 0.01f,                                                                                    \
              .Improve = PID_IMPROVE_NONE,                                                                          \
              .IntegralLimit = 0.0f,                                                                                \
          },                                                                                                        \
      .phi_PID_config =                                                                                             \
          {                                                                                                         \
              .Kp = 1.0f,                                                                                           \
              .Ki = 0.1f,                                                                                           \
              .Kd = 0.0f,                                                                                           \
              .MaxOut = 0.4f,                                                                                       \
              .DeadBand = 0.005f,                                                                                   \
              .Improve = PID_Integral_Limit,                                                                        \
              .IntegralLimit = 0.2f,                                                                                \
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
      .leg_cali_mode = LEG_PRE_CALI_MODE,                                                                           \
  }

static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            // .robot_weight = 0.0f,  // 机器人重量,单位为kg(千克)
            .robot_weight = 14.0f,  // 机器人重量,单位为kg(千克)
            .track_width = 0.245f,
            .leg_length_initial = 0.25f,  // 初始腿长,单位为m(米)
        },

    // 通过设置电机输出/反馈方向，来使腿部控制镜像对称
    .leg_init_config[0] = LEG_INIT_CONFIG(MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE, &hcan2, 0x02, 0x01, &hcan2,
                                          0x06, 0x03, &hcan1, 0x01, 0x00),

    .leg_init_config[1] = LEG_INIT_CONFIG(MOTOR_DIRECTION_REVERSE, MOTOR_DIRECTION_NORMAL, &hcan2, 0x08, 0x04, &hcan2,
                                          0x10, 0x05, &hcan1, 0x02, 0x00),
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
    // .imu_init_config = {.flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 0.0f, .Roll = 0.0f}
};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 0.3f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .DeadBand = 0.1f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 20.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 6000.0f,
                            .Ki = 100.0f,
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
                    // .tx_id = 2,
                    .tx_id = 0x104,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 0.5f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 20.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 5000.0f,
                            .Ki = 200.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 28000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    // .tx_id = 1,
                    .tx_id = 0x105,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
};

#define FRICTION_MOTOR_CONFIG(id, reverse_flag)                          \
  {                                                                      \
      .controller_param_init_config =                                    \
          {                                                              \
              .speed_PID =                                               \
                  {                                                      \
                      .Kp = 20.0f,                                       \
                      .Ki = 1.0f,                                        \
                      .Kd = 0.0f,                                        \
                      .Improve = PID_Integral_Limit,                     \
                      .IntegralLimit = 10000.0f,                         \
                      .MaxOut = 15000.0f,                                \
                  },                                                     \
              .current_PID =                                             \
                  {                                                      \
                      .Kp = 0.7f,                                        \
                      .Ki = 0.1f,                                        \
                      .Kd = 0.0f,                                        \
                      .Improve = PID_Integral_Limit,                     \
                      .IntegralLimit = 10000.0f,                         \
                      .MaxOut = 15000.0f,                                \
                  },                                                     \
          },                                                             \
      .motor_type = M3508,                                               \
      .can_init_config.can_handle = &hcan3,                              \
      .can_init_config.tx_id = id,                                       \
      .controller_setting_init_config.motor_reverse_flag = reverse_flag, \
  }

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = ONE_BULLET_DELTA_ANGLE,
            .reduction_ratio_loader = REDUCTION_RATIO_LOADER,
            .num_per_circle = NUM_PER_CIRCLE,
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(2, MOTOR_DIRECTION_REVERSE),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(1, MOTOR_DIRECTION_NORMAL),
    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 60.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .MaxOut = 20000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 1.0f,
                            .Ki = 0.1f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 10000.0f,
                        },
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan3,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
};

static PID_Init_Config_s chassis_follow_PID_config = {
    .Kp = 0.15f,
    .Ki = 0.0f,
    .Kd = 0.005f,  // todo: kd 大了
    .IntegralLimit = 0.1f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut = 1.0f,
};

static SuperCap_Init_Config_s super_cap_config = {
    .can_config = {
        .can_handle = &hcan3,
        .tx_id = 0x302,  // 超级电容默认接收id
        .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    }};