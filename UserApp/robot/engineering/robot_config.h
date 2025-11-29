/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
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

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5326
#define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 11.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -15.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE 296.5                                         // hero的计算比较特殊，直接从读出来
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
//  轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
#define WHEEL_MOTOR_CONFIG(handle, id)                                                                         \
  ((Motor_Init_Config_s){                                                                                      \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = handle,                                                                            \
              .tx_id = id,                                                                                     \
          },                                                                                                   \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 0.5,                                                                               \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 6000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
              .current_PID =                                                                                   \
                  {                                                                                            \
                      .Kp = 0,                                                                                 \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 3000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .outer_loop_type = SPEED_LOOP,                                                                   \
              .close_loop_type = SPEED_LOOP,                                                                   \
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,                                                   \
          },                                                                                                   \
      .motor_type = M3508,                                                                                     \
  })

static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            // 机器人底盘修改的参数,单位为mm(毫米)
            .wheel_base = 350.0f,            // 纵向轴距(前进后退方向)
            .track_width = 300.0f,           // 横向轮距(左右平移方向)
            .center_gimbal_offset_x = 0.0f,  // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
            .center_gimbal_offset_y = 0.0f,  // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
            .wheel_radius = 60.0f,           // 轮子半径
            .wheel_reduction_ratio = 19.0f,  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
                                             // 3508功率模型参数
            .power_param.k0 = 0.7441993412640775f,
            .power_param.k1 = 0.006444284468539646f,
            .power_param.k2 = 0.0001423857226262331f,
            .power_param.k3 = 0.015644430204543864f,
            .power_param.k4 = 0.1580143850678086f,
            .power_param.k5 = 2.896721772539512e-05f,
        },
    .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan3, 1),
    .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan3, 4),
    .wheel_motor_config[2] = WHEEL_MOTOR_CONFIG(&hcan3, 2),
    .wheel_motor_config[3] = WHEEL_MOTOR_CONFIG(&hcan3, 3),
    // 跟随PID
    .follow_pid =
        {
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 1000.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 10000.0f,
        },

};

static Grab_Init_Config_s grab_init_config_s = {
    .Grab_motor_config[0] =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 12.0f,  // 12
                            .Ki = 0.00f,
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 0.5f,  // 0.5
                            .Ki = 0.1f,  // 0.1
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.5f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x01,
                    .rx_id = 0x11,
                },
        },
    .Grab_motor_config[1] =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 12.0f,  // 12
                            .Ki = 0.00f,
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 0.5f,  // 0.5
                            .Ki = 0.1f,  // 0.1
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.5f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x02,
                    .rx_id = 0x12,
                },

        },
    .Grab_motor_config[2] =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 12.0f,  // 12
                            .Ki = 0.00f,
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 0.5f,  // 0.5
                            .Ki = 0.1f,  // 0.1
                            .Kd = 0.00f,
                            .MaxOut = 8.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.5f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x03,
                    .rx_id = 0x13,
                },

        },
    .Grab_motor_config[3] =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 30.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .MaxOut = 30000.0f,
                        },
                    .speed_PID = {.Kp = 2.0f,
                                  .Ki = 0.0f,
                                  .Kd = 0.0f,
                                  .Improve = PID_Integral_Limit | PID_ErrorHandle,
                                  .IntegralLimit = 0.0f,
                                  .MaxOut = 10000.0},
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },

        },
    .Grab_motor_config[4] =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 30.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .MaxOut = 30000.0f,
                        },
                    .speed_PID = {.Kp = 2.0f,
                                  .Ki = 0.0f,
                                  .Kd = 0.0f,
                                  .Improve = PID_Integral_Limit | PID_ErrorHandle,
                                  .IntegralLimit = 0.0f,
                                  .MaxOut = 10000.0},
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 1,
                },

        },
};

// static SuperCap_Init_Config_s super_cap_config = {
//     .can_config = {
//         .can_handle = &hcan2,
//         .tx_id = 0x302,  // 超级电容默认接收id
//         .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
//     }};