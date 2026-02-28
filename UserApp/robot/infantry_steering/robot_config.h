/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"
#include "super_cap.h"

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
#define YAW_CHASSIS_ALIGN_ECD 1570
#define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 11.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -15.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // yaw与底盘对齐时电机的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
#define WHEEL_MAX_SLEW_RATE 100.0f
//  轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
#define WHEEL_MOTOR_CONFIG(handle, id,reverse_flag)                                                                         \
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
                      .Kp = 1,                                                                               \
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
              .motor_reverse_flag = reverse_flag,                                                              \
              .feedback_reverse_flag = reverse_flag,                                                           \
          },                                                                                                   \
      .motor_type = M3508,                                                                                     \
  })
#define RUDDER_MOTOR_CONFIG(handle, id)                                                                        \
  ((Motor_Init_Config_s){                                                                                      \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = handle,                                                                            \
              .tx_id = id,                                                                                     \
          },                                                                                                   \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .angle_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 70,                                                                                \
                      .Ki = 0,                                                                                 \
                      .Kd = 0.4,                                                                               \
                      .IntegralLimit = 960,                                                                  \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 1920,                                                                          \
                  },                                                                                           \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 9,                                                                                 \
                      .Ki = 25,                                                                                \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 12500,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 20000,                                                                         \
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
              .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,                                                           \
          },                                                                                                   \
      .motor_type = GM6020,                                                                                    \
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
          .power_param_6020.k0 = 1.0015598804886459f,
          .power_param_6020.k1 = -0.0011271483180564656f,
          .power_param_6020.k2 = 0.1628643511122505f,
          .power_param_6020.k3 = 0.630911350030345f,
          .power_param_6020.k4 = 0.001254230359389605f,
          .power_param_6020.k5 = 7.632464718194001f,

            // 6020舵机电机零位偏移值，用于校准安装后的零偏
            .rudder_motor_offset = {6844, 3443,13, 3388}  // 根据实际安装情况设置每个舵机的零偏值LFLBRBRF
        },
     .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan1, 1,MOTOR_DIRECTION_REVERSE),
     .wheel_motor_config[2] = WHEEL_MOTOR_CONFIG(&hcan1, 4,MOTOR_DIRECTION_REVERSE),
     .wheel_motor_config[3] = WHEEL_MOTOR_CONFIG(&hcan1, 2,MOTOR_DIRECTION_REVERSE),
     .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan1, 3,MOTOR_DIRECTION_REVERSE),
    .rudder_motor_config[0] = RUDDER_MOTOR_CONFIG(&hcan2, 1),
    .rudder_motor_config[1] = RUDDER_MOTOR_CONFIG(&hcan2, 3),
    .rudder_motor_config[2] = RUDDER_MOTOR_CONFIG(&hcan2, 4),
    .rudder_motor_config[3] = RUDDER_MOTOR_CONFIG(&hcan2, 2),
  .yaw_motor_config=RUDDER_MOTOR_CONFIG(&hcan1, 1),
    //跟随PID
    .follow_pid={
        .Kp = 250.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .IntegralLimit = 1000.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 40000.0f,
      .DeadBand = 0.0f,

    },

};

// static Gimbal_Init_Config_s gimbal_init_config = {
//     .yaw_motor_config =
//         {
//             .controller_param_init_config =
//                 {
//                     .angle_PID =
//                     {
//                       .Kp = -1.5f,
//                       .Ki = 0.0f,
//                       .Kd = -0.08f,
//                       .DeadBand = 0.1f,
//                       .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                       .IntegralLimit = 5.0f,
//                       .MaxOut = 22.0f,
//                   },
//                     .speed_PID =
//                     {
//                       .Kp = 5500.0f,
//                       .Ki = 70.0f,
//                       .Kd = 0.0f,
//                       .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                       .IntegralLimit = 12000.0f,
//                       .MaxOut = 26000.0f,
//                   },
//
//                 },
//             .motor_type = GM6020,
//             .can_init_config =
//                 {
//                     .can_handle = &hcan1,
//                     .tx_id = 2,
//                 },
//             .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
//         },
//     .pitch_motor_config =
//         {
//             .controller_param_init_config =
//                 {
//                     .angle_PID =
//                     {
//                       .Kp = 0.8f,
//                       .Ki = 0.0f,
//                       .Kd = 0.0f,
//                       .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                       .IntegralLimit = 5.0f,
//                       .MaxOut = 25.0f,
//                   },
//               .speed_PID = {
//                       .Kp = 20000.0f,
//                       .Ki = 120.0f,
//                       .Kd = 0.0f,
//                       .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                       .IntegralLimit = 12000.0f,
//                       .MaxOut = 26000.0f,
//                   },
//                 },
//             .motor_type = GM6020,
//             .can_init_config =
//                 {
//                     .can_handle = &hcan2,
//                     .tx_id = 1,
//                 },
//             .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
//         },
// };

// #define FRICTION_MOTOR_CONFIG(handle, id, direction) \
// ((Motor_Init_Config_s) { \
// .controller_param_init_config = { \
// .speed_PID = { \
// .Kp = 0.5f, \
// .Ki = 0.0f, \
// .Kd = 0.0f, \
// .Improve = PID_Integral_Limit, \
// .IntegralLimit = 10000.0f, \
// .MaxOut = 15000.0f, \
// }, \
// }, \
// .controller_setting_init_config ={\
//   .angle_feedback_source = MOTOR_FEED,\
//   .speed_feedback_source = MOTOR_FEED,\
//   .outer_loop_type = SPEED_LOOP,\
//   .close_loop_type = SPEED_LOOP,\
//   .motor_reverse_flag = direction,\
// },\
// .motor_type = M3508, \
// .can_init_config = { \
// .can_handle = handle, \
// .tx_id = id, \
// }, \
// })

// static Shoot_Init_Config_s shoot_init_config = {
//     .shoot_param =
//         {
//             .one_bullet_delta_angle = 60.0f,          // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
//             .reduction_ratio_loader = 100.0f,         // 3508拨盘电机的减速比,英雄
//             .num_per_circle = 6,                      // 拨盘一圈的装载量
//             .loader_direction = -1,                    // 拨盘旋转方向,1为正向，-1为反向
//             .friction_num = 3,                        //摩擦轮数量
//         },
//     .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan2, 1, MOTOR_DIRECTION_NORMAL),
//     .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan2, 2, MOTOR_DIRECTION_REVERSE),
//     //.friction_motor_config[2] = FRICTION_MOTOR_CONFIG(&hcan2, 4, MOTOR_DIRECTION_NORMAL),
//
//     .loader_motor_config =
//         {
//             .controller_param_init_config =
//                 {
//                     .angle_PID =
//                         {
//                             .Kp = 30.0f,
//                             .Ki = 0.0f,
//                             .Kd = 0.0f,
//                             .MaxOut = 50000.0f,
//                         },
//                     .speed_PID =
//                         {
//                             .Kp = 2.0f,
//                             .Ki = 0.4f,
//                             .Kd = 0.0f,
//                             .Improve = PID_Integral_Limit | PID_ErrorHandle,
//                             .IntegralLimit = 7000.0f,
//                             .MaxOut = 16000.0f,
//                         },
//                 },
//             .motor_type = M3508,
//             .can_init_config =
//                 {
//                     .can_handle = &hcan2,
//                     .tx_id = 3,
//                 },
//             .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//             .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
//             .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
//             .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
//             .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
//         },
// };

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#pragma once

#include "robot.h"

#if DEVICE_ROLE_TX
// 发送板配置
#define BOARD_TX_ID 0x218
#define BOARD_RX_ID 0x219
#else
// 接收板配置
#define BOARD_TX_ID 0x311
#define BOARD_RX_ID 0x10
#endif

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 26,        // 接收数据长度，根据实际需求调整
  .send_data_len = 28,        // 发送数据长度，根据实际需求调整
  .daemon_count = 10,      // 看门狗重载计数，根据实际需求调整
  .can_config = {
    .can_handle = &hcan1,  // 假设使用CAN1，根据实际使用的CAN句柄调整
    .tx_id = BOARD_TX_ID,        // 发送ID，根据实际需求调整
    .rx_id = BOARD_RX_ID,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &BOARD_CAN,
  .tx_id = BOARD_TX_ID,          // 与comm_config中的ID保持一致
  .rx_id = BOARD_RX_ID,
  .txconf = {
    .StdId = BOARD_TX_ID,      // 发送ID
    .IDE = CAN_ID_STD,   // 标准帧
    .RTR = CAN_RTR_DATA, // 数据帧
    .DLC = 0x08,         // 数据长度8字节
  }
};
static SuperCap_Init_Config_s supercab_init_config = {
  .can_config = {
    .can_handle = &hcan2,  // 根据实际情况选择CAN接口，英雄的h7是can1
    .rx_id = 0x211,        // 接收ID (CAN_SUPERCAP_ID)
    .tx_id = 0X210,        // 发送ID
  }
};
//6020电机
#define GM6020_K0 0.8130f
#define GM6020_K1 -0.0005f
#define GM6020_K2 6.0021f
#define GM6020_K3 1.3715f
#define GM6020_Current_To_Out (3.0f/16384.0f)
#endif  // CONTROL_2026_ROBOT_CONFIG_H
