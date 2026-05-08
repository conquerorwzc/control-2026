/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "robot.h"
#include "can_comm.h"

/* 输出车体角速度 rad/s */
#define Wheel_radius 76.475f
#define Wheel_base 345.96f
#define Reduction_ratio 19.0f
#define WZ_CMD_TO_CAR_WZ_RAD_S (DEGREE_2_RAD * DEGREE_2_RAD * Wheel_radius / Reduction_ratio) //wz乘以这个参数，得出的就是车辆的实际转动角速度


// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define CHASSIS_BOARD  // 单板控制整车
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据



//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 100
//双板can通信设备
#define BOARD_CAN hcan2
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 3845  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 3494      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 30.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

//轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
#define WHEEL_MOTOR_CONFIG(handle, id) \
((Motor_Init_Config_s) { \
    .can_init_config = { \
        .can_handle = handle, \
        .tx_id = id, \
    }, \
    .controller_param_init_config = { \
        .speed_PID = { \
            .Kp = 4, \
            .Ki = 0, \
            .Kd = 0, \
            .IntegralLimit = 8000, \
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
            .MaxOut = 16000, \
        }, \
        .current_PID = { \
            .Kp = 0, \
            .Ki = 0, \
            .Kd = 0, \
            .IntegralLimit = 3000, \
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
            .MaxOut = 15000, \
        }, \
    }, \
    .controller_setting_init_config = { \
        .angle_feedback_source = MOTOR_FEED, \
        .speed_feedback_source = MOTOR_FEED, \
        .outer_loop_type = SPEED_LOOP, \
        .close_loop_type = SPEED_LOOP, \
        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,\
        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,\
    }, \
    .motor_type = M3508, \
})

static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            // 机器人底盘修改的参数,单位为mm(毫米)
            .wheel_base = Wheel_base,              // 纵向轴距(前进后退方向)
            .track_width = 345.96f,             // 横向轮距(左右平移方向)
            .center_gimbal_offset_x = 0.0f,    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
            .center_gimbal_offset_y = 0.0f,    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
            .wheel_radius = Wheel_radius,             // 轮子半径
            .wheel_reduction_ratio = Reduction_ratio,  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
            //3508功率模型参数
      .power_param.k0=0.7441993412640775f,
      .power_param.k1=0.0090164284468539646f,
      .power_param.k2=0.0001988857226262331f,
      .power_param.k3=0.024694430204543864f,
      .power_param.k4=0.20160143850678086f,
      .power_param.k5=3.715221772539512e-05f,
        },
    .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan1,1),
    .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan1,4),
    .wheel_motor_config[2] = WHEEL_MOTOR_CONFIG(&hcan1,2),
    .wheel_motor_config[3] = WHEEL_MOTOR_CONFIG(&hcan1,3),
    //跟随PID
    .follow_pid={
        .Kp = -70.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .DeadBand = 1.0f,
        .IntegralLimit = 1000.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 20000.0f,
    },
  .yaw_hold_pid = {
      .Kp = -80.0f,
      .Ki = 0.0f,
      .Kd = 0.0f,
      .DeadBand = 1.0f,
      .IntegralLimit = 1000.0f,
      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
      .MaxOut = 10000.0f,
  },
  .imu_init_config = {
      .flag = 1,
      .scale = {1.0f, 1.0f, 1.0f},
      .Yaw = 0.0f,
      .Pitch = 0.0f,
      .Roll = 0.0f,
  },
};

static SuperCap_Init_Config_s super_cap_config = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x302,  // 超级电容默认接收id
        .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    }};

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H
#if DEVICE_ROLE_TX
// 发送板配置
#define BOARD_TX_ID 0x218
#define BOARD_RX_ID 0x219
#else
// 接收板配置
#define BOARD_TX_ID 0x211
#define BOARD_RX_ID 0x210
#endif

#ifdef USE_DUAL_RC_NEW
#define DUALBOARD_CMD_LEN ((uint8_t)sizeof(Send_Data_RC_NEW))
#else
#define DUALBOARD_CMD_LEN ((uint8_t)sizeof(Send_Data_RC))
#endif
#define DUALBOARD_REF_LEN ((uint8_t)sizeof(Referee_Data))

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = DUALBOARD_CMD_LEN,
  .send_data_len = DUALBOARD_REF_LEN,
  .daemon_count = 10,
  .can_config = {
    .can_handle = &hcan2,  // 假设使用CAN1，根据实际使用的CAN句柄调整
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
#endif  // CONTROL_2026_ROBOT_CONFIG_H