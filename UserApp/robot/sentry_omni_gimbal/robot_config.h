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
#define GIMBAL_BOARD  // 单板控制整车
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define CANID_MAIN_TX 0x03
#define CANID_MAIN_RX 0x01
#define CANID_MOTION_TX 0x210
#define CANID_MOTION_RX 0x211
#define CANID_GAME_STATE_TX 0x212
#define CANID_GAME_STATE_RX 0x213

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 3845  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 2900      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 165.0f   // 云台竖直方向最大角度 (注意是电机角度)
#define PITCH_MIN_ANGLE 100.0f  // 云台竖直方向最小角度 (注意是电机角度)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH -1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                    {
                      .Kp = 2.3f,
                      .Ki = 0.0f,
                      .Kd = 0.03f,
                      .DeadBand = 0.01f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5.0f,
                      .MaxOut = 50.0f,
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
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                    {
                      .Kp = 2.5f,//可以调小点
                      .Ki = 0.0f,
                      .Kd = 0.035f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 5.0f,
                      .MaxOut = 25.0f,
                  },
              .speed_PID = {
                      .Kp = 5500.0f,
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
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
    .imu_init_config = {
        .flag = 1,
        .offset_flag=1,
        .scale = {1.0f, 1.0f, 1.0f},
        .Yaw = -90.0f,
        .Pitch = 0.0f,
        .Roll = 0.0f,
        .GyroOffset ={0.0038355093,-0.00053135067,0.00193249178},
      },
};
//原来参数是2.0f, 0.2f, 0.0f
//lsy参数是1.5f, 0.1f, 0.0f
#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
  ((Motor_Init_Config_s){                            \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 1.5f,                    \
                      .Ki = 0.1f,                    \
                      .Kd = 0.0f,                    \
                      .Improve = PID_Integral_Limit, \
                      .IntegralLimit = 10000.0f,     \
                      .MaxOut = 15000.0f,            \
                  },                                 \
          },                                         \
      .controller_setting_init_config =              \
          {                                          \
              .angle_feedback_source = MOTOR_FEED,   \
              .speed_feedback_source = MOTOR_FEED,   \
              .outer_loop_type = SPEED_LOOP,         \
              .close_loop_type = SPEED_LOOP,         \
              .motor_reverse_flag = direction,       \
              .feedback_reverse_flag = direction,    \
          },                                         \
      .motor_type = M3508,                           \
      .can_init_config =                             \
          {                                          \
              .can_handle = handle,                  \
              .tx_id = id,                           \
          },                                         \
  })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = 36.0f,              // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
            .reduction_ratio_loader = 66.0f,              // M2006拨盘电机的减速比
            .num_per_circle = 10,                          // 拨盘一圈的装载量
            .loader_direction = 1,                        // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                            // 摩擦轮数量
            .friction_speed = 37500.0f,                   // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},  // 摩擦轮速度比例系数。
            .deadtime_burstfire = 56,
            .deadtime_onebullet = 500,
            .target_speed = 22.0f,
            .bullet_speed_adjustment = 50.0f,
            .bullet_speed_deadband = 0.2f,//弹速死区，正负deadband。
            .one_barrel_heat_value = 10,//一发弹丸所需热量
            .shooter_barrel_cooling_value = 30,//每秒冷却回复
            // .shooter_barrel_heat_limit = 210,//热量上限
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 2, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 1, MOTOR_DIRECTION_NORMAL),
  .loader_motor_config =
    {
      .controller_param_init_config =
      {
        .angle_PID =
        {
          .Kp = 70.0f,
          .Ki = 0.0f,
          .Kd = 1.5f,
          .MaxOut = 30000.0f,
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
.motor_type = M2006, //拨盘电机为M2006
.can_init_config =
      {
        .can_handle = &hcan2,
        .tx_id = 4,
    },
      .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
      .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

.controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
.controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
.controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
},
};


#ifdef USE_DUAL_RC_NEW
#define DUALBOARD_CMD_LEN ((uint8_t)sizeof(Send_Data_RC_NEW))
#else
#define DUALBOARD_CMD_LEN ((uint8_t)sizeof(Send_Data_RC))
#endif
#define DUALBOARD_REF_MAIN_LEN ((uint8_t)sizeof(Referee_Main_s))
#define DUALBOARD_REF_MOTION_LEN ((uint8_t)sizeof(Chassis_Motion_s))
#define DUALBOARD_REF_GAME_STATE_LEN ((uint8_t)sizeof(Referee_Game_State_s))

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = DUALBOARD_REF_MAIN_LEN,
  .send_data_len = DUALBOARD_CMD_LEN,
  .daemon_count = 10,
  .can_config = {
    .can_handle = &hcan2,  // 假设使用CAN2，根据实际使用的CAN句柄调整
    .tx_id = CANID_MAIN_TX,        // 发送ID，根据实际需求调整
    .rx_id = CANID_MAIN_RX,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

static CANComm_Init_Config_s comm_config_motion = {
  .recv_data_len = DUALBOARD_REF_MOTION_LEN,
  .send_data_len = 0,
  .daemon_count = 10,
  .can_config = {
    .can_handle = &hcan2,
    .tx_id = CANID_MOTION_TX,
    .rx_id = CANID_MOTION_RX,
    .id = NULL
  }
};

static CANComm_Init_Config_s comm_config_game_state = {
  .recv_data_len = DUALBOARD_REF_GAME_STATE_LEN,
  .send_data_len = 0,
  .daemon_count = 200,
  .can_config = {
    .can_handle = &hcan2,
    .tx_id = CANID_GAME_STATE_TX,
    .rx_id = CANID_GAME_STATE_RX,
    .id = NULL
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &hcan2,
  .tx_id = CANID_MAIN_TX,          // 与comm_config中的ID保持一致
  .rx_id = CANID_MAIN_RX,
  .txconf = {
    .StdId = CANID_MAIN_TX,      // 发送ID
    .IDE = CAN_ID_STD,   // 标准帧
    .RTR = CAN_RTR_DATA, // 数据帧
    .DLC = 0x08,         // 数据长度8字节
  }
};

// static SuperCap_Init_Config_s super_cap_config = {
//     .can_config = {
//         .can_handle = &hcan2,
//         .tx_id = 0x302,  // 超级电容默认接收id
//         .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
//     }};