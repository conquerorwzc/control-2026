#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#include "gimbal.h"
#include "shoot.h"

#define BOARD_TX_ID 0x218
#define BOARD_RX_ID 0x219

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5326
#define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 11.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -15.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// // 二阶线性控制器参数
// #define YAW_FEED_FORWARD 0.7f
// #define YAW_ANGLE_ERROR_COEF 140000.0f
// #define YAW_ANGLE_SPEED_COEF 0.0f
// #define YAW_MAX_OUT 30000.0f
// #define YAW_MIN_OUT -30000.0f
//
// #define PITCH_FEED_FORWARD 0.8f
// #define PITCH_ANGLE_ERROR_COEF 0.01f
// #define PITCH_ANGLE_SPEED_COEF 0.0f
// #define PITCH_MAX_OUT 30000.0f
// #define PITCH_MIN_OUT -30000.0f

// //线性控制器前馈系数
// #define YAW_D 0.0f//15.0
// //角度误差项系数
// #define K_YAW_ANGLE_ERROR 	140000.0f//90000.0f
// #define K_PITCH_ANGLE_ERROR 0.01//350000.0f
// //Pitch前馈项
// #define PITCH_FORWARD	0.0f

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE 296.5 //hero的计算比较特殊，直接从读出来
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
#define GYRO2GIMBAL_DIR_YAW 1    // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1  // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1   // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

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
                      .MaxOut = 22.0f,
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
                    .can_handle = &hcan1,
                    .tx_id = 1,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                    {
                      .Kp = 0.0f,
                      .Ki = 0.01f,
                      .Kd = 0.0f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 0.0f,
                      .MaxOut = 10.0f,
                  },
              .speed_PID = {
                      .Kp = 0.0f,
                      .Ki = 0.0f,
                      .Kd = 0.0f,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 10000.0f,
                      .MaxOut = 30000.0f,
                  },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 1,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
    // .yaw_motor_second_order_linear_controller_init_config = {
    //     .k_feed_forward = YAW_FEED_FORWARD,
    //     .k_angle_error = YAW_ANGLE_ERROR_COEF,
    //     .k_angle_speed = YAW_ANGLE_SPEED_COEF,
    //     .max_out = YAW_MAX_OUT,
    //     .min_out = YAW_MIN_OUT,
    // },
    // .pitch_motor_second_order_linear_controller_init_config = {
    //     .k_feed_forward = PITCH_FEED_FORWARD,
    //     .k_angle_error = PITCH_ANGLE_ERROR_COEF,
    //     .k_angle_speed = PITCH_ANGLE_SPEED_COEF,
    //     .max_out = PITCH_MAX_OUT,
    //     .min_out = PITCH_MIN_OUT,
    // },
  .imu_init_config = {
      .flag = 1,
      .scale = {1.0f, 1.0f, 1.0f},
      .Yaw = 0.0f,
      .Pitch = 0.0f,
      .Roll = 0.0f
    }
};

// #define FRICTION_MOTOR_CONFIG(handle, id, direction) \
// ((Motor_Init_Config_s) { \
// .controller_param_init_config = { \
// .speed_PID = { \
// .Kp = 2.0f, \
// .Ki = 0.00f, \
// .Kd = 0.05f, \
// .Improve = PID_Integral_Limit, \
// .IntegralLimit = 10000.0f, \
// .MaxOut = 15000.0f, \
// }, \
// }, \
// .controller_setting_init_config ={\
// .angle_feedback_source = MOTOR_FEED,\
// .speed_feedback_source = MOTOR_FEED,\
// .outer_loop_type = SPEED_LOOP,\
// .close_loop_type = SPEED_LOOP,\
// .motor_reverse_flag = direction,\
// },\
// .motor_type = M3508, \
// .can_init_config = { \
// .can_handle = handle, \
// .tx_id = id, \
// }, \
// })
//
// static Shoot_Init_Config_s shoot_init_config = {
//     .shoot_param =
//         {
//             .one_bullet_delta_angle = 60.0f,          // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
//             .reduction_ratio_loader = 100.0f,         // 3508拨盘电机的减速比,英雄
//             .num_per_circle = 6,                      // 拨盘一圈的装载量
//             .loader_direction = -1,                    // 拨盘旋转方向,1为正向，-1为反向
//             .friction_num = 2,                        //摩擦轮数量
//             .friction_speed = 26000.0f,               //摩擦轮速度
//             .friction_coefficients = {1.0f, 1.0f},//摩擦轮速度比例系数
//             .deadtime_burstfire = 500,
//             .deadtime_onebullet = 1000,
//             .target_speed = 12.0f,
//             .bullet_speed_adjustment = 10.0f,
//
//         },
//     .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 8, MOTOR_DIRECTION_NORMAL),
//     //.friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 0x206, MOTOR_DIRECTION_REVERSE),
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

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 8,        // 接收数据长度，根据实际需求调整
  .send_data_len = 8,        // 发送数据长度，根据实际需求调整
  .daemon_count = 1000,      // 看门狗重载计数，根据实际需求调整
  .can_config = {
    .can_handle = &hcan1,  // 假设使用CAN1，根据实际使用的CAN句柄调整
    .tx_id = BOARD_TX_ID,        // 发送ID，根据实际需求调整
    .rx_id = BOARD_RX_ID,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &hcan1,
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
