
#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#include "gimbal.h"
#include "shoot.h"

#define BOARD_TX_ID 0x10
#define BOARD_RX_ID 0x11

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 7000  // 【注意！】务必上电确定云台发射方向的ecd并填入，切勿上电就将左拨杆拨至中间
// #define PITCH_HORIZON_ECD 5748  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改(4310没ecd请忽略）

#define PITCH_MAX_ANGLE 10.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define YAW_MAX_ANGLE 30.0f   // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define YAW_MIN_ANGLE -30.0f  // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// 私有宏,自动将编码器转换成角度值
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
                            .Kp = 0.8f,
                            .Ki = 0.0f,
                            .Kd = 0.004f,
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 2000.0f,  // 4000
                            .Ki = 100.0f,   // 60
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 10000.0f,  // 20000,测试的时候务必用低一点的maxout的
                        },

                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 1,
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
                            .Kd = 0.005f,
                            .MaxOut = 15.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 0.3f,
                            .Ki = 0.5f,
                            .Kd = 0.0f,
                            .MaxOut = 4.0f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 2.0f,
                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x01,  // 0x01
                    .rx_id = 0x00,  // 0x206
                },
        },
    .imu_init_config = {.flag = 1,
                        .scale = {1.0f, 1.0f, 1.0f},
                        .offset_flag = 1,
                        .GyroOffset = {0.00119521946, -0.000239898043, 0.00218124315},
                        .Yaw = -90.0f,
                        .Pitch = 0.0f,
                        .Roll = 0.0f}};

#define FRICTION_MOTOR_CONFIG(handle, id, direction) \
  ((Motor_Init_Config_s){                            \
      .controller_param_init_config =                \
          {                                          \
              .speed_PID =                           \
                  {                                  \
                      .Kp = 1.5f,                    \
                      .Ki = 0.2f,                   \
                      .Kd = 0.0f,                     \
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
            .one_bullet_delta_angle = 51.4285714f,  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出60
            .reduction_ratio_loader = 36.0f,        // 2006
            .num_per_circle = 7,                    // 拨盘一圈的装载量6
            .loader_direction = 1,                  // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                      // 摩擦轮数量
            .friction_speed = 45000.0f,             // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},  // 摩擦轮速度比例系数
            .deadtime_burstfire = 100,
            .deadtime_onebullet = 500,  // 弹丸发射间隔
            .target_speed = 22.0f,
            .bullet_speed_adjustment = 0.0f,

        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 2, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 45.0f,  // 30
                            .Ki = 0.0f,
                            .Kd = 0.1f,          // 0.0
                            .MaxOut = 30000.0f,  // 50000
                        },
                    .speed_PID =
                        {
                            .Kp = 2.0f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,  // 7000
                            .MaxOut = 8000.0f,         // 16000
                        },
                },
            .motor_type = M2006,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

#endif  // CONTROL_2026_ROBOT_CONFIG_H