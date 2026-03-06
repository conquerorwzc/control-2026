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
#pragma message                                                                                                        \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义!
 */
#define ONE_BOARD // 单板控制整车

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||                 \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

//  轮电机参数模板，追求响应一致，所以参数一样的，只有id有所区别
#define WHEEL_MOTOR_CONFIG(handle, id)                                                                                 \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 0.5,                                                                                     \
                        .Ki = 0,                                                                                       \
                        .Kd = 0,                                                                                       \
                        .IntegralLimit = 6000,                                                                         \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,       \
                        .MaxOut = 15000,                                                                               \
                    },                                                                                                 \
                .current_PID =                                                                                         \
                    {                                                                                                  \
                        .Kp = 0,                                                                                       \
                        .Ki = 0,                                                                                       \
                        .Kd = 0,                                                                                       \
                        .IntegralLimit = 3000,                                                                         \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,       \
                        .MaxOut = 15000,                                                                               \
                    },                                                                                                 \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = SPEED_LOOP,                                                                         \
                .close_loop_type = SPEED_LOOP,                                                                         \
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,                                                         \
                .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,                                                      \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
    })
#define LIFT_FORWARD_MOTOR_CONFIG(handle, id, direction)                                                               \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {.angle_PID =                                                                                              \
                 {                                                                                                     \
                     .Kp = 10.0f,                                                                                      \
                     .Ki = 0,                                                                                          \
                     .Kd = 0,                                                                                          \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 3000.0f,                                                                                \
                 },                                                                                                    \
             .speed_PID =                                                                                              \
                 {                                                                                                     \
                     .Kp = 4.0f,                                                                                       \
                     .Ki = 0,                                                                                          \
                     .Kd = 0,                                                                                          \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 16000.0f,                                                                               \
                 }                                                                                                     \
                                                                                                                       \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                            \
                .motor_reverse_flag = direction,                                                                       \
                .feedback_reverse_flag = direction,                                                                    \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
    })
#define LIFT_BACKWARD_MOTOR_CONFIG(handle, id, direction)                                                              \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {.angle_PID =                                                                                              \
                 {                                                                                                     \
                     .Kp = 40.0f,                                                                                      \
                     .Ki = 0,                                                                                          \
                     .Kd = 0,                                                                                          \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 24000.0f,                                                                               \
                 },                                                                                                    \
             .speed_PID =                                                                                              \
                 {                                                                                                     \
                     .Kp = 5.0f,                                                                                       \
                     .Ki = 0.0f,                                                                                       \
                     .Kd = 0.0f,                                                                                       \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 1460.0f,                                                                                \
                 }},                                                                                                   \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                            \
                .motor_reverse_flag = direction,                                                                       \
                .feedback_reverse_flag = direction,                                                                    \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
    })
// TODO:电机pid参数待测，方向待测，2006/3508
/**
 *
 */
static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            // 机器人底盘修改的参数,单位为mm(毫米)
            .wheel_base = 400.0f,           // 纵向轴距(前进后退方向)
            .track_width = 470.0f,          // 横向轮距(左右平移方向)
            .center_gimbal_offset_x = 0.0f, // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
            .center_gimbal_offset_y = 0.0f, // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
            .wheel_radius = 60.0f,          // 轮子半径
            .wheel_reduction_ratio = 19.0f, // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
                                            // 3508功率模型参数
            .power_param.k0 = 0.7441993412640775f,
            .power_param.k1 = 0.006444284468539646f,
            .power_param.k2 = 0.0001423857226262331f,
            .power_param.k3 = 0.015644430204543864f,
            .power_param.k4 = 0.1580143850678086f,
            .power_param.k5 = 2.896721772539512e-05f,

            .forward_lift_in = 0,
            .forward_lift_out = 13000.0f,
            .backward_lift_in = 1500.0f,
            .backward_lift_out = 313280.938f,

        },
    .wheel_motor_config[0] = WHEEL_MOTOR_CONFIG(&hcan3, 1),
    .wheel_motor_config[1] = WHEEL_MOTOR_CONFIG(&hcan3, 4),
    .wheel_motor_config[2] = WHEEL_MOTOR_CONFIG(&hcan3, 2),
    .wheel_motor_config[3] = WHEEL_MOTOR_CONFIG(&hcan3, 3),
    .lift_forward_motor_config[0] = // 前左，0是左，1是右
    {
        .controller_param_init_config =
            {

                .angle_PID =
                    {
                        .Kp = 13.0f,
                        .Ki = 0,
                        .Kd = 0,
                        .IntegralLimit = 0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 4000.0f,
                    },
                .speed_PID =
                    {
                        .Kp = 4.0f,
                        .Ki = 0,
                        .Kd = 0,
                        .IntegralLimit = 0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 16000.0f,
                    }

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
        .motor_type = M3508,
        .can_init_config =
            {
                .can_handle = &hcan2,
                .tx_id = 2,
            },
    },                                                                                               // 后右
    .lift_forward_motor_config[1] = LIFT_FORWARD_MOTOR_CONFIG(&hcan2, 1, MOTOR_DIRECTION_REVERSE),   // 前右
    .lift_backward_motor_config[0] = LIFT_BACKWARD_MOTOR_CONFIG(&hcan2, 3, MOTOR_DIRECTION_REVERSE), // 后左
    .lift_backward_motor_config[1] =
        {
            .controller_param_init_config =
                {

                    .angle_PID =
                        {
                            .Kp = 40.0f,
                            .Ki = 0.00f,
                            .Kd = 0.00f,
                            .MaxOut = 23600.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 0.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 5.0f,
                            .Ki = 0.0f,
                            .Kd = 0.00f,
                            .MaxOut = 1200.0f,
                            .DeadBand = 0.00f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,

                        },
                },
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 4,
                },
        }, // 后右
    // 跟随PID
    .follow_pid =
        {
            .Kp = 60.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 2000.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 10000.0f,
        },

};

static IMU_Init_Config_s imu_init_config = {
    .flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 0.0f, .Roll = 0.0f};
// 龙门架M3508电机配置宏 (抬升和前伸电机)
#define GANTRY_M3508_CONFIG(handle, id, angle_kp, angle_kd, speed_kp, speed_ki, direction)                             \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .angle_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = angle_kp,                                                                                \
                        .Ki = 0.0f,                                                                                    \
                        .Kd = angle_kd,                                                                                \
                        .IntegralLimit = 1000.0f,                                                                      \
                        .MaxOut = 7500.0f,                                                                             \
                    },                                                                                                 \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = speed_kp,                                                                                \
                        .Ki = speed_ki,                                                                                \
                        .Kd = 0.0f,                                                                                    \
                        .IntegralLimit = 15000.0f,                                                                     \
                        .MaxOut = 30000.0f,                                                                            \
                    },                                                                                                 \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                                            \
                .motor_reverse_flag = direction,                                                                       \
                .feedback_reverse_flag = direction,                                                                    \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
    })

// 龙门架M2006电机配置宏 (横移电机)
#define GANTRY_M2006_CONFIG(handle, id, angle_kp, angle_kd, speed_kp, speed_ki, direction)                             \
    ((Motor_Init_Config_s){                                                                                            \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .angle_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = angle_kp,                                                                                \
                        .Ki = 0.0f,                                                                                    \
                        .Kd = angle_kd,                                                                                \
                        .IntegralLimit = 600.0f,                                                                       \
                        .MaxOut = 6000.0f,                                                                             \
                    },                                                                                                 \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = speed_kp,                                                                                \
                        .Ki = speed_ki,                                                                                \
                        .Kd = 0.0f,                                                                                    \
                        .IntegralLimit = 5000.0f,                                                                      \
                        .MaxOut = 15000.0f,                                                                            \
                    },                                                                                                 \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                                            \
                .motor_reverse_flag = direction,                                                                       \
                .feedback_reverse_flag = direction,                                                                    \
            },                                                                                                         \
        .motor_type = M2006,                                                                                           \
    })

// 龙门架初始化配置实例
static Gantry_Init_Config_s gantry_init_config = {
    // ------------------- 龙门架系统参数配置 -------------------
    .Gantry_param =
        {
            .GANTRY_MAX_Y = 19000.0f, // 前伸最前位置
            .GANTRY_MAX_Z = 40000.0f, // 抬升最高位置
            .GANTRY_MAX_X = 17000.0f, // 横移最右位置

            .lift_sens_remote = 0.001f,     // 抬升电机灵敏度(遥控器)
            .stretch_sens_remote = 0.001f,  // 前伸电机灵敏度(遥控器)
            .sidesway_sens_remote = 0.015f, // 横移电机灵敏度(遥控器)

            .lift_sens_keyboard = 0.1f,     // 抬升电机灵敏度(键鼠)
            .stretch_sens_keyboard = 6.0f,  // 前伸电机灵敏度(键鼠)
            .sidesway_sens_keyboard = 7.0f, // 横移电机灵敏度(键鼠)

            .position_ecd_ratio = 30.0f,
        }, // 位置矢量与电机转动角度的比例

    // 抬升电机 (3508)
    .lift_motor_config[0] = GANTRY_M3508_CONFIG(&hfdcan2, 1,              // CA0N 句柄和 ID
                                                50.0f, 2.3f,              // 角度环 Kp, Kd
                                                1.5f, 0.0f,               // 速度环 Kp, Ki
                                                MOTOR_DIRECTION_NORMAL),  // 电机方向 (对应老代码中的 - ratio)
    .lift_motor_config[1] = GANTRY_M3508_CONFIG(&hfdcan2, 2, 50.0f, 2.3f, // 角度环 Kp, Kd
                                                1.5f, 0.0f,               // 速度环 Kp, Ki
                                                MOTOR_DIRECTION_NORMAL),  // 电机方向 (对应老代码中的 + ratio)

    // 前伸电机 (3508)
    .stretch_motor_config[0] = GANTRY_M3508_CONFIG(&hfdcan2, 3, 50.0f, 2.8f, 1.33f, 0.00f, MOTOR_DIRECTION_NORMAL),
    .stretch_motor_config[1] = GANTRY_M3508_CONFIG(&hfdcan2, 4, 45.0f, 2.7f, 1.29f, 0.00f, MOTOR_DIRECTION_NORMAL),

    // 横移电机 (2006)
    .sidesway_motor_config = GANTRY_M2006_CONFIG(&hfdcan2, 5, 0.0f, 0.0f, 0.0f, 0.0f,
                                                 MOTOR_DIRECTION_REVERSE), // 电机方向 (对应老代码中的 - ratio)
};
static Grab_Init_Config_s
    grab_init_config =
        {
            .Grab_cali_mode = GRAB_PRE_CALI_MODE,

            .Grab_param =
                {
                    .base_joint_sens_keyboard = 0.05,
                    .elbow_roll_sens_keyboard = 0.05,
                    .elbow_pitch_sens_keyboard = 0.05,
                    .wrist_roll_sens_keyboard = 0.05,
                    .wrist_pitch_sens_keyboard = 0.05,
                    .video_forward_sens_keyboard = 0.01,
                    .video_pitch_sens_keyboard = 0.01,
                },

            .Grab_motor_config[0] =
                {
                    .controller_param_init_config =
                        {
                            .angle_PID =
                                {
                                    .Kp = 12.0f, // 120
                                    .Ki = 0.00f,
                                    .Kd = 0.00f,
                                    .MaxOut = 8.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 0.5f, // 0.5
                                    .Ki = 0.1f, // 0.1
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
                    .motor_type = J4340,
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
                                    .Kp = 15.0f,
                                    .Ki = 0.00f,
                                    .Kd = 0.00f,
                                    .MaxOut = 2.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 6.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.00f,
                                    .MaxOut = 28.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.5f,
                                },
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
                    .motor_type = J4340,
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
                                    .Kp = 15.0f,
                                    .Ki = 0.00f,
                                    .Kd = 0.00f,
                                    .MaxOut = 2.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 6.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.00f,
                                    .MaxOut = 28.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.5f,
                                },
                        },
                    .controller_setting_init_config =
                        {
                            .outer_loop_type = ANGLE_LOOP,
                            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                            .angle_feedback_source = MOTOR_FEED,
                            .speed_feedback_source = MOTOR_FEED,
                            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                        },
                    .motor_type = J4340,
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
                                    .MaxOut = 15000.0f,
                                },
                            .speed_PID = {.Kp = 2.0f,
                                          .Ki = 0.0f,
                                          .Kd = 0.0f,
                                          .Improve = PID_Integral_Limit | PID_ErrorHandle,
                                          .IntegralLimit = 0.0f,
                                          .MaxOut = 12000.0},
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
                            .tx_id = 5,
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
                                    .MaxOut = 15000.0f,
                                },
                            .speed_PID = {.Kp = 2.0f,
                                          .Ki = 0.0f,
                                          .Kd = 0.0f,
                                          .Improve = PID_Integral_Limit | PID_ErrorHandle,
                                          .IntegralLimit = 0.0f,
                                          .MaxOut = 12000.0},
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
                            .tx_id = 6,
                        },

                },
            .Grab_motor_config[5] =
                {
                    .controller_param_init_config =
                        {
                            .current_PID =
                                {
                                    .Kp = 0.0f, // 12
                                    .Ki = 0.00f,
                                    .Kd = 0.00f,
                                    .MaxOut = 8.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 3.0f,
                                },
                            .angle_PID =
                                {
                                    .Kp = 12.0f, // 12
                                    .Ki = 0.00f,
                                    .Kd = 0.00f,
                                    .MaxOut = 8.0f,
                                    .DeadBand = 0.01f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 0.5f, // 0.5
                                    .Ki = 0.1f, // 0.1
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
                            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                            .angle_feedback_source = MOTOR_FEED,
                            .speed_feedback_source = MOTOR_FEED,
                            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                        },
                    .motor_type = J4310,
                    .can_init_config =
                        {
                            .can_handle = &hcan1,
                            .tx_id = 0x04,
                            .rx_id = 0x14,
                        },
                },
            .Grab_motor_config[6] =
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
                            .tx_id = 3,
                        },
                },
            .Grab_motor_config[7] =
                {
                    .controller_param_init_config =
                        {
                            .angle_PID =
                                {
                                    .Kp = 66.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                    .MaxOut = 1500.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 2.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 10000.0f,
                                    .MaxOut = 15000.0f,
                                },

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
                    .motor_type = M3508,
                    .can_init_config =
                        {
                            .can_handle = &hcan1,
                            .tx_id = 4,
                        },

                },

            .Grab_motor_config[8] =
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
                            .tx_id = 7,
                        },
                },
};

// static SuperCap_Init_Config_s super_cap_config = {
//     .can_config = {
//         .can_handle = &hcan2,
//         .tx_id = 0x302,  // 超级电容默认接收id
//         .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
//     }};
