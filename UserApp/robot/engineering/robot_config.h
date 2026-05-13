/**
******************************************************************************
* @file    robot_config.h
* @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
******************************************************************************
*/
#pragma once

#include "gimbal_video.h"
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
                        .Kp = 6,                                                                                       \
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
                     .MaxOut = 3700.0f,                                                                                \
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
                     .Kp = 60.0f,                                                                                      \
                     .Ki = 0,                                                                                          \
                     .Kd = 0,                                                                                          \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 5200.0f,                                                                                \
                 },                                                                                                    \
             .speed_PID =                                                                                              \
                 {                                                                                                     \
                     .Kp = 4.0f,                                                                                       \
                     .Ki = 0.0f,                                                                                       \
                     .Kd = 0.0f,                                                                                       \
                     .IntegralLimit = 0,                                                                               \
                     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,          \
                     .MaxOut = 10000.0f,                                                                               \
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
            .wheel_reduction_ratio = 14.0f, // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
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
             .climb_tilt_ratio = 0.30f,
        },
    .wheel_motor_config[0] =
        {
            .controller_param_init_config =
                {

                    .speed_PID =
                        {
                            .Kp = 5,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 6000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    .current_PID =
                        {
                            .Kp = 0,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan3,
                    .tx_id = 1,
                },
        },
    .wheel_motor_config[1] =
        {
            .controller_param_init_config =
                {

                    .speed_PID =
                        {
                            .Kp = 4.5,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 6000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    .current_PID =
                        {
                            .Kp = 0,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan3,
                    .tx_id = 4,
                },
        },
    .wheel_motor_config[2] =
        {
            .controller_param_init_config =
                {

                    .speed_PID =
                        {
                            .Kp = 5,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 6000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    .current_PID =
                        {
                            .Kp = 0,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan3,
                    .tx_id = 2,
                },
        },
    .wheel_motor_config[3] =
        {
            .controller_param_init_config =
                {

                    .speed_PID =
                        {
                            .Kp = 4.5,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 6000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                    .current_PID =
                        {
                            .Kp = 0,
                            .Ki = 0,
                            .Kd = 0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                        },
                },
            .controller_setting_init_config =
                {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
            .motor_type = M3508,
            .can_init_config =
                {
                    .can_handle = &hcan3,
                    .tx_id = 3,
                },
        },
    .lift_forward_motor_config[0] = // 前左，0是左，1是右
    {
        .controller_param_init_config =
            {

                .angle_PID =
                    {
                        .Kp = 12.0f,
                        .Ki = 0,
                        .Kd = 0,
                        .IntegralLimit = 0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 4400.0f,
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
    },                                                                                              // 前左
    .lift_forward_motor_config[1] = LIFT_FORWARD_MOTOR_CONFIG(&hcan2, 1, MOTOR_DIRECTION_REVERSE),  // 前右
    .lift_backward_motor_config[0] = LIFT_BACKWARD_MOTOR_CONFIG(&hcan2, 3, MOTOR_DIRECTION_NORMAL), // 后左
    .lift_backward_motor_config[1] =
        {
            .controller_param_init_config =
                {

                    .angle_PID =
                        {
                            .Kp = 60.0f,
                            .Ki = 0,
                            .Kd = 0.01,
                            .IntegralLimit = 0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 5200.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 3.0f,
                            .Ki = 0.0f,
                            .Kd = 0.0f,
                            .IntegralLimit = 0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 10000.0f,
                        }},
            .controller_setting_init_config =
                {
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = MOTOR_DIRECTION_NORMAL,
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
            .Kp = 100.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 2000.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 10000.0f,
        },

};

static IMU_Init_Config_s imu_init_config = {.flag = 1,
                                            .scale = {1.0f, 1.0f, 1.0f},
                                            .Yaw = 0.0f,
                                            .Pitch = 0.0f,
                                            .Roll = 0.0f,
                                            .GyroOffset[0] = 0.000841554138f,
                                            .GyroOffset[1] = -0.00300184754f,
                                            .GyroOffset[2] = 0.0022677423f,
                                            .offset_flag = 1};

static Grab_Init_Config_s
    grab_init_config =
        {
            .Grab_cali_mode = GRAB_PRE_CALI_MODE,

            .Grab_param =
                {
                    // 软件限位与灵敏度
                    .base_joint_sens_keyboard = 0.05,
                    .elbow_roll_sens_keyboard = 0.05,
                    .elbow_pitch_sens_keyboard = 0.05,
                    .wrist_roll_sens_keyboard = 0.05,
                    .wrist_pitch_sens_keyboard = 0.05,
                    .arm_lift_sens_keyboard = 1.0,
                    .arm_extend_sens_keyboard =1.0,

                    .elbow_pitch_max = 97.143158f,
                    .elbow_pitch_min = -106.869514f,
                    .base_joint_max = 125.338615f,
                    .base_joint_min = -28.0f,
                    .elbow_roll_max = 464.973602f,
                    .elbow_roll_min = -362.617554f,
                    .arm_lift_max = 320.0f,

                    // 物理传动比参数
                    .pulley_gear_ratio = 2.125f,
                    .bevel_gear_ratio = 1.6667f,
                    .planar_gear_ratio = 1.571428f,
                    .motor2006_reduction_ratio = 36.0f,
                    .motor3508_p51_reduction_ratio = 51.0f,
                    .motor3508_p19_reduction_ratio = 19.0f,

                    // 标定速度与容差参数
                    .dm_homing_tolerance = 5.0f,
                    .dm_cali_max_ticks = 5000,

                    .wrist_cali_max_ticks = 3000,
                    .wrist_cali_speed = 0.10f,
                    .wrist_cali_check_ticks = 500,
                    .wrist_cali_tolerance = 300.0f,
                    .wrist_cali_stall_current = 800.0f,

                    .extend_cali_max_ticks = 5000,
                    .extend_cali_speed = 0.2f,

                    // 硬件挂载与标定模式开关
                    .use_wrist_stall_cali = 1,
                    .use_wrist_left_motor = 1,
                    .use_wrist_right_motor = 1,
                    .wrist_soft_limit_margin = 0.90f,

                    .gripper_close_torque = 2.0f,
                    .gripper_open_torque = -0.6f,
                },

            .Grab_motor_config[0] =
                {
                    .controller_param_init_config =
                        {
                            .angle_PID =
                                {
                                    .Kp = 100.0f,
                                    .Ki = 0.00f,
                                    .Kd = 0.01f,
                                    .MaxOut = 5.0f,
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
                                          .MaxOut = 8000.0},
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
                                          .MaxOut = 8000.0},
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
                                    .MaxOut = 13000.0f,
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
            .Grab_motor_config[7] =
                {
                    .controller_param_init_config =
                        {
                            .angle_PID =
                                {
                                    .Kp = 5.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                    .MaxOut = 15000.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 4.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 10000.0f,
                                    .MaxOut = 8000.0f,
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
                    .motor_type = M3508,
                    .can_init_config =
                        {
                            .can_handle = &hcan2,
                            .tx_id = 5,
                        },
                },
            .Grab_motor_config[8] =
                {
                    .controller_param_init_config =
                        {
                            .angle_PID =
                                {
                                    .Kp = 10.0f, // 初始参数，需根据前伸机构惯量微调
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 0.0f,
                                    .MaxOut = 15000.0f,
                                },
                            .speed_PID =
                                {
                                    .Kp = 4.0f,
                                    .Ki = 0.0f,
                                    .Kd = 0.0f,
                                    .Improve = PID_Integral_Limit,
                                    .IntegralLimit = 10000.0f,
                                    .MaxOut = 16000.0f,
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
                    .motor_type = M3508,//前伸电机
                    .can_init_config =
                        {
                            .can_handle = &hcan3, // 建议挂在CAN3上，ID设为7以避开底盘和云台
                            .tx_id = 5,
                        },
                },
};

static VideoGimbal_Init_Config_s video_gimbal_init_config = {
    // GM6020 Yaw (hcan2, id=3)
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 80,
                            .Ki = 0,
                            .Kd = 1,
                            .IntegralLimit = 960,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 1500,
                        },
                    .speed_PID =
                        {
                            .Kp = 4,
                            .Ki = 40,
                            .Kd = 0,
                            .IntegralLimit = 12500,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 20000,
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
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,//云台yaw
                    .tx_id = 3,
                },
        },

    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 180.0f,
                            .Ki = 8.0f,
                            .Kd = 3.0f,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 0.0f,
                            .MaxOut = 5000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 5.0f,
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
                    .can_handle = &hcan3,//云台pitch
                    .tx_id = 6, // pitch
                },
        },

    // ⚙️  GM6020 Yaw 固定零点：用 vofa/调试器读到朝向正前方时的 total_angle，填到这里
    // 填 0.0f = 上电当前位置作零点（不管云台在哪里，那里就是中位）
    .yaw_zero_angle = 242.491074f,
};