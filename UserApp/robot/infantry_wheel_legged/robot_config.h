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

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350               // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300              // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define WHEEL_RADIUS 60              // 轮子半径
#define WHEEL_REDUCTION_RATIO 19.0f  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

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

const static Motor_Init_Config_s wheel_motor_config = {
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 2.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 12000.0f,
                },
            .current_PID =
                {
                    .Kp = 0.5f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 15000.0f,
                },
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan1,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
};

static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            .wheel_base = WHEEL_BASE,
            .track_width = TRACK_WIDTH,
            .center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X,
            .center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y,
            .wheel_radius = WHEEL_RADIUS,
            .wheel_reduction_ratio = WHEEL_REDUCTION_RATIO,
        },
    .wheel_motor_config[0] = wheel_motor_config,
    .wheel_motor_config[0].can_init_config.tx_id = 1,
    .wheel_motor_config[1] = wheel_motor_config,
    .wheel_motor_config[1].can_init_config.tx_id = 4,
    .wheel_motor_config[2] = wheel_motor_config,
    .wheel_motor_config[2].can_init_config.tx_id = 2,
    .wheel_motor_config[3] = wheel_motor_config,
    .wheel_motor_config[3].can_init_config.tx_id = 3,
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
                    .can_handle = &hcan1,
                    .tx_id = 2,
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
                    .tx_id = 1,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
};

const static Motor_Init_Config_s friction_motor_config = {
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 20.0f,
                    .Ki = 1.0f,
                    .Kd = 0.0f,
                    .Improve = PID_Integral_Limit,
                    .IntegralLimit = 10000.0f,
                    .MaxOut = 15000.0f,
                },
            .current_PID =
                {
                    .Kp = 0.7f,
                    .Ki = 0.1f,
                    .Kd = 0.0f,
                    .Improve = PID_Integral_Limit,
                    .IntegralLimit = 10000.0f,
                    .MaxOut = 15000.0f,
                },
        },
    .motor_type = M3508,
    .can_init_config.can_handle = &hcan2,
};

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = ONE_BULLET_DELTA_ANGLE,
            .reduction_ratio_loader = REDUCTION_RATIO_LOADER,
            .num_per_circle = NUM_PER_CIRCLE,
        },
    .friction_motor_config[0] = friction_motor_config,
    .friction_motor_config[0].can_init_config.tx_id = 2,
    .friction_motor_config[0].controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    .friction_motor_config[1] = friction_motor_config,
    .friction_motor_config[1].can_init_config.tx_id = 1,
    .friction_motor_config[1].controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,

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
                    .can_handle = &hcan2,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
};

static PID_Init_Config_s chassis_follow_PID_config = {
    .Kp = 50.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .IntegralLimit = 1000.0f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut = 8000.0f,
};

static SuperCap_Init_Config_s super_cap_config = {
    .can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x302,  // 超级电容默认接收id
        .rx_id = 0x301,  // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    }};