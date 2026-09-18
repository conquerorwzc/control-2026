/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   拨弹盘(DM4310)测试机器人的配置:开发板类型、CAN总线与拨盘步进参数
 * @note    本机器人仅用于拨弹盘机构的台架测试,遥控器右拨杆直接控制拨弹盘电机
 ******************************************************************************
 */
#pragma once

#include "general_def.h"
#include "robot.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD  // 单板控制
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

/* ------------------------------- 拨弹盘电机 ------------------------------- */
/* 遥控器(DBUS)一帧的字节数,仅用于诊断打印 */
#define LOADER_RC_FRAME_SIZE 18

/* 拨弹盘电机挂载的CAN总线,按实际接线修改(hcan1或hcan2) */
#define LOADER_CAN hcan1
/* 与达妙调试助手中设置的CAN ID保持一致:tx_id用于主控发送指令,rx_id用于接收电机反馈 */
#define LOADER_MOTOR_TX_ID 0x01
#define LOADER_MOTOR_RX_ID 0x00

/* ------------------------------- 拨盘步进参数 ------------------------------- */
/* 每次步进的角度(°).这里的角度指电机输出轴的角度,DM4310直接驱动拨盘时即为拨盘角度;
   若拨盘与电机之间存在减速比,需要乘以减速比后再填入 */
#define LOADER_STEP_ANGLE_DEG 30.0f
/* 遥控器拨杆[中]/[上]时的步进间隔(ms),正转(右拨杆)与反转(左拨杆)共用同一组参数 */
#define LOADER_STEP_PERIOD_MID_MS 700
#define LOADER_STEP_PERIOD_UP_MS 40

/* 步进折算出的平均角速度(rad/s),作为速度环的前馈值:
   让速度环直接按目标转速运行,角度环只负责修正残差,
   避免纯比例位置环在快速步进时产生与转速成正比的固定滞后.
   反转时由robot.c取负值传入,前馈符号与目标角度的变化方向保持一致 */
#define LOADER_STEP_RATE_MID (LOADER_STEP_ANGLE_DEG * DEGREE_2_RAD / ((float)LOADER_STEP_PERIOD_MID_MS * 0.001f))
#define LOADER_STEP_RATE_UP (LOADER_STEP_ANGLE_DEG * DEGREE_2_RAD / ((float)LOADER_STEP_PERIOD_UP_MS * 0.001f))

/* 速度前馈值,由robot.c按当前拨杆档位写入,通过DMMotor模块的SPEED_FEEDFORWARD作用到速度环 */
static float loader_speed_feedforward = 0.0f;

/* 拨弹盘电机(DM4310)配置:角度环+速度环串级,最终输出为力矩参考,由DMMotor模块下发 */
static Motor_Init_Config_s loader_motor_config = {
    .controller_param_init_config =
        {
            .speed_feedforward_ptr = &loader_speed_feedforward,
            .angle_PID =
                {
                    .Kp = 10.0f,         // 角度环增益,误差1rad对应12rad/s的速度参考
                    .Ki = 0.0f,          // 速度前馈已提供主要的速度参考,无需积分(纯比例)
                    .Kd = 0.0f,
                    .MaxOut = 15.0f,     // 角度环输出为速度参考(rad/s),DM4310速度上限为30rad/s
                    .DeadBand = 0.005f,  // 死区约0.29°,避免静止时抖动
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                },
            .speed_PID =
                {
                    .Kp = 0.6f,
                    .Ki = 0.1f,
                    .Kd = 0.0f,
                    .MaxOut = 8.0f,  // 速度环输出为力矩参考(N·m),DM4310力矩上限为10N·m
                    .DeadBand = 0.01f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .IntegralLimit = 5.0f,
                },
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            /* 拨盘与电机正方向相反,故两个方向标志一起取反(力矩取反+反馈取反).
               注意位置环+速度环串级时二者必须同时取反:只反一个会使负反馈变成正反馈,电机会飞车.
               若实车发现方向又反了,把这两个标志一起改回MOTOR_DIRECTION_NORMAL/FEEDBACK_DIRECTION_NORMAL即可 */
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
    .motor_type = J4310,
    .can_init_config =
        {
            .can_handle = &LOADER_CAN,
            .tx_id = LOADER_MOTOR_TX_ID,
            .rx_id = LOADER_MOTOR_RX_ID,
        },
};
