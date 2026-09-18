/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   拨弹盘(M2006 + C610电调)测试机器人的配置:开发板类型、CAN总线与拨盘步进参数
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

/* 拨弹盘电机(M2006+C610)挂载的CAN总线,按实际接线修改(hcan1或hcan2) */
#define LOADER_CAN hcan1
/* M2006走C610电调,使用DJI标准CAN协议:tx_id填电调拨码开关/调参软件设置的ID(1~8),
   DJIMotor模块会据此自动算出反馈ID(0x200+ID)并把它挂到对应的分组发送帧(0x200/0x1FF),
   所以这里不要再手填rx_id,填了也会被覆盖 */
#define LOADER_MOTOR_ID 1
/* M2006减速箱的减速比:DJIMotor反馈的total_angle(°)和speed_aps(°/s)都是转子侧的,
   除以36才是拨盘侧的角度/转速.机械上如果拨盘与电机之间还有别的传动比,需要乘进来 */
#define LOADER_MOTOR_REDUCTION_RATIO 36.0f

/* ------------------------------- 拨盘步进参数 ------------------------------- */
/* 每发弹丸拨盘转过的角度(°),由机械设计图纸给出:拨盘一圈8发弹丸 => 360/8 = 45 */
#define LOADER_STEP_ANGLE_DEG 45.0f
/* 上述步距折算到电机转子侧的角度(°):转子转36圈拨盘才转1圈 */
#define LOADER_MOTOR_STEP_DEG (LOADER_STEP_ANGLE_DEG * LOADER_MOTOR_REDUCTION_RATIO)
/* 遥控器拨杆[中]/[上]时的步进间隔(ms),正转(右拨杆)与反转(左拨杆)共用同一组参数 */
#define LOADER_STEP_PERIOD_MID_MS 700
#define LOADER_STEP_PERIOD_UP_MS 50

/* 步进折算出的平均角速度(°/s,转子侧),作为速度环的前馈值:
   让速度环直接按目标转速运行,角度环只负责修正残差,
   避免纯比例位置环在快速步进时产生与转速成正比的固定滞后.
   反转时由robot.c取负值传入,前馈符号与目标角度的变化方向保持一致.
   注意单位必须与DJIMotor的speed_aps(°/s)一致,这里填的是转子侧而不是拨盘侧的速度 */
#define LOADER_STEP_RATE_MID (LOADER_MOTOR_STEP_DEG / ((float)LOADER_STEP_PERIOD_MID_MS * 0.001f))
#define LOADER_STEP_RATE_UP (LOADER_MOTOR_STEP_DEG / ((float)LOADER_STEP_PERIOD_UP_MS * 0.001f))

/* 速度前馈值(转子侧°/s),由robot.c按当前拨杆档位写入,通过SPEED_FEEDFORWARD作用到速度环 */
static float loader_speed_feedforward = 0.0f;

/* 角度环输出(速度参考)限幅,单位:转子侧°/s.
   M2006配C610时输出轴空载500rpm,折算到转子为500×36=18000rpm=108000°/s,这里先取一半留电流余量 */
#define LOADER_MAX_SPEED_DPS 40000.0f
/* 速度环输出(转矩电流控制量)限幅:C610为±10000对应±10A(若换C620则是±16384对应±20A) */
#define LOADER_MAX_CURRENT_CMD 10000.0f

/* 拨弹盘电机(M2006+C610)配置:角度环+速度环串级,最终输出为转矩电流控制量,由DJIMotor模块下发 */
static Motor_Init_Config_s loader_motor_config = {
    .controller_param_init_config =
        {
            .speed_feedforward_ptr = &loader_speed_feedforward,
            .angle_PID =
                {
                    .Kp = 40.0f,                      // TODO: 待整定,按需求先全部给0
                    .Ki = 0.0f,                      // TODO: 待整定
                    .Kd = 0.2f,                      // TODO: 待整定
                    .MaxOut = LOADER_MAX_SPEED_DPS,  // 角度环输出为速度参考(转子侧°/s)
                    .DeadBand = 0.0f,                // TODO: 待整定,过大会给步进留下固定残差
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                },
            .speed_PID =
                {
                    .Kp = 1.5f,                        // TODO: 待整定,按需求先全部给0
                    .Ki = 0.4f,                        // TODO: 待整定
                    .Kd = 0.0f,                        // TODO: 待整定
                    .MaxOut = LOADER_MAX_CURRENT_CMD,  // 速度环输出为转矩电流控制量
                    .DeadBand = 0.0f,                  // TODO: 待整定
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .IntegralLimit = 5000.0f,  // 积分限幅,约2A,防止堵转时积分饱和
                },
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            /* 拨盘与电机正方向相反时,把下面两个方向标志一起改成REVERSE(力矩取反+反馈取反).
               注意位置环+速度环串级时二者必须同时取反:只反一个会使负反馈变成正反馈,电机会飞车 */
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
    .motor_type = M2006,
    .can_init_config =
        {
            .can_handle = &LOADER_CAN,
            .tx_id = LOADER_MOTOR_ID,
        },
};
