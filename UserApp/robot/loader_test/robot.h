/**
 ******************************************************************************
 * @file    robot.h
 * @brief   拨弹盘(M2006 + C610电调)测试机器人的抽象,由遥控器左右拨杆控制拨弹盘电机
 ******************************************************************************
 */
#pragma once

#include "dji_motor.h"
#include "remote_control.h"

/* 拨弹盘工作模式,由遥控器左右拨杆的档位决定 */
typedef enum
{
    LOADER_DISABLE = 0,        // 左右拨杆[下]:失能,电机不输出力矩
    LOADER_STEP_SLOW,          // 右拨杆[中]:每隔700ms正转45°
    LOADER_STEP_FAST,          // 右拨杆[上]:每隔50ms正转45°
    LOADER_STEP_SLOW_REVERSE,  // 左拨杆[中]:每隔700ms反转45°
    LOADER_STEP_FAST_REVERSE,  // 左拨杆[上]:每隔50ms反转45°
} Loader_Mode_e;

typedef struct
{
    Loader_Mode_e loader_mode;       // 拨弹盘工作模式
    RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
    DJIMotorInstance *loader_motor;  // 拨弹盘电机(M2006+C610)
    float target_angle;              // 目标角度(°),M2006转子侧的多圈累计角度(拨盘侧角度 = 该值/36)
} RobotInstance;

/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit(void);

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void RobotTask(void);
