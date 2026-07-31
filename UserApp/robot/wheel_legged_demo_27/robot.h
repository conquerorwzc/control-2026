/**
 ******************************************************************************
 * @file    robot.h
 * @brief   双闭环轮腿机器人板级对象定义
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"

/* 机器人当前的基础工作状态，控制器接入后可在此扩展状态机。 */
typedef enum
{
    ROBOT_MODE_POWER_OFF = 0, /* 底盘请求零力矩，不允许输出驱动力矩。 */
    ROBOT_MODE_ZERO_TORQUE,   /* 预留：达妙在线，但始终下发零力矩。 */
    ROBOT_MODE_CONTROL,       /* 后续接入整车控制器后的受控状态。 */
} RobotMode_e;


/* 顶层机器人对象；所有板级状态必须从 robot 指针向下归属。 */
typedef struct
{
    RobotMode_e robot_mode;                     /* 机器人基础工作状态。 */
    WheelLeggedChassisInstance_t *chassis;      /* 轮腿底盘对象。 */
    GimbalInstance *gimbal;                     /* 云台对象；当前未接入本机器人。 */
    RC_ctrl_t *rc_data;                         /* 遥控器运行数据；由 RobotInit 初始化。 */
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
void RobotCMDTask(RobotInstance *robot_instance);
