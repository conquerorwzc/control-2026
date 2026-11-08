//
// 四舵轮 demo 机器人定义
// robot 层负责遥控、虚拟云台和急停；底盘组件负责电机及 HKUST 超电。
//
#pragma once

#include "chassis.h"
#include "remote_control.h"

typedef struct {
  RC_ctrl_t *rc_data;         // 遥控器未启用时为 NULL
  ChassisInstance *chassis;   // 舵轮底盘实例
} RobotInstance;

/**
 * @brief 机器人初始化, 请在开启 rtos 之前调用. 这也是唯一需要放入 main 函数的函数
 */
void RobotInit();

/**
 * @brief 机器人任务, 放入实时系统以一定频率运行, 内部会调用各个应用的任务
 */
void RobotTask();
