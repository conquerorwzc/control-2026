//
// 四舵轮 demo 机器人定义
// 阶段 A: 单个轮组(CAN1 上 ID=4 的 M3508)连通性自检
// 底盘组件使用 dev 分支的 chassis_steering 版本(2026/1/1 重写版)
//
#pragma once

#include "chassis.h"
#include "remote_control.h"

typedef struct {
  RC_ctrl_t *rc_data;         // 遥控器数据, 阶段 A 未初始化, 为 NULL
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
