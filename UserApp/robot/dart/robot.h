/**
 * @file robot.h
 * @brief dart 发射架机器人入口: 实例定义与任务接口
 */
#pragma once

#include "dart_launcher.h"
#include "remote_control.h"

/* 机器人模式 */
typedef enum {
  ROBOT_POWER_ON = 0,      // 使能运行
  ROBOT_EMERGENCY_STOP,    // 失能/急停(右开关下档或遥控器离线)
} Robot_Mode_e;

/* 机器人主实例 */
typedef struct {
  DartLauncherInstance* launcher;  // 发射架执行层
  RC_ctrl_t* rc_data;              // DR16 遥控器
  Robot_Mode_e robot_mode;
} RobotInstance;

/* 机器人系统初始化(由 os_task 调用) */
void RobotInit(void);

/* 机器人主任务, ~1kHz(由 os_task 调用) */
void RobotTask(void);
