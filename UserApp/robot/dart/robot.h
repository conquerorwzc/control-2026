/**
 * @file robot.h
 * @brief dart 发射架机器人入口: 实例定义与任务接口
 *
 * 执行层按机构拆分(见 UserApp/components):
 *   - shoot  (shoot/shoot_dart):  同步带 x2 + 扳机丝杆 + 扳机舵机的状态机;
 *   - gimbal (gimbal/gimbal_dart): 发射架 yaw。
 */
#pragma once

#include "gimbal.h"
#include "remote_control.h"
#include "shoot.h"

/* 机器人模式 */
typedef enum {
  ROBOT_POWER_ON = 0,      // 使能运行
  ROBOT_EMERGENCY_STOP,    // 失能/急停(右开关下档或遥控器离线)
} Robot_Mode_e;

/* 机器人主实例 */
typedef struct {
  ShootDartInstance* shoot;    // 发射机构: 同步带/扳机丝杆/扳机舵机
  GimbalDartInstance* gimbal;  // 云台: 发射架 yaw
  RC_ctrl_t* rc_data;          // DR16 遥控器
  Robot_Mode_e robot_mode;
} RobotInstance;

/* 机器人系统初始化(由 os_task 调用) */
void RobotInit(void);

/* 机器人主任务, ~1kHz(由 os_task 调用) */
void RobotTask(void);
