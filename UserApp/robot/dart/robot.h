//
// Created by PC on 2025/11/18.
// 飞镖机器人: 纯遥控器遥操作
//   右拨杆(switch_right): 下档 = 4 个电机 + 舵机全部失能
//                         中档 = 左摇杆竖直控同步带 + 右摇杆水平控 yaw
//                         上档 = 左摇杆水平控 yaw + 右摇杆竖直控扳机位置电机(同步带失能)
//   左拨杆(switch_left):  下档 = 舵机失能; 中档 = 转到角度1; 上档 = 转到角度2
//
#pragma once

#include <stdint.h>

#include "dart_shoot.h"
#include "remote_control.h"

/* 机器人主实例结构体 */
typedef struct {
  DartShootInstance* dart_shoot;  // 发射机构组件
  RC_ctrl_t* rc_data;             // 遥控器数据(RemoteControlInit 的返回值)
  DartShoot_Mode_e dart_mode;     // 档位, 由遥控器右拨杆选择(遥控器离线时为 DISABLED)
  uint8_t rc_online;              // 遥控器是否在线
} RobotInstance;

extern RobotInstance* robot;  // 全局变量声明

/*=======核心机器人函数=======*/
/**
 * @brief 机器人系统初始化
 */
void RobotInit(void);

/**
 * @brief 机器人主任务循环
 */
void RobotTask(void);
