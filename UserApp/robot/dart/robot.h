//
// Created by PC on 2025/11/18.
// 飞镖机器人: 纯遥控器遥操作, 右拨杆选档位, 左拨杆控舵机
//
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dart_shoot.h"
#include "remote_control.h"

/* 机器人档位, 由遥控器右拨杆选择 */
typedef enum {
  ROBOT_MODE_DISABLED = 0,  // 右拨杆下档(或遥控器离线): 4 个电机 + 舵机全部失能
  ROBOT_MODE_AIM,           // 右拨杆中档: 左摇杆竖直控同步带, 右摇杆水平控 yaw(均为位置累加)
  ROBOT_MODE_TRIGGER_ADJ,   // 右拨杆上档: 只允许右摇杆竖直控扳机位置电机(速度环)
} Robot_Mode_e;

/* 机器人主实例结构体 */
typedef struct {
  DartShootInstance* dart_shoot;  // 发射机构组件
  RC_ctrl_t* rc_data;             // 遥控器数据
  Robot_Mode_e robot_mode;        // 当前档位
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

/*=======工具函数=======*/
/**
 * @brief 摇杆死区检测
 *
 * @param value 摇杆值
 * @return true 在死区内
 */
bool IsInDeadzone(int16_t value);

/**
 * @brief 摇杆值线性映射为速度, 死区内输出 0
 *
 * @param stick_value 摇杆值
 * @param max_speed 摇杆推到底对应的速度
 * @return float 速度参考
 */
float MapStickToSpeed(int16_t stick_value, float max_speed);

/**
 * @brief 摇杆方向判定: 只取方向, 与摇杆具体数值无关
 *
 * @param stick_value 摇杆值
 * @param threshold 判定阈值
 * @return int8_t 1: 正向, -1: 反向, 0: 无指令
 */
int8_t StickToDirection(int16_t stick_value, int16_t threshold);
