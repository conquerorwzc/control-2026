//
// Created by PC on 2025/12/22.
//
#ifndef CONTROL_2026_ROBOT_H
#define CONTROL_2026_ROBOT_H

#include "remote_control.h"
#include "dji_motor.h"
#include "general_def.h"
#include <stdint.h>
#include <stdbool.h>

/* 机器人实例 */
typedef struct {
  DJIMotorInstance* outpost_motor; // 更名为前哨站电机
  RC_ctrl_t* rc_data;

  // 锁死逻辑专用变量
  float lock_angle;
  bool  is_locked;
  bool  is_first_loop;
} RobotInstance;

extern RobotInstance* robot;

void RobotInit(void);
void RobotTask(void);

#endif