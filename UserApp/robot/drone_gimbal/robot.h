#pragma once
#include "main.h"
#include "remote_control.h"
#include "dji_motor.h"
#include <stdint.h>
#include <stdbool.h>

// 机器人状态
typedef enum {
  ROBOT_STOP = 0,
  ROBOT_RUNNING,
} Robot_Mode_e;

// 机器人主结构体 (仿照 Dart 写法，直接持有电机指针)
typedef struct {
  Robot_Mode_e mode;
  RC_ctrl_t *rc; // 遥控器指针

  // --- 核心执行机构 ---
  DJIMotorInstance *yaw_motor;   // GM6020 Yaw
  DJIMotorInstance *pitch_motor; // GM6020 Pitch
  DJIMotorInstance *fric_l;      // 3508 左摩擦轮
  DJIMotorInstance *fric_r;      // 3508 右摩擦轮

  // --- 控制目标值 ---
  float target_yaw;
  float target_pitch;
  float target_fric_speed;
  bool is_first_loop;

} RobotInstance;

// 全局变量声明
extern RobotInstance *robot;

void RobotInit(void);
void RobotTask(void);