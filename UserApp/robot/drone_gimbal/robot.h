#pragma once

#include "main.h"
#include "remote_control.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include <stdint.h>
#include <stdbool.h>
#include "ins_task.h"

// 1. 引入需要的模块
#include "gimbal.h"
#include "shoot.h"

// 2. 机器人状态枚举
typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

// 3. 机器人总实例结构体
typedef struct {
  RC_ctrl_t *rc; // 遥控器指针

  // --- 组件实例 ---
  GimbalInstance *gimbal; // 云台
  ShootInstance *shoot;   // 发射机构
  DMMotorInstance *pitch_dm_motor;

  // --- 辅助变量 ---
  bool is_first_loop;
  bool safety_lock;

  // --- 串级 PID 反馈接口 ---
  // 这两个变量用于把处理后的 IMU 数据“喂”给电机控制器的外环
  float yaw_imu_feed;
  float pitch_imu_feed;

} RobotInstance;

extern RobotInstance *robot;

void RobotInit(void);
void RobotTask(void);