#pragma once
#include "main.h"
#include "remote_control.h"
#include "dji_motor.h"
#include <stdint.h>
#include <stdbool.h>
#include "ins_task.h"
#include "gimbal.h"
#include "shoot.h"

typedef struct {
  RC_ctrl_t *rc;

  // 组件实例指针
  GimbalInstance *gimbal;
  ShootInstance *shoot;

  // 辅助变量
  bool is_first_loop;

  float yaw_imu_feed;
  float pitch_imu_feed;

} RobotInstance;

extern RobotInstance *robot;

void RobotInit(void);
void RobotTask(void);