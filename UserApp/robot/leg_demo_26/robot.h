#pragma once

#include "chassis.h"

typedef enum {
  LEG_LEFT = 0,
  LEG_RIGHT,
  LEG_COUNT,
} LegSide;

typedef struct {
  LegInstance* leg[LEG_COUNT];
  INS_t* imu;

  float joint_ref[LEG_COUNT][2];
  uint8_t joint_ref_initialized[LEG_COUNT][2];
  uint8_t balance_active;

  uint32_t dwt_count;
  float dt;
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
RobotInstance* RobotGetInstance(void);
