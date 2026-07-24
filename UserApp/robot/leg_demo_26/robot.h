#pragma once

#include "dmmotor.h"
#include "remote_control.h"

typedef struct {
  RC_ctrl_t* rc_data;
  DMMotorInstance* test_motor;
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
RobotInstance* RobotGetInstance(void);
