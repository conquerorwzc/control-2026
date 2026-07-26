#pragma once

#include "chassis.h"

typedef enum {
  LEG_LEFT = 0,
  LEG_RIGHT,
  LEG_COUNT,
} LegSide;

typedef struct {
  LegInstance* leg[LEG_COUNT];
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
RobotInstance* RobotGetInstance(void);
