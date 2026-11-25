#pragma once

#include "robot.h"

bool VofaInit(void);
void VofaTick(const ChassisRequest *command, uint64_t now_us, bool online, unsigned configured_joints, SysIdTrial trial,
              SysIdHipPattern hip_pattern, int direction);
