#pragma once

#include "gimbal.h"
#include "shoot.h"
#include "remote_control.h"

typedef enum { ROBOT_POWER_OFF = 0, ROBOT_POWER_ON } Robot_Mode_e;

typedef struct {
    Robot_Mode_e robot_mode;
    RC_ctrl_t *rc_data;
    GimbalInstance *gimbal;
    ShootInstance *shoot;
} RobotInstance;

void RobotInit(void);
void RobotCMDTask(void);
void RobotTask(void);
RobotInstance *RobotGet(void);
