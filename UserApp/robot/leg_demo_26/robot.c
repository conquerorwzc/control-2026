/**
 ******************************************************************************
 * @file    robot.c
 * @date    2026/7/26
 * @brief   Six-motor CAN communication test for a wheeled-legged chassis
 ******************************************************************************
 */

#include "robot.h"

#include <stdlib.h>

#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot;

RobotInstance* RobotGetInstance(void) { return robot; }

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  robot->leg[LEG_LEFT] = LegInit(&chassis_init_config.leg_init_config[LEG_LEFT]);
  robot->leg[LEG_RIGHT] = LegInit(&chassis_init_config.leg_init_config[LEG_RIGHT]);

  LegStop(robot->leg[LEG_LEFT]);
  LegStop(robot->leg[LEG_RIGHT]);
}

void RobotTask(void) {
  LegStop(robot->leg[LEG_LEFT]);
  LegStop(robot->leg[LEG_RIGHT]);
}
