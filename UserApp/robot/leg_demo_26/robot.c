/**
 ******************************************************************************
 * @file    robot.c
 * @date    2026/7/24
 * @brief   Remote-control torque test for a DAMIAO H6215 motor
 ******************************************************************************
 */

#include "robot.h"

#include <stdlib.h>

#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot;
static float target_T = 0.04f;  // N*m

RobotInstance* RobotGetInstance(void) { return robot; }

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  robot->rc_data = RemoteControlInit(&huart5);
  robot->test_motor = DMMotorInit(&test_motor_config);
  DMMotorStop(robot->test_motor);
}

void RobotTask(void) {
  const uint8_t motor_armed = RemoteControlIsOnline() && switch_is_mid(robot->rc_data[TEMP].rc.switch_right);

  if (!motor_armed) {
    DMMotorSetRef(robot->test_motor, 0.0f);
    DMMotorStop(robot->test_motor);
    return;
  }

  DMMotorEnable(robot->test_motor);
  DMMotorSetRef(robot->test_motor, target_T);
}
