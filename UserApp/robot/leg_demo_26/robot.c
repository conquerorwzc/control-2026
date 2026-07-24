/**
 ******************************************************************************
 * @file    robot.c
 * @date    2026/7/24
 * @brief   Remote-control test for a DAMIAO J4310 motor
 ******************************************************************************
 */

#include "robot.h"

#include <stdlib.h>

#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot;

RobotInstance* RobotGetInstance(void) { return robot; }

static float GetTargetSpeed(void) {
  const int16_t rocker = robot->rc_data[TEMP].rc.rocker_r1;
  if (abs(rocker) <= RC_ROCKER_DEADBAND) return 0.0f;

  return (float)rocker / RC_ROCKER_MAX * TEST_MOTOR_MAX_SPEED;
}

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
  DMMotorSetPIDRef(robot->test_motor, GetTargetSpeed());
}
