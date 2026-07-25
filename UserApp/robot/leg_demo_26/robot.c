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

#define TEST_V_MAX 5.0f

static RobotInstance* robot;
static volatile float target_speed = 1.0f;  // rad/s

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
  target_speed = robot->rc_data[TEMP].rc.rocker_r1 / 660.0f * TEST_V_MAX;
  DMMotorSetPIDRef(robot->test_motor, target_speed);
}
