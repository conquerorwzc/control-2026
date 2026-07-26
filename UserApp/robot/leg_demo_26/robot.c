/**
 ******************************************************************************
 * @file    robot.c
 * @date    2026/7/26
 * @brief   Fixed-leg first-order inverted-pendulum test
 ******************************************************************************
 */

#include "robot.h"

#include <math.h>
#include <stdlib.h>

#include "robot_config.h"
#include "user_lib.h"

static RobotInstance* robot;

// Runtime-tunable balance parameters. Start with the chassis supported off the ground.
static volatile uint8_t balance_enable = 1;
static volatile float balance_kp = 8.0f;
static volatile float balance_kd = 1.0f;
static volatile float balance_torque_max = 2.0f;
static volatile float balance_control_sign = 1.0f;

static float MoveTowards(float current, float target, float max_step) {
  float error = target - current;
  if (error > max_step) return current + max_step;
  if (error < -max_step) return current - max_step;
  return target;
}

static uint8_t MotorOnline(DMMotorInstance* motor) { return DaemonIsOnline(motor->daemon); }

static void StopWheelMotors(void) {
  for (size_t leg = 0; leg < LEG_COUNT; leg++) {
    DMMotorInstance* wheel = robot->leg[leg]->dm_wheel_motor;
    DMMotorSetRef(wheel, 0.0f);
    DMMotorStop(wheel);
  }
}

static uint8_t UpdateJointLocks(void) {
  uint8_t all_locked = 1;

  for (size_t leg = 0; leg < LEG_COUNT; leg++) {
    for (size_t joint = 0; joint < 2; joint++) {
      DMMotorInstance* motor = robot->leg[leg]->joint_motor[joint];

      if (!MotorOnline(motor)) {
        robot->joint_ref_initialized[leg][joint] = 0;
        DMMotorSetRef(motor, 0.0f);
        DMMotorStop(motor);
        all_locked = 0;
        continue;
      }

      if (!robot->joint_ref_initialized[leg][joint]) {
        robot->joint_ref[leg][joint] = motor->measure.position;
        robot->joint_ref_initialized[leg][joint] = 1;
      }

      robot->joint_ref[leg][joint] =
          MoveTowards(robot->joint_ref[leg][joint], joint_target[leg][joint], JOINT_REF_RATE * robot->dt);

      DMMotorEnable(motor);
      DMMotorSetPIDRef(motor, robot->joint_ref[leg][joint]);

      if (fabsf(motor->measure.position - joint_target[leg][joint]) > JOINT_LOCK_TOLERANCE) all_locked = 0;
    }
  }

  return all_locked;
}

static void UpdateBalance(uint8_t joints_locked) {
  const float pitch = -robot->imu->Pitch * DEGREE_2_RAD;
  const float pitch_rate = -robot->imu->Gyro[0];
  uint8_t wheels_online = 1;

  for (size_t leg = 0; leg < LEG_COUNT; leg++) {
    if (!MotorOnline(robot->leg[leg]->dm_wheel_motor)) wheels_online = 0;
  }

  if (!balance_enable || !joints_locked || !wheels_online || fabsf(pitch) > BALANCE_STOP_ANGLE) {
    robot->balance_active = 0;
  } else if (!robot->balance_active && fabsf(pitch) < BALANCE_START_ANGLE) {
    robot->balance_active = 1;
  }

  if (!robot->balance_active) {
    StopWheelMotors();
    return;
  }

  float torque = balance_control_sign * (balance_kp * pitch + balance_kd * pitch_rate);
  LIMIT_MIN_MAX(torque, -balance_torque_max, balance_torque_max);

  // The wheel motors are mirrored: equal chassis-forward torque requires opposite shaft torque.
  DMMotorEnable(robot->leg[LEG_LEFT]->dm_wheel_motor);
  DMMotorEnable(robot->leg[LEG_RIGHT]->dm_wheel_motor);
  DMMotorSetRef(robot->leg[LEG_LEFT]->dm_wheel_motor, torque);
  DMMotorSetRef(robot->leg[LEG_RIGHT]->dm_wheel_motor, -torque);
}

RobotInstance* RobotGetInstance(void) { return robot; }

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  robot->leg[LEG_LEFT] = LegInit(&chassis_init_config.leg_init_config[LEG_LEFT]);
  robot->leg[LEG_RIGHT] = LegInit(&chassis_init_config.leg_init_config[LEG_RIGHT]);
  robot->imu = INS_Init(&chassis_init_config.imu_init_config);

  LegStop(robot->leg[LEG_LEFT]);
  LegStop(robot->leg[LEG_RIGHT]);
  DWT_GetDeltaT(&robot->dwt_count);
}

void RobotTask(void) {
  robot->dt = DWT_GetDeltaT(&robot->dwt_count);
  if (robot->dt <= 0.0f || robot->dt > 0.02f) robot->dt = 0.001f;

  UpdateBalance(UpdateJointLocks());
}
