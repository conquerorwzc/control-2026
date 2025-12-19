#include "robot.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

float target_torque = 0;
const float reduction_ratio = 268.0f / 17.0f;
float q2i_coeff = (3591.0f / 187.0f) / reduction_ratio / 0.3f;

// static DMMotorInstance* J8009P_instance;
static DJIMotorInstance* M3508_instance;
float angle_ref = 200.0f;

void RobotInit() {
  // J8009P_instance = DMMotorInit(&J8009P_config);
  M3508_instance = DJIMotorInit(&M3508_config);
}

void RobotTask() {
  // DMMotorSetPIDRef(J8009P_instance, speed_ref);
  // M3508_instance->motor_controller.final_output = target_torque * q2i_coeff * (16384.0f / 20.0f);
  DJIMotorSetPIDRef(M3508_instance, angle_ref);
}
