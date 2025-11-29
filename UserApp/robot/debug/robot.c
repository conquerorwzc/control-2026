#include "robot.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static DMMotorInstance* J8009P_instance;
static DJIMotorInstance* M3508_instance;
float speed_ref = 0.0f;

void RobotInit() {
  J8009P_instance = DMMotorInit(&J8009P_config);
  M3508_instance = DJIMotorInit(&M3508_config);
}

void RobotTask() {
  // DMMotorSetPIDRef(J8009P_instance, speed_ref);
  DJIMotorSetPIDRef(M3508_instance, speed_ref);
}
