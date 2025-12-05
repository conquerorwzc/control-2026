#include "robot.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static DMMotorInstance* motor_instance;

void RobotInit() {
  wheel_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
  wheel_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP;
  motor_instance = DMMotorInit(&wheel_motor_config);
}

void RobotTask() {
  DMMotorSetRef(motor_instance, 400.0f);
}