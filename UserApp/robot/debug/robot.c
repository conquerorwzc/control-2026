#include "robot.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"

static DMMotorInstance* motor_instance;
float speed_ref = 0.0f;

void RobotInit() {
  wheel_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  wheel_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
  motor_instance = DMMotorInit(&wheel_motor_config);
}

void RobotTask() { DMMotorSetPIDRef(motor_instance, speed_ref); }
