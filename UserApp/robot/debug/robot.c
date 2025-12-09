#include "robot.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "general_def.h"
#include "master_process.h"
#include "navigator.h"
#include "robot_config.h"
#include "user_lib.h"

static DJIMotorInstance* motor_instance;

navigator_recv_t* navigator_recv_data;
Vision_Receive_s* vision_recv_data;

void RobotInit() {
  wheel_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  wheel_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
  wheel_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP;
  motor_instance = DJIMotorInit(&wheel_motor_config);
}

void RobotTask() { DJIMotorSetPIDRef(motor_instance, 400.0f); }
