//
// Created by yang6 on 2026/3/3.
//

#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd=NULL;
static RobotInstance* robot;

void RobotInit() {
    robot=(RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->gimbal = GimbalInit(&gimbal_init_config);
    gimbal_ctrl_cmd=&robot->gimbal->gimbal_ctrl_cmd;
}

void RobotTask() {
    gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
     gimbal_ctrl_cmd->yaw = 1.0f;
     gimbal_ctrl_cmd->pitch = 90.0f;
     GimbalTask();

    // DMMotorSetPIDRef(J8009P_instance, speed_ref);
    // M3508_instance->motor_controller.final_output = target_torque * q2i_coeff * (16384.0f / 20.0f);
    //DJIMotorSetPIDRef(M3508_instance, speed_ref);
}
