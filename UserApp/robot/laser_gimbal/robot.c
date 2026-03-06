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
static  float yaw_max_angle=-40.0f;
static  float yaw_min_angle=-60.0f;
static  float T=2000;
static  float step;


void RobotInit() {
    robot=(RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->gimbal = GimbalInit(&gimbal_init_config);
    gimbal_ctrl_cmd=&robot->gimbal->gimbal_ctrl_cmd;
    robot->rc_data = RemoteControlInit(&huart3);
    step = (yaw_max_angle-yaw_min_angle)/T;

}

void RobotTask() {
    if (switch_is_down(robot->rc_data[TEMP].rc.switch_right))
    {
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_POWER_OFF;
        gimbal_ctrl_cmd->yaw = -50.0f;
        gimbal_ctrl_cmd->pitch = 90.0f;
    }
    else if(switch_is_mid(robot->rc_data[TEMP].rc.switch_right))
    {
        gimbal_ctrl_cmd->gimbal_mode=GIMBAL_ON;
        gimbal_ctrl_cmd->yaw += -0.0005f * (float)robot->rc_data[TEMP].rc.rocker_r_;
        gimbal_ctrl_cmd->pitch -= 0.0003f * (float)robot->rc_data[TEMP].rc.rocker_r1;
    }
    else if (switch_is_up(robot->rc_data[TEMP].rc.switch_right))
    {
        if (gimbal_ctrl_cmd->yaw>=yaw_max_angle)
        {
            step=(yaw_min_angle-yaw_max_angle)/T;
        }
        else if (gimbal_ctrl_cmd->yaw<=yaw_min_angle)
        {
            step=(yaw_max_angle-yaw_min_angle)/T;
        }
        gimbal_ctrl_cmd->yaw += step;
        gimbal_ctrl_cmd->pitch -= 0.0003f * (float)robot->rc_data[TEMP].rc.rocker_r1;
    }

     GimbalTask();

    // DMMotorSetPIDRef(J8009P_instance, speed_ref);
    // M3508_instance->motor_controller.final_output = target_torque * q2i_coeff * (16384.0f / 20.0f);
    //DJIMotorSetPIDRef(M3508_instance, speed_ref);
}
