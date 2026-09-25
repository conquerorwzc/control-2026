//
// Created by Dell on 2026/9/24.
//

#include "robot.h"
#include "can_comm.h"
#include "general_def.h"
#include "master_process.h"
#include "robot_config.h"
#include "user_lib.h"
static RobotInstance *robot;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static RC_ctrl_t *rc_data;

float Delta_Yaw(const GimbalInstance *gimbal_ins)
{
    const float big_yaw_deg = gimbal_ins->big_yaw_motor->measure.total_angle / DEGREE_2_RAD;
    const float imu_yaw_deg = gimbal_ins->gimbal_IMU_data->Yaw;
    const float offset = imu_yaw_deg - big_yaw_deg - ALIGN_DEG;
    return offset;
}
static void RemoteControlSet()
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    }
    else
    {
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
    }
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        gimbal_ctrl_cmd->big_pitch_pose=GIMBAL_BIG_PITCH_HORIZONTAL;    //水平姿态
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        gimbal_ctrl_cmd->big_pitch_pose=GIMBAL_BIG_PITCH_VERTICAL;      //垂直姿态
    }
    // 云台使能,纯遥控器拨杆控制
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_ON)
    {
        if (1)
        {
            gimbal_ctrl_cmd->yaw -= 0.0003f * (float)rc_data[TEMP].rc.rocker_r_;
        }
        gimbal_ctrl_cmd->big_yaw -= 0.0006f * (float)rc_data[TEMP].rc.rocker_r_;
        gimbal_ctrl_cmd->pitch += 0.0005f * (float)rc_data[TEMP].rc.rocker_r1;
    }
}
static void EmergencyHandler()
{
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) ||
        switch_is_off(rc_data[TEMP].rc.switch_left) || switch_is_off(rc_data[TEMP].rc.switch_right)) // 全部失能
    {
        robot->robot_mode = ROBOT_POWER_OFF;
        gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;
        for (int i = 0; i < 16; i++)
            rc_data[TEMP].key_count[KEY_PRESS][i] = 0; // 复位    注意：更改键位的时候要对这里以及下面的复位进行大改。
        LOGERROR("[CMD] emergency stop!");
    }
    else
    {
        robot->robot_mode = ROBOT_POWER_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}
void RobotCMDTask()
{
    RemoteControlSet();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
}
void RobotInit()
{
    robot = (RobotInstance *)malloc(sizeof(RobotInstance));
    robot->robot_mode = ROBOT_POWER_OFF;
    robot->rc_data = RemoteControlInit(&huart3);
    robot->gimbal = GimbalInit(&gimbal_init_config);
    gimbal_ctrl_cmd = &robot->gimbal->gimbal_ctrl_cmd;
    rc_data = robot->rc_data;
}
void RobotTask()
{
    RobotCMDTask();
    GimbalTask();
}