#include "robot.h"

#include "robot_config.h"

static RobotInstance robot_instance;
static RobotInstance *robot = &robot_instance;

static void RemoteControlSet(void)
{
    Gimbal_Ctrl_Cmd_s *gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
    Shoot_Ctrl_Cmd_s *shoot_cmd = &robot->shoot->shoot_ctrl_cmd;
    robot->robot_mode = ROBOT_POWER_OFF;
    gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    *shoot_cmd = (Shoot_Ctrl_Cmd_s){0};
    if (robot->rc_data == NULL || !RemoteControlIsOnline())
        return;

    uint8_t left = robot->rc_data[TEMP].rc.switch_left;
    uint8_t right = robot->rc_data[TEMP].rc.switch_right;
    if (switch_is_off(left) || switch_is_off(right))
        return;

    // 左中/上保留云台姿态切换，同时控制摩擦轮的低/高两档。
    if (switch_is_mid(left) || switch_is_up(left))
    {
        gimbal_cmd->big_pitch_pose = switch_is_mid(left) ? GIMBAL_BIG_PITCH_HORIZONTAL : GIMBAL_BIG_PITCH_VERTICAL;
        shoot_cmd->shoot_mode = SHOOT_ON;
        shoot_cmd->friction_mode = FRICTION_ON;
        shoot_cmd->friction_speed = switch_is_mid(left) ? FRICTION_SPEED_MID_APS : FRICTION_SPEED_UP_APS;
        robot->robot_mode = ROBOT_POWER_ON;
    }

    // 右下关闭云台和拨弹，右中/上使能云台；左中/上时允许慢/快速拨弹。
    if (!(switch_is_mid(right) || switch_is_up(right)))
        return;
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_cmd->gimbal_mode = GIMBAL_ON;
    if (shoot_cmd->friction_mode == FRICTION_ON)
    {
        shoot_cmd->load_mode = LOAD_BURSTFIRE;
        shoot_cmd->load_period_ms = switch_is_mid(right) ? LOADER_STEP_PERIOD_MID_MS : LOADER_STEP_PERIOD_UP_MS;
    }

    // 沿用 demo 的小 Yaw 相对偏角和大 Yaw 跟随控制，目标角度单位为 deg。
    float delta = robot->gimbal->small_yaw_motor->measure.total_angle - ALIGN_DEG;
    if (delta > -15.0f && delta < 15.0f)
        gimbal_cmd->yaw -= 0.001f * (float)robot->rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd->big_yaw -= PIDCalculate(&robot->gimbal->yaw_follow, delta, 0.0f);
    gimbal_cmd->big_yaw -= 0.0008f * (float)robot->rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd->pitch += 0.0005f * (float)robot->rc_data[TEMP].rc.rocker_r1;
}

void RobotInit(void)
{
    robot->robot_mode = ROBOT_POWER_OFF;
    robot->rc_data = RemoteControlInit(&huart3);
    robot->gimbal = GimbalInit(&gimbal_init_config);
    robot->shoot = ShootInit(&shoot_init_config);
}

void RobotCMDTask(void)
{
    RemoteControlSet();
}

void RobotTask(void)
{
    RobotCMDTask();
    GimbalTask();
    ShootTask();
}

RobotInstance *RobotGet(void)
{
    return robot;
}
