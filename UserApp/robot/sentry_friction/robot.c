#include "robot.h"

#include "robot_config.h"

static RobotInstance robot_instance;
static RobotInstance *robot = &robot_instance;

static void RemoteControlSet(void)
{
    Shoot_Ctrl_Cmd_s *cmd = &robot->shoot->shoot_ctrl_cmd;
    robot->robot_mode = ROBOT_POWER_OFF;
    *cmd = (Shoot_Ctrl_Cmd_s){0}; // 每周期从关闭状态生成命令，离线不保留旧命令。
    if (robot->rc_data == NULL || !RemoteControlIsOnline())
        return;

    uint8_t left = robot->rc_data[TEMP].rc.switch_left;
    uint8_t right = robot->rc_data[TEMP].rc.switch_right;
    if (switch_is_off(right) || !(switch_is_mid(left) || switch_is_up(left)))
        return;

    robot->robot_mode = ROBOT_POWER_ON;
    cmd->shoot_mode = SHOOT_ON;
    cmd->friction_mode = FRICTION_ON;
    cmd->friction_speed = switch_is_mid(left) ? FRICTION_SPEED_MID_APS : FRICTION_SPEED_UP_APS;
    // 左中/上才接受右拨杆；右下停止，右中/上分别慢速/快速连发。
    if (switch_is_mid(right) || switch_is_up(right))
    {
        cmd->load_mode = LOAD_BURSTFIRE;
        cmd->load_period_ms = switch_is_mid(right) ? LOADER_STEP_PERIOD_MID_MS : LOADER_STEP_PERIOD_UP_MS;
    }
}

void RobotInit(void)
{
    robot->robot_mode = ROBOT_POWER_OFF;

    // STM32F4 的 DBUS 接收串口，与 infantry_mecanum 一致。
    robot->rc_data = RemoteControlInit(&huart3);

    robot->shoot = ShootInit(&shoot_init_config);
}

void RobotCMDTask(void)
{
    if (robot->shoot == NULL)
        return;

    RemoteControlSet();
}

void RobotTask(void)
{
    RobotCMDTask();

    if (robot->shoot != NULL)
        ShootTask();
}

RobotInstance *RobotGet(void)
{
    return robot;
}