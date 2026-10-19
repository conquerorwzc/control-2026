#include "robot.h"

#include "robot_config.h"

static RobotInstance robot_instance;
static RobotInstance *robot = &robot_instance;

static void RemoteControlSet(void)
{
    Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd = &robot->shoot->shoot_ctrl_cmd;

    robot->robot_mode = ROBOT_POWER_OFF;
    shoot_ctrl_cmd->shoot_mode = SHOOT_OFF;
    shoot_ctrl_cmd->friction_mode = FRICTION_OFF;
    shoot_ctrl_cmd->friction_speed = 0.0f;

    // 遥控器离线或拨杆值无效时保持关闭，不沿用上一次的开启命令。
    if (robot->rc_data == NULL || !RemoteControlIsOnline())
        return;

    uint8_t switch_left = robot->rc_data[TEMP].rc.switch_left;
    uint8_t switch_right = robot->rc_data[TEMP].rc.switch_right;
    if (switch_is_off(switch_right))
        return;

    // 与 infantry_mecanum 的摩擦轮映射一致：左中/上开启，左下关闭。
    // 右拨杆不控制摩擦轮，保留给后续底盘/云台功能。
    if (switch_is_mid(switch_left))
    {
        robot->robot_mode = ROBOT_POWER_ON;
        shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
        shoot_ctrl_cmd->friction_mode = FRICTION_ON;
        // 先以配置下限作为固定测试转速，需要其他目标时修改这里。
        shoot_ctrl_cmd->friction_speed = 10000;
    }
    if(switch_is_up(switch_left))
    {
        robot->robot_mode = ROBOT_POWER_ON;
        shoot_ctrl_cmd->shoot_mode = SHOOT_ON;
        shoot_ctrl_cmd->friction_mode = FRICTION_ON;
        // 先以配置下限作为固定测试转速，需要其他目标时修改这里。
        shoot_ctrl_cmd->friction_speed = 30000;
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