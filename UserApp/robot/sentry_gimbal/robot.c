#include "robot.h"

#include "robot_config.h"

static RobotInstance robot_instance;
static RobotInstance *robot = &robot_instance;

static void RemoteControlSet(void)
{
    static uint32_t load_period_ms = LOADER_STEP_PERIOD_MID_MS;
    // Gimbal_Ctrl_Cmd_s *gimbal_cmd = &robot->gimbal->gimbal_ctrl_cmd;
    Shoot_Ctrl_Cmd_s *shoot_cmd = &robot->shoot->shoot_ctrl_cmd;
    robot->robot_mode = ROBOT_POWER_OFF;
    // gimbal_cmd->gimbal_mode = GIMBAL_POWER_OFF;
    *shoot_cmd = (Shoot_Ctrl_Cmd_s){0};
    if (robot->rc_data == NULL || !RemoteControlIsOnline())
    {
        load_period_ms = LOADER_STEP_PERIOD_MID_MS;
        return;
    }

    uint8_t left = robot->rc_data[TEMP].rc.switch_left;
    uint8_t right = robot->rc_data[TEMP].rc.switch_right;
    if (switch_is_off(left) || switch_is_off(right))
    {
        load_period_ms = LOADER_STEP_PERIOD_MID_MS;
        return;
    }

    // 左中只开摩擦轮，左上拨弹；拨轮负向选 700 ms 位置步进，正向选 20 Hz 速度连发。
    if (switch_is_mid(left) || switch_is_up(left))
    {
        shoot_cmd->shoot_mode = SHOOT_ON;
        shoot_cmd->friction_mode = FRICTION_ON;
        shoot_cmd->friction_speed = FRICTION_SPEED_MID_APS;
        robot->robot_mode = ROBOT_POWER_ON;
    }

    if (switch_is_up(left))
    {
        if (robot->rc_data[TEMP].rc.dial < -100)
            load_period_ms = LOADER_STEP_PERIOD_MID_MS;
        else if (robot->rc_data[TEMP].rc.dial > 100)
            load_period_ms = LOADER_STEP_PERIOD_UP_MS;
        shoot_cmd->load_mode = load_period_ms == LOADER_STEP_PERIOD_MID_MS ? LOAD_1_BULLET : LOAD_BURSTFIRE;
        shoot_cmd->load_period_ms = load_period_ms;
    }
    else
        load_period_ms = LOADER_STEP_PERIOD_MID_MS;

    /* 拨弹盘调试期间暂不控制云台。
    // 右中水平、右上垂直；两档均使能云台，右下关闭云台。
    if (!(switch_is_mid(right) || switch_is_up(right)))
        return;
    robot->robot_mode = ROBOT_POWER_ON;
    gimbal_cmd->gimbal_mode = GIMBAL_ON;
    gimbal_cmd->big_pitch_pose = switch_is_mid(right) ? GIMBAL_BIG_PITCH_HORIZONTAL : GIMBAL_BIG_PITCH_VERTICAL;

    // 同步新版四轴：小 Yaw 改为 J4310，IMU 目标仍为 deg；暂不启用大 Yaw 跟随。
    gimbal_cmd->yaw -= 0.0002f * (float)robot->rc_data[TEMP].rc.rocker_r_;
    gimbal_cmd->pitch += 0.0005f * (float)robot->rc_data[TEMP].rc.rocker_r1;
    */
}

void RobotInit(void)
{
    robot->robot_mode = ROBOT_POWER_OFF;
    robot->rc_data = RemoteControlInit(&huart3);
    // robot->gimbal = GimbalInit(&gimbal_init_config);
    robot->shoot = ShootInit(&shoot_init_config);
}

void RobotCMDTask(void)
{
    RemoteControlSet();
}

void RobotTask(void)
{
    RobotCMDTask();
    // GimbalTask();
    ShootTask();
}

RobotInstance *RobotGet(void)
{
    return robot;
}
