/**
 ******************************************************************************
 * @file    robot_cmd.c
 * @brief   双闭环轮腿机器人的遥控器命令解析与急停处理
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "robot.h"

#include "user_lib.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void RemoteControlSet(RobotInstance *robot_instance);
static void EmergencyHandler(RobotInstance *robot_instance);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 将上层遥控器输入转换为底盘命令，并优先处理急停。
 *
 * 当前只定义底盘使能命令：右拨杆上或中请求关节使能，右拨杆下请求关节失能。
 * 云台尚未初始化，故不调用 CalcOffsetAngle；键鼠到底盘运动、腿长和跳跃命令尚未定义，
 * 因而不保留没有实际行为的空 MouseKeySet 函数。
 * TODO：接入底盘运动命令、腿长目标、跳跃状态和 VMC/LQR 上层命令；上层只能写
 * chassis 命令或 leg.vmc 命令，禁止直接写电机。
 *
 * @param robot_instance 顶层机器人对象。
 */
void RobotCMDTask(RobotInstance *robot_instance)
{
    if (robot_instance == NULL || robot_instance->chassis == NULL)
    {
        return;
    }

    RemoteControlSet(robot_instance);
    EmergencyHandler(robot_instance);
}

/**
 * @brief 根据遥控器右拨杆写入底盘使能或失能命令。
 *
 * @param robot_instance 顶层机器人对象。
 */
static void RemoteControlSet(RobotInstance *robot_instance)
{
    if (robot_instance->rc_data == NULL)
    {
        return;
    }

    if (switch_is_up(robot_instance->rc_data[TEMP].rc.switch_right) ||
        switch_is_mid(robot_instance->rc_data[TEMP].rc.switch_right))
    {
        robot_instance->robot_mode = ROBOT_MODE_CONTROL;
        robot_instance->chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_ON;
    }
    else if (switch_is_down(robot_instance->rc_data[TEMP].rc.switch_right))
    {
        robot_instance->robot_mode = ROBOT_MODE_POWER_OFF;
        robot_instance->chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
    }
}

/**
 * @brief 处理遥控器离线和双拨杆下的急停请求。
 *
 * @param robot_instance 顶层机器人对象。
 */
static void EmergencyHandler(RobotInstance *robot_instance)
{
    const uint8_t remote_offline = RemoteControlIsOnline() == 0u;
    const uint8_t double_switch_down =
        robot_instance->rc_data != NULL && switch_is_down(robot_instance->rc_data[TEMP].rc.switch_right) &&
        switch_is_down(robot_instance->rc_data[TEMP].rc.switch_left);
    if (remote_offline == 0u && double_switch_down == 0u)
    {
        return;
    }

    const uint8_t was_power_off = robot_instance->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF;
    robot_instance->robot_mode = ROBOT_MODE_POWER_OFF;
    robot_instance->chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
    if (was_power_off == 0u)
    {
        LOGWARNING("[CMD] emergency stop");
    }
}
