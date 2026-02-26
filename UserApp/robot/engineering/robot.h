#pragma once

#include "chassis.h"
#include "gantry.h"
#include "grab.h"
#include "remote_control.h"

#include "selfcontrol.h"
// #include "rm_referee.h"
#include "ins_task.h"
#include "super_cap.h"
// todo: add vision_module

typedef enum
{
    ROBOT_POWER_OFF = 0,
    ROBOT_POWER_ON,
    ROBOT_EMERGENCY_STOP
} Robot_Mode_e;


// 添加机械臂控制模式枚举
typedef enum {
    GRAB_CONTROL_KEYBOARD = 0,    // 键鼠控制模式
    GRAB_CONTROL_CUSTOM          // 自定义控制器角度控制模式
} GrabControlMode_e;

typedef struct
{
    Robot_Mode_e robot_mode; // 机器人整体工作状态
    INS_t *ins_data;
    RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
    // referee_info_t* referee_data;     // 用于获取裁判系统的数据
    SuperCapInstance *super_cap;
    ChassisInstance *chassis;
    GantryInstance *gantry;
    GrabInstance *grab;
    SelfC *self_control; // 自定义控制器实例
} RobotInstance;

/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void RobotTask();