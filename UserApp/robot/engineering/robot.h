#pragma once

#include "chassis.h"
#include "grab.h"
#include "gimbal_video.h"
#include "remote_control.h"
#include "new_RC_VT13.h"
#include "selfcontrol.h"
#include "rm_referee.h"
#include "ins_task.h"
#include "super_cap.h"


// 添加机械臂控制模式枚举
typedef enum {
    GRAB_CONTROL_KEYBOARD = 0,    // 键鼠控制模式
    GRAB_CONTROL_CUSTOM,
    GRAB_CONTROL_HALF_AUTO// 自定义控制器角度控制模式
} GrabControlMode_e;

typedef struct
{
    Robot_Mode_e robot_mode; // 机器人整体工作状态
    INS_t *ins_data;
    VT13_RC_t *rc_data; // 遥控器数据,初始化时返回
    referee_info_t* referee_data;     // 用于获取裁判系统的数据
    SuperCapInstance *super_cap;
    ChassisInstance *chassis;
    GrabInstance *grab;
    VideoGimbalInstance *video_gimbal; // 图传云台独立组件
    SelfC *self_control; // 自定义控制器实例
    
    // UI 重置标志位（由 H 键触发）
    uint8_t ui_reset_flag;  // 1:需要重置 UI, 0:正常
} RobotInstance;

/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit();

/**
 * @brief 机器人任务，放入实时系统以一定频率运行，内部会调用各个应用的任务
 *
 */
void RobotTask();

/**
 * @brief 获取机器人实例
 * @return RobotInstance* 机器人实例指针
 */
RobotInstance* RobotGet(void);

/**
 * @brief 获取机械臂控制模式
 * @return GrabControlMode_e 机械臂控制模式
 */
GrabControlMode_e GetGrabControlMode(void);