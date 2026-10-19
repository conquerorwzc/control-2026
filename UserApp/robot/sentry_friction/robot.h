#pragma once

#include "shoot.h"
#include "remote_control.h"

typedef enum {
    ROBOT_POWER_OFF = 0,
    ROBOT_POWER_ON,
} Robot_Mode_e;

typedef struct {
    Robot_Mode_e robot_mode;
    RC_ctrl_t *rc_data;
    ShootInstance *shoot;
} RobotInstance;

/** @brief 机器人任务启动时调用一次，初始化发射组件和控制来源。 */
void RobotInit(void);

/** @brief 更新机器人模式和发射命令，在 ShootTask() 之前调用。 */
void RobotCMDTask(void);

/** @brief 机器人周期任务，由 os_task.c 调用。 */
void RobotTask(void);

/** @brief 获取机器人实例。 */
RobotInstance *RobotGet(void);
