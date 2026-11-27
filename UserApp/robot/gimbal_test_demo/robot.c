#include "robot.h"

/**
 ******************************************************************************
 * @file    robot.c
 * @brief   云台 Demo
 *          - 左摇杆左右 -> 云台 yaw 目标角(速度式: 摇杆大小 = 目标角速度)
 *          - 左摇杆上下 -> 云台 pitch 目标角(速度式, 带限位)
 *          - 右拨杆[上]摇杆控制，[中]鼠标控制,[下] 或遥控器掉线 失能
 *          - E : 使能    R : 失能
 *          - 鼠标左右滑动:改变yaw角 鼠标上下滑动:改变pitch角
 *          - 松手后目标角不变 => 云台保持当前指向
 * @note    只初始化云台, 不涉及底盘/发射/视觉/双板通信
 ******************************************************************************
 */

#include <stdlib.h>

#include "bsp_dwt.h"
#include "daemon.h"
#include "robot_config.h"
#include "user_lib.h"

#include <sys/types.h>
#include "robot.h"

#include "bsp_dwt.h"
#include "ctrl.h"
#include "robot_config.h"
#include "user_lib.h"

static RobotInstance *robot;

void RobotInit(void) {
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

#ifdef STM32F407xx
    robot->rc_data = RemoteControlInit(&huart3);  // 自研板 DBUS 需接反相器串口
#elif defined STM32H723XX
    robot->rc_data = RemoteControlInit(&huart5);
#endif

    /* 云台 */
    robot->gimbal = GimbalInit(&gimbal_init_config);

    /* 发射 */
    robot->shoot = ShootInit(&shoot_init_config);

    /* 控制层初始化(缓存指令指针 + 状态默认值) */
    CtrlInit(robot);

    /* 丢弃首帧: 避免第一次任务用"开机至今"的巨大 dt */
    DWT_GetDeltaT(&robot->DWT_CNT);
    robot->dt = 0.001f;
}

void RobotTask(void) {
    /* 本周期 dt: 用 CPU 周期计数, 精度到 1 个周期 */
    robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);
    if (robot->dt <= 0.0f || robot->dt > 0.05f) {
        robot->dt = 0.001f;  // 异常保护(调度卡顿/重复调用)
    }

    CtrlSolve(robot);  // 输入解析 + 控制量 + 急停保护

    GimbalTask();
    ShootTask();
}