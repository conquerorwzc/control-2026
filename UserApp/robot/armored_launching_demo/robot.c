#include "robot.h"
#include "latch_release/latch_release.h"
#include "robot_cmd.h"
#include "robot_config.h"
#include "spring_loader/spring_loader.h"

/**
 * @brief 初始化 Demo 的两个执行机构和遥控器。
 * @details 由 FreeRTOS 的机器人任务在进入循环前调用一次。先初始化上簧和击发模块，
 *          注册各自的指令订阅，再初始化遥控器接收和指令发布者。
 */
void RobotInit(void)
{
    SpringLoaderInit(); // 注册两台 3508，并订阅遥控器指令
    LatchReleaseInit(); // 注册 M2006，并订阅遥控器指令
    RobotCmdInit();     // 初始化遥控器和指令发布者
}

/**
 * @brief 每轮依次处理遥控器、上簧机构和击发机构。
 * @details 先读取摇杆并发布动作指令，再让两个执行模块在同一轮读取消息并更新电机控制。
 */
void RobotTask(void)
{
    RobotCmdTask();     // 先读摇杆，有新动作就发布
    SpringLoaderTask(); // 同一轮接收左摇杆指令，执行上簧或回程
    LatchReleaseTask(); // 同一轮接收右摇杆指令，执行击发
}