/**
 * @file robot.h
 * @brief 重装底盘提供给框架的初始化与周期运行接口。
 */

//
// Created by CX330y on 2026/11/23.
//

#ifndef CONTROL_2026_ROBOT_H
#define CONTROL_2026_ROBOT_H

/**
 * @brief 初始化重装底盘的电机与遥控接收。
 * @note 由框架在周期调用 RobotTask() 之前执行一次。
 */
void RobotInit(void);

/**
 * @brief 处理遥控使能状态并更新麦轮和抬升控制。
 * @note 由框架在 RobotInit() 完成后周期调用。
 */
void RobotTask(void);
#endif // CONTROL_2026_ROBOT_H
