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
 * @brief 注册重装底盘电机并初始化 DBUS 遥控接收，电机控制输出初始为停止。
 * @note 由框架在周期调用 RobotTask() 之前执行一次。
 */
void RobotInit(void);

/**
 * @brief 按遥控使能状态更新麦轮和四路抬升控制。
 * @details 首次允许抬升输出且四路反馈有效时，先完成机械回零，再响应右摇杆。
 * @note 由框架在 RobotInit() 完成后周期调用。
 */
void RobotTask(void);
#endif // CONTROL_2026_ROBOT_H
