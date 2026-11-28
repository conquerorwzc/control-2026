#pragma once

/**
 * @file robot.h
 * @brief target_car: 老遥控器(DT7)控制一个 6020(GM6020, CAN1/ID4) 以恒定角速度匀速旋转
 */

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
