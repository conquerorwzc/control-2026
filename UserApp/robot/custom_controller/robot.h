#pragma once
#include <stdbool.h>

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



/**
 * @brief 获取各电机当前角度
 * @param angles 角度数组指针(长度为4)
 */
// void GetMotorAngles(float* angles);  // 未使用，如有需要可取消注释

