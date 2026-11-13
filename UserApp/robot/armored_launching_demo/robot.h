//
// Created by CX330y on 2026/11/11.
//

#ifndef CONTROL_2026_ROBOT_H
#define CONTROL_2026_ROBOT_H

/**
 * @brief 在任务进入循环前初始化上簧模块、击发模块和遥控器。
 */
void RobotInit(void);

/**
 * @brief 周期执行任务，依次处理遥控器、上簧机构和击发机构。
 */
void RobotTask(void);

#endif // CONTROL_2026_ROBOT_H
