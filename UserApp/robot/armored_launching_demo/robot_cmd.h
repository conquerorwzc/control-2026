//
// Created by CX330y on 2026/11/11.
//

#ifndef CONTROL_2026_ROBOT_CMD_H
#define CONTROL_2026_ROBOT_CMD_H

/**
 * @brief 初始化 USART3 遥控器接收，并注册动作指令的发布者。
 */
void RobotCmdInit(void);

/**
 * @brief 读取左右摇杆的竖直通道，在检测到一次上推时发布对应的动作请求。
 */
void RobotCmdTask(void);

#endif // CONTROL_2026_ROBOT_CMD_H
