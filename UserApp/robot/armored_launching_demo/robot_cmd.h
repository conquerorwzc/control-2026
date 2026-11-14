
#ifndef CONTROL_2026_ROBOT_CMD_H
#define CONTROL_2026_ROBOT_CMD_H

/**
 * @brief 初始化 USART3 遥控器接收，并注册动作指令的发布者。
 */
void RobotCmdInit(void);

/**
 * @brief 发布左摇杆上推上簧、下拨回位及右摇杆上推击发的请求。
 */
void RobotCmdTask(void);

#endif // CONTROL_2026_ROBOT_CMD_H
