/**
 ******************************************************************************
 * @file    robot_to_custom_controller.h
 * @brief   机器人发送数据给自定义控制器 - UserApp/components层
 * @note    通过USART1发送机械臂5个电机角度数据给自定义控制器
 *          使用协议ID: 0x0309, 发送频率上限10Hz
 ******************************************************************************
 */
#ifndef ROBOT_TO_CUSTOM_CONTROLLER_H
#define ROBOT_TO_CUSTOM_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_usart.h"
#include "protocol.h"

/* ----------------------- 数据结构体 ----------------------------- */

/**
 * @brief 机器人发送给自定义控制器的数据包
 */
typedef struct {
    uint8_t packet_type;              // 数据包类型标识 (0x30)
    float motor_angles[5];            // 5个机械臂电机角度值(单位:度)
} RobotToCustomData_t;

/* ----------------------- 函数声明 ----------------------------- */

/**
 * @brief 初始化机器人到自定义控制器的通信
 * @param usart_instance USART实例指针(通常为USART1)
 * @return true 初始化成功, false 初始化失败
 */
bool RobotToCustomController_Init(USARTInstance* usart_instance);

/**
 * @brief 发送机械臂电机数据给自定义控制器
 * @param motor_angles 5个电机的角度值数组(单位:度)
 * @param usart_instance USART实例指针
 * @note 使用CMD_ID 0x0309, 数据格式: packet_type(1字节) + 5个float(20字节) = 21字节
 */
void RobotToCustomController_SendMotorData(const float motor_angles[5], USARTInstance* usart_instance);

#endif // ROBOT_TO_CUSTOM_CONTROLLER_H
