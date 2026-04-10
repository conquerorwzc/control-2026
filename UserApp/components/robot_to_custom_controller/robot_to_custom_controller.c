/**
 ******************************************************************************
 * @file    robot_to_custom_controller.c
 * @brief   机器人发送数据给自定义控制器实现
 ******************************************************************************
 */
#include "robot_to_custom_controller.h"
#include "bsp_log.h"
#include <string.h>

// USART实例(由初始化时传入)
static USARTInstance* robot_to_custom_usart = NULL;

/**
 * @brief 初始化机器人到自定义控制器的通信
 */
bool RobotToCustomController_Init(USARTInstance* usart_instance)
{
    if (usart_instance == NULL) {
        LOGERROR("RobotToCustom: USART instance is NULL");
        return false;
    }
    
    robot_to_custom_usart = usart_instance;
    LOGINFO("RobotToCustom: Initialized successfully");
    return true;
}

/**
 * @brief 发送机械臂电机数据给自定义控制器
 */
void RobotToCustomController_SendMotorData(const float motor_angles[5], USARTInstance* usart_instance)
{
    if (motor_angles == NULL || usart_instance == NULL) {
        return;
    }
    
    // 构造数据包
    uint8_t data_buffer[21] = {0};  // 1字节类型 + 5个float(20字节) = 21字节
    
    // 数据包类型标识
    data_buffer[0] = 0x30;  // 机器人->自定义控制器标识
    
    // 填充5个电机角度值(小端格式)
    for (int i = 0; i < 5; i++) {
        uint8_t* angle_bytes = (uint8_t*)&motor_angles[i];
        memcpy(&data_buffer[1 + i * 4], angle_bytes, 4);
    }
    
    // 使用协议打包函数
    uint16_t packed_length = 0;
    uint8_t* packed_data = custom_controller_protocol_pack(CMD_ID_ROBOT_TO_CUSTOM, 
                                                           data_buffer, 
                                                           21, 
                                                           &packed_length);
    
    // 通过USART发送
    if (packed_data != NULL && packed_length > 0) {
        USARTSend(usart_instance, packed_data, packed_length, USART_TRANSFER_DMA);
    }
}
