//
// Created by zhan_ on 2025/12/2.
// Modified for new custom controller interface
//

#ifndef CONTROL_2026_SELFCONTROL_H
#define CONTROL_2026_SELFCONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 电机数据结构
typedef struct {
    uint8_t id;           // 电机ID
    float angle;          // 电机角度 (0-360度)
    uint8_t is_online;    // 电机在线状态
} MotorData_t;

// 电位器数据结构
typedef struct {
    uint8_t id;           // 电位器ID
    float angle;          // 电位器角度 (0-360度)
    float voltage;        // 电位器电压
} PotentiometerData_t;

// 解析后的控制器数据
typedef struct {
    MotorData_t motors[4];           // 4个电机的数据 (1个4310 + 2个3508 + 1个2006)
    PotentiometerData_t pots[1];     // 1个电位器的数据
} UnpackedControllerData_t;

// 自定义控制器实例
typedef struct {
    uint8_t selfcontrol_buff[64];    // 接收缓冲区
    UnpackedControllerData_t unpacked_data;  // 解析后的数据
    uint8_t is_initialized;          // 初始化标志
    uint8_t is_active;               // 活跃状态
} SelfC;

typedef __packed struct {
  uint8_t SOF;           // 起始字节，固定值为0xA5
  uint16_t data_length;  // 数据帧中 data 的长度
  uint8_t seq;           // 包序号
  uint8_t CRC8;          // 帧头 CRC8 校验
} Frame_Header;

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
/**
 * @brief 初始化自定义控制器
 * @param usart_handle USART句柄
 * @return SelfC* 控制器实例指针
 */
SelfC *SelfControlInit(UART_HandleTypeDef *usart_handle);

/**
 * @brief 数据解析函数(保持原有可用逻辑)
 * @param frame 接收到的数据帧
 */
void selfcontrol_data_solve(uint8_t* frame);

/**
 * @brief 获取解析后的控制器数据指针
 * @return UnpackedControllerData_t* 数据指针
 */
UnpackedControllerData_t* GetSelfControlDataPtr(void);

/**
 * @brief 获取指定电机角度
 * @param controller 控制器实例
 * @param motor_index 电机索引(0-3)
 * @return float 电机角度
 */
float SelfControlGetMotorAngle(const SelfC* controller, uint8_t motor_index);

/**
 * @brief 获取指定电位器角度
 * @param controller 控制器实例
 * @param pot_index 电位器索引(0)
 * @return float 电位器角度
 */
float SelfControlGetPotAngle(const SelfC* controller, uint8_t pot_index);

/**
 * @brief 角度标准化(下位机已实现此功能)
 * @param angle 输入角度
 * @return float 原始角度(直接返回)
 */
float SelfControlNormalizeAngle(float angle);

#endif  // CONTROL_2026_SELFCONTROL_H