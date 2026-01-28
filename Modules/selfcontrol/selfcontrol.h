//
// Created by zhan_ on 2025/12/2.
//

#ifndef CONTROL_2026_SELFCONTROL_H
#define CONTROL_2026_SELFCONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 定义接收和解析的数据结构
typedef struct {
    uint8_t id;           // 舵机ID
    float angle;          // 舵机角度
    float smooth_angle;   // 【新增】平滑后的目标
    uint8_t torque_status; // 舵机扭矩状态
    uint8_t is_online;    // 舵机在线状态
} ServoData_t;

typedef struct {
    uint8_t id;           // 电位器ID
    float angle;          // 电位器角度
    float smooth_angle;   // 【新增】电位器平滑值
    float voltage;        // 电位器电压
} PotentiometerData_t;

typedef struct {
    ServoData_t servos[3];           // 3个舵机的数据
    PotentiometerData_t pots[2];     // 2个电位器的数据
} UnpackedControllerData_t;

typedef struct
{
  uint8_t selfcontrol_buff[64]; // 遥控器接收buffer
  UnpackedControllerData_t unpacked_data;  // 解析后的控制器数据
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
SelfC *SelfControlInit(UART_HandleTypeDef *rc_usart_handle);

void SelfControl_Smooth_Update(void); //平滑声明
/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */

#endif  // CONTROL_2026_SELFCONTROL_H