//
// Created by zhan_ on 2025/12/2.
//

#ifndef CONTROL_2026_SELFCONTROL_H
#define CONTROL_2026_SELFCONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

typedef struct
{
  uint8_t selfcontrol_buff[21]; // 遥控器接收buffer
} SelfC;
/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
SelfC *SelfControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */


#endif  // CONTROL_2026_SELFCONTROL_H
