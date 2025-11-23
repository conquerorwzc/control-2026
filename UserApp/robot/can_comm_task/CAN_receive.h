#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include <stdint.h>
#include "bsp_can.h"

// 声明接收数据全局变量
extern uint8_t received_ui_flag;
extern uint8_t received_fric_flag;
extern uint8_t received_chassis_vx;
extern uint8_t received_chassis_vy;
extern int16_t received_pitch_abs;
extern uint8_t received_chassis_behaviour;
extern uint8_t received_cap_flag;
extern uint8_t data_received;

// 初始化函数声明
extern void CANReceive_Init(void);

#endif