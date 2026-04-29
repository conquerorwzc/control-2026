#ifndef REFEREE_H
#define REFEREE_H
#include "robot.h"
#include "rm_referee.h"
#pragma pack(1)
typedef struct{
  xFrameHeader FrameHeader;   //帧头
  uint16_t cmd_id;          //0x301
  uint16_t data_cmd_id;     //0x120
  uint16_t sender_id;       //0x8080
  uint16_t receiver_id;     //红7蓝107
  uint8_t user_data[4];     //哨兵自主决策数据
  uint16_t frametail;       //CRC16校验
}sentry_interaction_data_t;
#pragma pack()
/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();
#ifdef CHASSIS_BOARD
void SentryInit();
void SentryTask();
#endif

#endif // REFEREE_H
