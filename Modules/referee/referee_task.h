#ifndef REFEREE_H
#define REFEREE_H
#include "robot.h"
#include "rm_referee.h"
#pragma pack(1)
typedef struct{
  xFrameHeader FrameHeader;
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[4];
  uint16_t frametail;
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
