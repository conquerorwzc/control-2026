#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

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

uint8_t SentryInit();
void SentryTask();
#endif // REFEREE_H
