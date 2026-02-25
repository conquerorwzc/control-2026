#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

//车辆示宽线
#define WIDTHLINE_UP                330  //上方间距
#define WIDTHLINE_DOWN              610  //下方间距

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

#endif // REFEREE_H
