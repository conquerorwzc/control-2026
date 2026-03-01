#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

//车辆示宽线
#define WIDTHLINE_UP                330  //上方间距
#define WIDTHLINE_DOWN              610  //下方间距

// 摩擦轮转速范围
#define FRIC_LOWER 5000
#define FRIC_UPPER 7000

// 电容电压范围
#define CAP_VOL_LOWER 16.0f
#define CAP_VOL_UPPER 23.0f

// 弹量上限
#define AMMO_UPPER 500

// #define QQ_SUPER_CAP

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
