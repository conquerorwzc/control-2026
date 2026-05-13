#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include "rm_referee.h"
#include "gimbal.h"
#include "shoot.h"


// 1. 定义变化标志位结构体
typedef struct {
  uint32_t gimbal_flag   : 1;
  uint32_t shoot_flag    : 1;
  uint32_t friction_flag : 1;
  uint32_t laser_flag    : 1;
  uint32_t heat_flag     : 1;
  uint32_t speed_flag    : 1;
  uint32_t vision_flag   : 1;
} ext_Referee_Interactive_Flag_t;

// 2. 定义核心交互信息结构体
typedef struct {
  ext_Referee_Interactive_Flag_t Referee_Interactive_Flag;

  // 当前模式/数值
  Gimbal_Mode_e gimbal_mode;
  Shoot_Mode_e  shoot_mode;
  Friction_Mode_e friction_mode;
  uint16_t laser_time;

  uint16_t heat;           // 新增：当前枪口热量
  float initial_speed;     // 新增：射击初速度
  uint8_t vision_lock;     // 新增：自瞄状态 (0:未开启, 1:开启锁定)

  // 上一次的模式/数值（用于 UIChangeCheck 比对）
  Gimbal_Mode_e gimbal_last_mode;
  Shoot_Mode_e  shoot_last_mode;
  Friction_Mode_e friction_last_mode;
  uint16_t last_laser_time;

  uint16_t last_heat;
  float last_initial_speed;
  uint8_t last_vision_lock;

  uint8_t force_refresh_ui;
} Referee_Interactive_info_t;

// 函数声明
void MyUIInit(void);
void UITask(void);
Referee_Interactive_info_t* getUI(void);

#endif