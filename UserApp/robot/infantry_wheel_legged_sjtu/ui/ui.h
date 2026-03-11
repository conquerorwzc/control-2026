#ifndef INFANTRY_UI_H
#define INFANTRY_UI_H

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

// Define types locally for decoupling
typedef enum {
    LID_CLOSE = 0,
    LID_OPEN,
} Lid_Mode_e;

typedef struct {
    float chassis_power_mx;
} Chassis_Power_Data_s;

typedef struct {
  uint32_t chassis_flag : 1;
  uint32_t gimbal_flag : 1;
  uint32_t shoot_flag : 1;
  uint32_t lid_flag : 1;
  uint32_t fric_flag : 1;
  uint32_t Power_flag : 1;
  uint32_t ammo_flag : 1;
  uint32_t pitch_flag : 1;
  uint32_t autoaim_flag : 1;
  uint32_t cap_flag : 1;
  uint32_t yaw_flag : 1;
} UI_Interactive_Flag_t;

typedef struct {
  UI_Interactive_Flag_t Referee_Interactive_Flag;
  
  Chassis_Mode_e chassis_mode;
  Chassis_Mode_e chassis_last_mode;
  
  Gimbal_Mode_e gimbal_mode;
  Gimbal_Mode_e gimbal_last_mode;
  
  Shoot_Mode_e shoot_mode;
  Shoot_Mode_e shoot_last_mode;
  
  Friction_Mode_e fric_mode;
  Friction_Mode_e fric_last_mode;
  
  Lid_Mode_e lid_mode;
  Lid_Mode_e lid_last_mode;
  
  Chassis_Power_Data_s Chassis_Power_Data;
  Chassis_Power_Data_s Chassis_last_Power_Data;
  
  float pitch_angle;
  float last_pitch_angle;
  
  uint8_t autoaim_mode;
  uint8_t last_autoaim_mode;
  
  float cap_voltage;
  float last_cap_voltage;
  
  uint8_t cap_mode;
  uint8_t last_cap_mode;
  
  uint16_t bullet_left_real;
  uint16_t last_bullet_left_real;
  
  float fric_speed_left;
  float last_fric_speed_left;
  
  float fric_speed_right;
  float last_fric_speed_right;
  
  float chassis_relative_angle;
  float last_chassis_relative_angle;

} Referee_Interactive_info_t;

// 暴露给外部的接口
Referee_Interactive_info_t *getUI();

/**
 * @brief UI初始化
 */
void MyUIInit(void* robot);

/**
 * @brief UI任务
 */
void UITask(void const *argument);


#endif // INFANTRY_UI_H
