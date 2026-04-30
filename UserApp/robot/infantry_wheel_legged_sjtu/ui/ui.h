#pragma once

#include "robot.h"

#define UI_CENTER_X 960
#define UI_CENTER_Y 540

#pragma pack(1)
typedef struct {
  uint32_t mode_flag : 1;
  uint32_t power_flag : 1;
  uint32_t yaw_flag : 1;
  uint32_t leg_flag : 1;
  uint32_t vision_flag : 1;
  uint32_t hit_flag : 1;
  uint32_t point_flag : 1;
  uint32_t jump_flag : 1;
  uint32_t speed_flag : 1;
  uint32_t distance_flag : 1;
} UI_Interactive_Flag_t;
#pragma pack()

typedef struct {
  UI_Interactive_Flag_t UI_Interactive_Flag;

  Robot_Mode_e robot_mode;
  Chassis_Mode_e chassis_mode;
  Gimbal_Mode_e gimbal_mode;
  Shoot_Mode_e shoot_mode;
  Friction_Mode_e friction_mode;
  SuperCap_Ctrl_Cmd_e cap_ctrl_cmd;
  SuperCap_Measure_s cap_msg;
  Jump_State_e jump_state;

  float chassis_relative_angle;
  float leg_left_angle;
  float leg_right_angle;
  float current_speed;
  float jump_distance;
  uint16_t buffer_energy;
  uint32_t event_type;
  uint8_t vision_tracking;
  uint8_t autoaim_hit;
  uint8_t auto_jump_req;
  uint8_t force_refresh_ui;

  Robot_Mode_e last_robot_mode;
  Chassis_Mode_e last_chassis_mode;
  Gimbal_Mode_e last_gimbal_mode;
  Shoot_Mode_e last_shoot_mode;
  Friction_Mode_e last_friction_mode;
  SuperCap_Ctrl_Cmd_e last_cap_ctrl_cmd;
  Jump_State_e last_jump_state;
  float last_chassis_relative_angle;
  float last_leg_left_angle;
  float last_leg_right_angle;
  float last_current_speed;
  float last_jump_distance;
  float last_cap_voltage;
  float last_cap_in_power;
  float last_cap_out_power;
  uint16_t last_buffer_energy;
  uint32_t last_event_type;
  uint8_t last_cap_error;
  uint8_t last_vision_tracking;
  uint8_t last_autoaim_hit;
  uint8_t last_auto_jump_req;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI(void);
void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);
