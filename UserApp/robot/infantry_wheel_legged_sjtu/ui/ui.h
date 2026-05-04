#pragma once

#include "robot.h"

#define UI_CENTER_X 960
#define UI_CENTER_Y 540

#pragma pack(1)
typedef struct {
  uint32_t relative_flag : 1;
  uint32_t leg_flag : 1;
  uint32_t status_flag : 1;
  uint32_t cap_flag : 1;
} UI_Interactive_Flag_t;
#pragma pack()

typedef struct {
  UI_Interactive_Flag_t UI_Interactive_Flag;

  float chassis_relative_angle;
  float last_chassis_relative_angle;

  uint8_t leg_valid;
  uint8_t last_leg_valid;
  float leg_phi1;
  float leg_phi2;
  float leg_phi3;
  float leg_phi4;
  float last_leg_phi1;
  float last_leg_phi2;
  float last_leg_phi3;
  float last_leg_phi4;

  Robot_Mode_e robot_mode;
  Robot_Mode_e last_robot_mode;
  Chassis_Mode_e chassis_mode;
  Chassis_Mode_e last_chassis_mode;
  Gimbal_Mode_e gimbal_mode;
  Gimbal_Mode_e last_gimbal_mode;
  Friction_Mode_e friction_mode;
  Friction_Mode_e last_friction_mode;
  Loader_Mode_e loader_mode;
  Loader_Mode_e last_loader_mode;
  SuperCap_Mode_e super_cap_mode;
  SuperCap_Mode_e last_super_cap_mode;
  float cap_voltage;
  float last_cap_voltage;
  uint8_t cap_error;
  uint8_t last_cap_error;

  uint8_t force_refresh_ui;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI(void);
void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);
