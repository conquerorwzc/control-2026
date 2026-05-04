#pragma once

#include "robot.h"

#define UI_CENTER_X 960
#define UI_CENTER_Y 540

#pragma pack(1)
typedef struct {
  uint32_t relative_flag : 1;
  uint32_t leg_flag : 1;
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

  uint8_t force_refresh_ui;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI(void);
void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);
