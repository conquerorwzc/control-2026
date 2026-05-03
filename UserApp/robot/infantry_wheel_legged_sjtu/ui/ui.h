#pragma once

#include "robot.h"

#define UI_CENTER_X 960
#define UI_CENTER_Y 540
#define UI_RELATIVE_CENTER_X 960
#define UI_RELATIVE_CENTER_Y 540

#pragma pack(1)
typedef struct {
  uint32_t relative_flag : 1;
} UI_Interactive_Flag_t;
#pragma pack()

typedef struct {
  UI_Interactive_Flag_t UI_Interactive_Flag;

  float chassis_relative_angle;
  float last_chassis_relative_angle;
  uint8_t force_refresh_ui;
} Referee_Interactive_info_t;

Referee_Interactive_info_t *getUI(void);
void MyUIInit(RobotInstance *robot);
void UITask(RobotInstance *robot);
