#pragma once
#include "dmmotor.h"

typedef struct {
  Motor_Init_Config_s Grab_motor_config[3];  // 修改为数组以支持多个电机
} Grab_Init_Config_s;

typedef struct {
  float r1;
  float r2;
  float r3;
} Grab_Ctrl_Cmd_s;

typedef struct {
  Grab_Ctrl_Cmd_s Grab_ctrl_cmd;
  DMMotorInstance* Grab_motor[3];  // 修改为数组以支持多个电机实例
} GrabInstance;


GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config);
void GrabTask();