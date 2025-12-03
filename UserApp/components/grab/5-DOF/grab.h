#pragma once
#include "dmmotor.h"
#include "dji_motor.h"

typedef enum {
  GRAB_POWER_OFF = 0,  // 电流零输入
  GRAB_ON,
} Grab_Mode_e;

typedef struct {
  Motor_Init_Config_s Grab_motor_config[5];  // 修改为数组以支持多个电机
} Grab_Init_Config_s;

typedef struct {
  float r1;
  float r2;
  float r3;
  Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct {
  Grab_Ctrl_Cmd_s grab_ctrl_cmd;
  DMMotorInstance* grab_dmmotor[3];
  DJIMotorInstance* grab_djimotor[2]; // 修改为数组以支持多个电机实例
} GrabInstance;


GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config);
void GrabTask();