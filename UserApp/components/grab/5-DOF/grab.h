#pragma once
#include "dji_motor.h"
#include "dmmotor.h"

typedef enum {
  GRAB_POWER_OFF = 0,  // 电流零输入
  GRAB_ON,
} Grab_Mode_e;

typedef struct {
  Motor_Init_Config_s Grab_motor_config[6];  // 修改为数组以支持多个电机
} Grab_Init_Config_s;

typedef struct {
  float r1;
  float r2;
  float r3;
  Grab_Mode_e grab_mode;
} Grab_Ctrl_Cmd_s;

typedef struct {
  DJIMotorInstance* grab_djimotor[2];
  DMMotorInstance* grab_dmmotor[1];
  float wrist_roll;     // 腕部关节旋转角度
  float wrist_pitch;    // 腕部关节俯仰角度
  float gripper_joint;  // 末端夹爪关节角度
  float L_target;       //左侧电机旋转角度
  float R_target;       //右侧电机旋转角度
} ActuatorInstance;

typedef struct {
  DMMotorInstance* grab_dmmotor[3];
  float base_joint;   // 基座旋转关节角度
  float elbow_roll;   // 肘部关节旋转角度
  float elbow_pitch;  // 肘部关节俯仰角度
} ArmInstance;

typedef struct {
  Grab_Ctrl_Cmd_s grab_ctrl_cmd;
  ArmInstance* arm;
  ActuatorInstance* actuator;

} GrabInstance;

GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config);
void GrabTask();