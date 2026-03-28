/**
* @file    chassis.h
* @author  Shiyu Li
* @date    2026/3/28
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   wheel_leg->prostrate
*/
#pragma once

#include <stdint.h>

#include "ins_task.h"
#include "super_cap.h"
#include "dji_motor.h"
#include "dmmotor.h"


// 中科大的功率模型
// ===== M3508轮毂电机 6参数模型系数 =====
#define WHEEL_K0 0.7441993412640775f
#define WHEEL_K1 0.0090164284468539646f
#define WHEEL_K2 0.0001988857226262331f
#define WHEEL_K3 0.024694430204543864f
#define WHEEL_K4 0.20160143850678086f
#define WHEEL_K5 3.715221772539512e-05f
// ===== DJI M3508 =====
#define DJI_CURRENT_SCALE     (16384.0f / 20.0f)

typedef enum {
  CHASSIS_POWER_OFF = 0,
  CHASSIS_PROSTRATE,
} Chassis_Mode_e;

typedef struct {
  float vx;
  float wz;
  float target_yaw;
  uint16_t max_power;
  Chassis_Mode_e chassis_mode;
  uint8_t SuperCapBoost;
} Chassis_Ctrl_Cmd_s;


//超级电容策略结构体
typedef enum {
  SAFETY_MODE=0,//安全模式，超电电压低于8伏时进入，大于18伏退出，底盘限制30W
  PASSIVE_MODE,//被动模式，超电电压正常时的工作模式
  ACTIVE_MODE,//，主动模式，主动使用超电能量
  CHARGING_MODE,//充电模式，衰减底盘功率，保障电容电压健康
  FORCED_CHARGING_MODE,//强制充电模式，更极端的功率衰减，强制超电快速充电
} SuperCapMode;

typedef struct {
  float track_width;
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s param;
  Motor_Init_Config_s wheel_motor_config[2];
  Motor_Init_Config_s joint_motor_config[4];
  IMU_Init_Config_s imu_init_config;
  SuperCap_Init_Config_s super_cap_config;
  PID_Init_Config_s yaw_prostrate_PID_config;
} Chassis_Init_Config_s;


typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  Chassis_Param_s param;

  INS_t* imu;

  SuperCapInstance* super_cap;
  SuperCapMode super_cap_mode;

  DJIMotorInstance* wheel_motor[2];
  DMMotorInstance* joint_motor[4];
  PIDInstance yaw_prostrate_PID;
} ChassisInstance;

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);
void ChassisTask(void);
