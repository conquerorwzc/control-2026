/**
******************************************************************************
* @file    chassis.h
* @author  Enhao Zhang
* @date    2025/8/8
* @brief   chassis control for parallel wheel-legged robot
******************************************************************************
* @attention
* None
*
******************************************************************************
*/
#pragma once

#include "ins_task.h"
#include "parallel_leg.h"

typedef enum {
  CHASSIS_POWER_OFF = 0,  // 电流零输入
  CHASSIS_POWER_ON,
} Chassis_Mode_e;

typedef struct {
  // 控制部分
  float vx;  // 前进方向速度
  float vy;  // 横移方向速度
  float wz;  // 旋转速度
  Chassis_Mode_e chassis_mode;
  float roll;  // 横滚补偿
  float yaw;   // 偏航补偿
  // UI部分
  //  ...

} Chassis_Ctrl_Cmd_s;

typedef struct {
  Leg_Init_Config_s leg_init_config[2];
  PID_Init_Config_s delta_theta_PID_config;
  PID_Init_Config_s roll_PID_config;
  PID_Init_Config_s yaw_PID_config;
} Chassis_Init_Config_s;

typedef struct {
  LegInstance* leg[2];
  attitude_t* chassis_IMU_data;

  PIDInstance delta_theta_PID;  // Only use PD
  PIDInstance roll_PID;         // Only use P
  PIDInstance yaw_PID;          // Only use PD

  float delta_theta_comp;
  float roll_comp;
  float yaw_comp;

  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;  // 底盘接收到的控制命令
} ChassisInstance;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 *
 */
void ChassisTask();
