/**
******************************************************************************
* @file    chassis.h
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief  Parallel Wheel-Legged Chassis Module
******************************************************************************
* @attention
* Wheel-Legged Chassis Layout:
* LEFT Leg[1]       Leg[0] RIGHT
*        ☉----------☉
*        |          |
*        |          |
*        |          |
*        ◉          ◉
*        |          |
*        |          |
*       ---        ---
*       | |        | |
*       ---        ---
******************************************************************************
*/
#pragma once

#include "dji_motor.h"
#include "ins_task.h"
#include "parallel_leg.h"

typedef enum {
  CHASSIS_POWER_OFF = 0,  // 电流零输入
  CHASSIS_RECOVERY,       // 一阶倒立摆
  CHASSIS_ON,             // 二阶倒立摆
  CHASSIS_JUMP,           // 跳跃
} Chassis_Mode_e;

typedef enum {
  JUMP_STATE_IDLE,      // 空闲状态
  JUMP_STATE_COMPRESS,  // 压缩状态（施加F）
  JUMP_STATE_EXTEND,    // 伸腿状态（准备跳跃）
  JUMP_STATE_RETRACT,   // 收腿状态
  JUMP_STATE_LAND       // 着陆状态
} Jump_State_e;

#pragma pack(1)
typedef struct {
  Chassis_Mode_e chassis_mode;
  float vx;  // 前进方向速度
  float wz;  // 旋转速度
  float roll;
  float leg_length;
  float offset_angle;  // 底盘和归中位置的夹角
  int chassis_speed_buff;
  uint16_t max_power;  // 最大功率限制
} Chassis_Ctrl_Cmd_s;
#pragma pack()

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  float center_gimbal_offset_x;  // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
  float center_gimbal_offset_y;  // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
  float track_width;
  float robot_weight;
  float leg_length_initial;
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s chassis_param;             // 底盘参数
  Leg_Init_Config_s leg_init_config[2];      // 轮腿实例配置文件
  PID_Init_Config_s delta_theta_PID_config;  // 防劈叉PID
  PID_Init_Config_s roll_PID_config;         // Roll PID
  IMU_Init_Config_s imu_init_config;
} Chassis_Init_Config_s;

typedef struct {
  Jump_State_e jump_state;
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  LegInstance* leg[2];
  INS_t* chassis_IMU;
  KalmanFilter_t vaEstimateKF;

  PIDInstance delta_theta_PID;  // Only use PD
  PIDInstance roll_PID;         // Only use P

  float delta_theta_comp;
  float roll_comp;
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