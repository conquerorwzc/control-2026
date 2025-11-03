/**
******************************************************************************
* @file    chassis.h
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   Mecanum Chassis Module
******************************************************************************
* @attention
* Mecanum Chassis Motor Layout:
*
*          motor[0]     motor[1]
*        Left Front   Right Front
*             ╭─────────────╮
*             │      ↑      │
*             │    Front    │
*             ╰─────────────╯
*          motor[2]     motor[3]
*        Left Rear    Right Rear
*
* @note    Motor Index:
*          motor[0] - Left Front Wheel
*          motor[1] - Right Front Wheel
*          motor[2] - Left Rear Wheel
*          motor[3] - Right Rear Wheel
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
} Chassis_Mode_e;

typedef struct {
  Chassis_Mode_e chassis_mode;
  float vx;  // 前进方向速度
  float wz;  // 旋转速度
  float roll;
  float leg_length_d;
  float offset_angle;  // 底盘和归中位置的夹角
  int chassis_speed_buff;
  uint16_t max_power;  // 最大功率限制
} Chassis_Ctrl_Cmd_s;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  Leg_Param_s leg_param;
  float center_gimbal_offset_x;  // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
  float center_gimbal_offset_y;  // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
  float wheel_radius;            // 轮子半径
  float robot_weight;
  float wheel_reduction_ratio;  // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s chassis_param;             // 底盘参数
  Leg_Init_Config_s leg_init_config[2];      // 轮腿实例配置文件
  PID_Init_Config_s delta_theta_PID_config;  // 防劈叉PID
  PID_Init_Config_s roll_PID_config;         // Roll PID
} Chassis_Init_Config_s;

typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  LegInstance* leg[2];
  attitude_t* chassis_IMU_data;

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