#ifndef CONTROL_2026_GIMBAL_H
#define CONTROL_2026_GIMBAL_H

#pragma once

#include "dji_motor.h"
#include "ins_task.h"

//线性控制器前馈系数
#define YAW_FEED_FORWARD 0.7f		//1.05f
#define PITCH_FEED_FORWARD 0.8f
#define YAW_D 0.0f//15.0
//角度误差项系数
#define K_YAW_ANGLE_ERROR 	140000.0f//90000.0f
#define K_PITCH_ANGLE_ERROR 0.01//350000.0f
//Pitch前馈项
#define PITCH_FORWARD	0.0f

//云台电机二阶线性控制器
typedef struct
{
  //设定值
  float set_angle;
  float set_angle_speed;
  //当前角度一阶状态
  float cur_angle;
  //当前角速度 二阶状态
  float cur_angle_speed;
  //角度误差项 一阶状态误差
  float angle_error;

  //前馈项，用于消除系统固有扰动
  float feed_forward;

  //输出值
  float output;
  //最大输出值
  float max_out;
  //最小输出值
  float min_out;
  //前馈项系数
  float k_feed_forward;
  //误差项系数
  float k_angle_error;
  //二阶角速度项系数
  float k_angle_speed;
}gimbal_motor_second_order_linear_controller_t;

typedef enum {
  GIMBAL_POWER_OFF = 0,  // 电流零输入
  GIMBAL_ON,
} Gimbal_Mode_e;

typedef struct {
  float yaw;
  float pitch;
  float chassis_rotate_wz;
  Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s yaw_motor_config;
  Motor_Init_Config_s pitch_motor_config;
} Gimbal_Init_Config_s;

typedef struct {
  Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
  DJIMotorInstance *yaw_motor, *pitch_motor;
  attitude_t* gimbal_IMU_data;  // 云台IMU数据
  gimbal_motor_second_order_linear_controller_t gimbal_motor_second_order_linear_controller;
  gimbal_motor_second_order_linear_controller_t gimbal_pitch_motor_second_order_linear_controller;
} GimbalInstance;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config);

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

#endif  // CONTROL_2026_GIMBAL_H
