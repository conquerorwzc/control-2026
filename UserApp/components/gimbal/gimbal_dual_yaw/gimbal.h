/**
 ******************************************************************************
 * @file    gimbal.h
 * @author  SRM-Control 2026
 * @brief   Dual Yaw Gimbal Module - 双yaw串联云台模块
 ******************************************************************************
 * @attention
 * 双yaw串联云台结构：
 *          yaw_master (GM6020)
 *               │
 *          yaw_slave (GM6020)
 *               │
 *          pitch (J4310)
 *
 * 控制逻辑：
 * - 主yaw电机控制整体旋转角度，使用IMU反馈
 * - 副yaw电机根据主yaw角度进行微调，实现叠加旋转
 * - pitch电机使用4310电机，控制俯仰角度
 ******************************************************************************
 */

#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"

typedef enum {
  GIMBAL_POWER_OFF = 0,  // 电流零输入
  GIMBAL_ON,
  GIMBAL_VISION,
} Gimbal_Mode_e;

typedef struct {
  float yaw;
  float pitch;
  float chassis_rotate_wz;
  Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s yaw_master_motor_config;  // 主yaw电机配置（GM6020）
  Motor_Init_Config_s yaw_slave_motor_config;   // 副yaw电机配置（GM6020）
  Motor_Init_Config_s pitch_motor_config;       // pitch电机配置（J4310）
  IMU_Init_Config_s imu_init_config;
} Gimbal_Dual_Yaw_Init_Config_s;

typedef struct {
  Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
  DJIMotorInstance *yaw_master_motor;  // 主yaw电机（GM6020）
  DJIMotorInstance *yaw_slave_motor;   // 副yaw电机（GM6020）
  DMMotorInstance *pitch_motor;        // pitch电机（J4310）
  INS_t* gimbal_IMU_data;              // 云台IMU数据
  float yaw_slave_offset;              // 副yaw相对于主yaw的偏移量
} GimbalDualYawInstance;

/**
 * @brief 初始化双yaw云台,会被RobotInit()调用
 *
 */
GimbalDualYawInstance* GimbalDualYawInit(Gimbal_Dual_Yaw_Init_Config_s* gimbal_init_config);

/**
 * @brief 双yaw云台任务
 *
 */
void GimbalDualYawTask();
