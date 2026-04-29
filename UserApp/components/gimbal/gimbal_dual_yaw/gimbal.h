/**
 ******************************************************************************
 * @file    gimbal.h
 * @author  SRM-Control 2026
 * @brief   Dual Yaw Gimbal Module - 双yaw串联云台模块 (虚拟陀螺方案)
 ******************************************************************************
 * @attention
 *
 *  机械结构 (自上而下):
 *    pitch (J4310/DM)
 *      └── 小yaw / yaw_slave (GM6020)  — 精跟踪, 实物陀螺仪(安装在pitch侧)
 *           └── 大yaw / yaw_master (GM6020) — 粗跟踪/重定位, 虚拟陀螺仪
 *                └── 底盘
 *
 *  虚拟陀螺仪:
 *    大yaw没有实物陀螺仪, 通过小yaw的实物陀螺仪 + 小yaw电机编码器差值反算:
 *      θ_virtual = θ_gyro - θ_slave_rel         — 虚拟绝对航向角
 *      ω_virtual = ω_gyro - dir * ω_slave_aps   — 虚拟绝对角速度
 *    大yaw PID 使用 OTHER_FEED, 指向上述虚拟量, 与控制标准云台一致。
 *
 *  状态机:
 *    NORMAL (|θ_rel| < 15°):  大yaw ref = virtual_hold → 静止
 *                            小yaw IMU跟目标 → θ_rel 渐增
 *    FOLLOW (15°≤|θ_rel|<30°): 大yaw ref = θ_gyro (真实陀螺) → PID 追
 *                              error = θ_gyro - θ_virtual = θ_rel
 *    LOCKED (|θ_rel| ≥ 30°):  大yaw ref = 锁定时捕获的 θ_gyro (冻结)
 *                            小yaw 断电
 *      → 恢复: |virtual_gyro - lock_target| < 3°
 *
 ******************************************************************************
 */

#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"

/* ========================== 可调宏定义 ========================== */

// 小yaw机械对中大yaw时的编码器原始值 (实车测量后填入)
#ifndef SLAVE_YAW_CENTER_ECD
#define SLAVE_YAW_CENTER_ECD 2048
#endif
#define SLAVE_YAW_CENTER_ANGLE (SLAVE_YAW_CENTER_ECD * ECD_ANGLE_COEF_DJI)

// 小yaw电机速度方向与IMU Yaw轴一致性修正 (1=同向, -1=反向)
#ifndef SLAVE_YAW_SPEED_DIR
#define SLAVE_YAW_SPEED_DIR 1
#endif

// 状态切换阈值 (单位: °)
#define DUAL_YAW_ENTER_FOLLOW_THRESHOLD  15.0f
#define DUAL_YAW_EXIT_FOLLOW_THRESHOLD   10.0f   // 迟滞退出, 防止边界抖动
#define DUAL_YAW_LOCK_THRESHOLD          30.0f
#define DUAL_YAW_RECOVER_THRESHOLD        3.0f

/* ========================== 类型定义 ========================== */

typedef enum {
  GIMBAL_POWER_OFF = 0,
  GIMBAL_ON,
  GIMBAL_VISION,
} Gimbal_Mode_e;

typedef enum {
  DUAL_YAW_NORMAL = 0,
  DUAL_YAW_FOLLOW,
  DUAL_YAW_LOCKED,
} DualYawState_e;

typedef struct {
  float yaw;
  float pitch;
  float chassis_rotate_wz;
  Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s yaw_master_motor_config;  // 大yaw, NORMAL状态PID由此配置
  Motor_Init_Config_s yaw_slave_motor_config;   // 小yaw
  Motor_Init_Config_s pitch_motor_config;       // pitch
  IMU_Init_Config_s imu_init_config;

  // 大yaw在FOLLOW/LOCKED状态使用的PID参数 (NORMAL状态使用base motor config中的PID)
  PID_Init_Config_s yaw_master_follow_angle_PID;
  PID_Init_Config_s yaw_master_follow_speed_PID;
} Gimbal_Dual_Yaw_Init_Config_s;

typedef struct {
  Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;

  DJIMotorInstance *yaw_master_motor;  // 大yaw
  DJIMotorInstance *yaw_slave_motor;   // 小yaw
  DMMotorInstance *pitch_motor;

  INS_t *gimbal_IMU_data;

  DualYawState_e state;
  DualYawState_e state_last;
  uint8_t align_flag;             // 上电对齐完成标志

  // --- 虚拟陀螺仪 (大yaw PID 的 OTHER_FEED 来源) ---
  float virtual_gyro_angle;       // θ_virtual = θ_gyro - θ_slave_rel_continuous
  float virtual_gyro_speed;       // ω_virtual = ω_gyro - dir * ω_slave_aps

  // --- 中心参考系 (基于 SLAVE_YAW_CENTER_ECD) ---
  float slave_center_total_offset; // slave_total_angle 对中时的偏移量 (连续值)

  // --- 状态目标 ---
  float virtual_gyro_hold_angle;  // NORMAL状态下大yaw的虚拟陀螺保持值
  float virtual_gyro_lock_target; // LOCKED状态下冻结的虚拟陀螺目标值

  // --- 两套PID参数 ---
  PID_Init_Config_s normal_angle_pid;
  PID_Init_Config_s normal_speed_pid;
  PID_Init_Config_s follow_angle_pid;
  PID_Init_Config_s follow_speed_pid;

  // --- 前馈电流 (用于补偿大小yaw耦合) ---
  float yaw_master_feedforward_current;
  float yaw_slave_feedforward_current;

  // --- 可调阈值 ---
  float enter_follow_threshold;
  float exit_follow_threshold;
  float lock_threshold;
  float recover_threshold;
} GimbalDualYawInstance;

/* ========================== 函数声明 ========================== */

GimbalDualYawInstance* GimbalDualYawInit(Gimbal_Dual_Yaw_Init_Config_s *config);
void GimbalDualYawTask();

/**
 * @brief 设置大小yaw的前馈电流, 用于解耦补偿
 * @param master_current 大yaw前馈电流 (mA)
 * @param slave_current  小yaw前馈电流 (mA)
 */
void GimbalDualYawSetFeedforward(float master_current, float slave_current);
