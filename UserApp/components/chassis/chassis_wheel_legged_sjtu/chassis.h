/**
******************************************************************************
* @file    chassis.h
* @author  Enhao Zhang
* @date    2026/2/25
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   SJTU Full-Chassis Wheel-Legged Module (10-dim state, 4x10 LQR)
******************************************************************************
* @attention
* Wheel-Legged Chassis Layout:
* LEFT Leg[1]       Leg[0] RIGHT
*        ☉----------☉
*        |          |
*        |          |
*        |          |
*        |          |
*        ◉          ◉
*       ---        ---
* State: [X_b_h, V_b_h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]
* Control: u = [T_r_to_b, T_l_to_b, T_wr_to_r, T_wl_to_l]
******************************************************************************
*/
#pragma once

#include "ins_task.h"
#include "parallel_leg.h"

typedef enum {
  CHASSIS_POWER_OFF = 0,
  CHASSIS_RECOVERY,
  CHASSIS_ON,
  CHASSIS_JUMP_READY,
  CHASSIS_JUMP_START,
} Chassis_Mode_e;

typedef enum {
  JUMP_STATE_IDLE,
  JUMP_STATE_COMPRESS,
  JUMP_STATE_EXTEND,
  JUMP_STATE_RETRACT,
} Jump_State_e;

typedef struct {
  float k0;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
} Power_Param_3508_s;

typedef struct {
  float vx;
  float wz;          // 保留用于小陀螺前馈角速度.单位m/s
  float target_yaw;  // 新增：LQR目标yaw角度 (rad)
  float roll;
  float leg_length;
  float jump_force;
  float theta_ff;
  int chassis_speed_buff;
  uint16_t max_power;
  Chassis_Mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;
/* SJTU model: 10-dim state vector */
typedef struct {
  float x_b_h;     // 机身在水平地面的水平位置（全局系），无传感器，保持为0或积分得到
  float v_b_h;     // 机身在水平地面的水平速度（全局系），底盘前向平均速度
  float phi;       // 机身航向角 yaw（全局系, rad），由IMU YawTotalAngle获得
  float dphi;      // 机身航向角速度 yaw_rate（rad/s），由IMU陀螺仪Gyro[2]获得
  float theta_l;   // 左腿杆体与Z负方向夹角（rad），由VMC解算
  float dtheta_l;  // 左腿角速度（rad/s）
  float theta_r;   // 右腿杆体与Z负方向夹角（rad），由VMC解算
  float dtheta_r;  // 右腿角速度（rad/s）
  float theta_b;   // 机身俯仰角 pitch（rad），由IMU Pitch获得
  float dtheta_b;  // 机身俯仰角速度 pitch_rate（rad/s），由IMU陀螺仪Gyro[0]获得
} State_Var_t;

/* K matrix 4x10, 2D poly fitting coeffs [p00,p10,p01,p20,p11,p02] per element */

typedef struct {
  float track_width;
  float body_mass;
  float initial_leg_length;
  float leg_min_length;
  float leg_max_length;
  float LQR_K_Coefficients[40][6];
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s param;
  Leg_Init_Config_s leg_init_config[2];
  PID_Init_Config_s delta_theta_PID_config;
  PID_Init_Config_s roll_PID_config;
  PID_Init_Config_s length_PID_config;
  IMU_Init_Config_s imu_init_config;
} Chassis_Init_Config_s;

typedef struct {
  Jump_State_e jump_state;
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  Chassis_Param_s param;

  INS_t* imu;

  KalmanFilter_t vaEstimateKF;

  State_Var_t state_var;
  State_Var_t last_state_var;

  LegInstance* leg[2];
  PIDInstance delta_theta_PID;
  PIDInstance roll_PID;
  PIDInstance length_PID[2];  // 每腿单独腿长环

  float delta_theta_comp;

  float LQR_K[4][10];  // [4输出][10状态变量]

  uint32_t DWT_CNT;
  float dt;
  struct {
    uint8_t is_first_update : 1;  // 观测器和状态变量是否完成第一次更新
    uint8_t is_restart : 1;       // 是否需要重启更新（如时间步长过大时）
    uint8_t is_controlled : 1;    // 是否处于受控状态, 1表示受控（如有前进指令时）, 0表示非受控, 用于切换控制策略
  } update_flag;
} ChassisInstance;

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);
void ChassisTask(void);
