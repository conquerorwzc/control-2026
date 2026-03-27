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
#include "super_cap.h"

// 中科大的功率模型
// ===== M3508轮毂电机 6参数模型系数 =====
#define WHEEL_K0 0.7441993412640775f
#define WHEEL_K1 0.0090164284468539646f
#define WHEEL_K2 0.0001988857226262331f
#define WHEEL_K3 0.024694430204543864f
#define WHEEL_K4 0.20160143850678086f
#define WHEEL_K5 3.715221772539512e-05f
// ===== 功率控制参数 =====
#define POWER_DEFAULT_LIMIT   40.0f
#define POWER_KP_VEL          0.1f   // s_d_max = Kp * sqrt(P_ref)
#define POWER_VX_RAMP_ACC     1.0f    // 加速斜率 (m/s²)，对应场景(b)
#define POWER_VX_RAMP_DEC     2.0f    // 减速斜率 (m/s²)，对应场景(c)
// ===== DJI M3508 =====
#define DJI_CURRENT_SCALE     (16384.0f / 20.0f)

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
  float vx;
  float wz;          // 保留用于小陀螺前馈角速度.单位rad/s
  float target_yaw;  // 新增：LQR目标yaw角度 (rad)
  float roll;
  float leg_length;
  float jump_force;
  float theta_ff;
  int chassis_speed_buff;
  uint16_t max_power;
  Chassis_Mode_e chassis_mode;
  uint8_t SuperCapBoost;
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
  IMU_Init_Config_s imu_init_config;
  SuperCap_Init_Config_s super_cap_config;
} Chassis_Init_Config_s;

/**
 * @brief 功率控制结构体（无超级电容版本）
 *
 * 控制逻辑：
 *   1. 用6参数模型实时估计总功率 P_est（低通滤波后得 P_filtered）
 *   2. 开环基准：vel_max_base = Kp_vel * sqrt(P_limit)
 *   3. PI闭环：根据 (P_limit - P_filtered) 修正 vel_max
 *   4. vel_max 非对称低通：超功率时快速降速，功率富余时缓慢恢复
 *   5. 对目标速度做幅值限制 + 斜率限制，输出 limited_vx
 */
typedef struct {
  // ===== 6参数功率模型系数 =====
  float wheel_k[6];
  float P_wheel_L;
  float P_wheel_R;
  float P_total_pre;
  float P_real;
  float P_limit;

  float Kp_vel;       // s_d_max = Kp_vel * sqrt(P_ref)
  float vx_ramp;      // 斜率限制后的当前速度指令
  float vx_ramp_acc;  // 加速斜率 (m/s²)
  float vx_ramp_dec;  // 减速斜率 (m/s²
} PowerCtrl_t;

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

  // ===== 功率控制 =====
  PowerCtrl_t power_ctrl;
  float limited_vx;  // 经过功率限制后输入LQR的目标速度 (m/s)

  float delta_theta_comp;

  float LQR_K[4][10];  // [4输出][10状态变量]

  uint32_t DWT_CNT;
  float dt;
  struct {
    uint8_t is_first_update : 1;    // 观测器和状态变量是否完成第一次更新
    uint8_t is_restart : 1;         // 是否需要重启更新（如时间步长过大时）
    uint8_t is_controlled : 1;      // 是否处于受控状态, 1表示受控（如有前进指令时）, 0表示非受控, 用于切换控制策略
    uint8_t is_recovered : 1;       // 本次倒地自起是否已完成，pitch<阈值后置1并退出 recovery，未失控时由上层清零
  } update_flag;

  SuperCapInstance* super_cap;
  SuperCapMode super_cap_mode;
} ChassisInstance;

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);
void ChassisTask(void);
