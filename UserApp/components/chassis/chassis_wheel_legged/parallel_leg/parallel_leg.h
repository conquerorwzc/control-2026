/**
 ******************************************************************************
 * @file    parallel_leg.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @brief   None
 ******************************************************************************
 * @attention
 *     T2         T1
 *  joint[1] l5 joint[0]    LEFT       RIGHT
 *    phi4 ☉---☉  phi1       ☉----------☉
 *        /     \             |          |
 *    l4 /       \ l1         |          |
 *      /         \           |          |
 * phi3◉           ◉phi2      ◉          ◉
 *      \         /           |          |
 *    L3 \  ___  / L2         |          |
 *        /     \            ---        ---
 *       |   ◉   |           | |        | |
 *        \ ___ /            ---        ---
 ******************************************************************************
 */
#ifndef CHASSIS_CAL_H
#define CHASSIS_CAL_H
#include <stdint.h>

#include "controller.h"
#include "dmmotor.h"
#include "ins_task.h"

typedef struct {
  // Connecting rods length
  float l1, l2, l3, l4, l5;
  // Joint coordinates
  float xb, yb;
  float xb_d, yb_d;
  float xc, yc;
  float xd, yd;
  // Joint angles
  float phi2, phi3;
  float phi1, phi4;
  // Joint angle velocities
  float phi1_d, phi4_d;
  // Joint torque
  float Tp_1, Tp_2;
  // Wheel torque
  float T;
} Real_Model_t;

typedef struct {
  PIDInstance length_PID;  // PID + FeedForward
  PIDInstance length_d_PID;
  // Leg position
  float length, length_d, length_dd, last_length_d;
  float phi, phi_d, phi_dd, last_phi_d;
  // Leg force & torque
  float F, FN;
  float Tp;
} Virtual_Model_t;

typedef struct {
  float x, x_d;
  float theta, theta_d;
  float phi, phi_d;
} State_Var_t;

typedef struct {
  float x_ref, x_d_ref;
  float length_ref, length_d_ref;
} Leg_Ctrl_Cmd_t;

typedef struct {
  float rod_length[5];
  float joint_motor_zero_offset[2];
} Leg_Param_s;

typedef struct {
  Leg_Param_s leg_param;
  float LQR_K_Coefficient[2][6][4];  // [2腿][6状态变量][4多项式系数]
  PID_Init_Config_s length_PID_config;
  PID_Init_Config_s length_d_PID_config;
  Motor_Init_Config_s joint_motor_config[2];
  Motor_Init_Config_s wheel_motor_config;
} Leg_Init_Config_s;

typedef struct {
  DMMotorInstance* joint_motor[2];
  DMMotorInstance* wheel_motor;

  Real_Model_t real_model;
  Virtual_Model_t virtual_model;
  State_Var_t state_var;

  float J[2][2];
  float LQR_K[2][6];
  uint32_t DWT_CNT;
  float dt;

  Leg_Ctrl_Cmd_t leg_ctrl_cmd;

  struct {
    uint8_t is_initialized : 1;
    uint8_t is_grounded : 1;
  } update_flag;
} LegInstance;

LegInstance* LegInit(Leg_Init_Config_s* config);

void LegCtrlUpdate(LegInstance* leg, const attitude_t* imu_data);

void JointTorqueUpdate(LegInstance* leg);
#endif  // CHASSIS_CAL_H