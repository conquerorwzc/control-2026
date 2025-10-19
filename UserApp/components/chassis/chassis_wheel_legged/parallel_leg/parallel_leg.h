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
#include "dmmotor.h"
#include "ins_task.h"
#include "controller.h"

#define CONNECTING_ROD_L1 0.0725 //U: M 0.0725
#define CONNECTING_ROD_L2 0.14 //U: M
#define CONNECTING_ROD_L3 0.14 //U: M
#define CONNECTING_ROD_L4 0.0725 //U: M
#define CONNECTING_ROD_L5 0.079 //U: M, length AE

//下限位零点
#define PHI1_OFFSET 56.13 * PI / 180.0f
#define PHI2_OFFSET PHI1_OFFSET + PI / 2

typedef struct {
  // Connecting rods length
  float l1, l2, l3, l4, l5;
  // Joint coordinates
  float xb, yb;
  float xb_d, yb_d;
  float xc, yc;
  float xd, yd;
  // Intermediate variables
  float A0, B0, C0;
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
  PIDInstance length_PID; // PID + FeedForward
  PIDInstance length_d_PID;
  // Intermediate variables
  float A1;
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

  float J [2][2];
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

void LegControlUpdate(LegInstance* leg, const attitude_t* imu_data);

void JointTorqueUpdate(LegInstance* leg);
#endif //CHASSIS_CAL_H