/**
 ******************************************************************************
 * @file    parallel_leg.c
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Parallel-Leg Module
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "parallel_leg.h"

#include <math.h>

#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"

// todo: LegInstance应当以static形式在内部保存指针，内部态函数调用内部实例进行数据操作
// robot param
static float rod_length[5];
static float joint_motor_zero_offset[2];
static float wheel_radius;
static float wheel_reduction_ratio;
static float LQR_K_Coefficient[2][6][4];
static float MPC_K_Coefficient[2][6][4];
// intermediate variables
static float A0, B0, C0;
static float A1;

static float state_err[6];
static float state_delta[6];

static float T_LQR;
static float Tp_LQR;
static float Tp_MPC;

/**
 * @brief   更新腿部机构VMC真实模型参数
 * @param   leg 指向腿部实例的指针
 * @retval  无
 * @note    计算各个关节坐标、中间变量和关节角度phi2
 */
static void RealModelUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  // Get motor angle
  rm->phi1 = joint_motor_zero_offset[0] + leg->joint_motor[0]->measure.position;
  rm->phi4 = joint_motor_zero_offset[1] + leg->joint_motor[1]->measure.position;
  rm->phi1_d = leg->joint_motor[0]->measure.velocity;
  rm->phi4_d = leg->joint_motor[1]->measure.velocity;

  // Calculate joint B\D coordinates
  rm->xb = rod_length[0] * mcos(rm->phi1);
  rm->yb = rod_length[0] * msin(rm->phi1);
  rm->xd = rod_length[4] + rod_length[3] * mcos(rm->phi4);
  rm->yd = rod_length[3] * msin(rm->phi4);

  // Calculate intermediate variables
  A0 = 2.0f * rod_length[1] * (rm->xd - rm->xb);
  B0 = 2.0f * rod_length[1] * (rm->yd - rm->yb);
  C0 = rod_length[1] * rod_length[1] + (rm->xb - rm->xd) * (rm->xb - rm->xd) + (rm->yb - rm->yd) * (rm->yb - rm->yd) -
       rod_length[2] * rod_length[2];

  // Calculate joint angle phi2
  rm->phi2 = 2 * atan2f((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
  rm->phi3 = atan2f(rm->yb - rm->yd + rod_length[1] * msin(rm->phi2), rm->xb - rm->xd + rod_length[1] * mcos(rm->phi2));
  // Calculate C coordinates
  rm->xc = rm->xb + rod_length[1] * mcos(rm->phi2);
  rm->yc = rm->yb + rod_length[1] * msin(rm->phi2);
}

/**
 * @brief   正运动学变换，更新腿部机构VMC虚拟模型参数
 * @param   leg 指向腿部实例的指针
 * @retval  无
 * @note    计算腿长、角度phi、角速度等参数
 */
static void VirtualModelUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;

  // Calculate leg length & angle & theta
  vm->length = sqrtf((rm->xc - rod_length[4] / 2.0f) * (rm->xc - rod_length[4] / 2.0f) + rm->yc * rm->yc);
  vm->phi = atan2f(rm->yc, rm->xc - rod_length[4] / 2.0f);
  vm->alpha = PI / 2.0f - vm->phi;
  vm->alpha_d = -vm->phi_d;

  // Calculate A1, xB_dot, yB_dot
  A1 = (rod_length[0] * rm->phi1_d * msin(rm->phi1 - rm->phi3) +
        rod_length[3] * rm->phi4_d * msin(rm->phi3 - rm->phi4)) /
       msin(rm->phi3 - rm->phi2);
  rm->xb_d = -rod_length[0] * rm->phi1_d * msin(rm->phi1);
  rm->yb_d = rod_length[0] * rm->phi1_d * mcos(rm->phi1);

  // Calculate length_d, phi_d
  vm->length_d =
      (rm->yc * (rm->yb_d + A1 * mcos(rm->phi2)) + (rm->xc - rod_length[4] / 2.0f) * (rm->xb_d - A1 * msin(rm->phi2))) /
      vm->length;
  vm->phi_d =
      ((rm->xc - rod_length[4] / 2.0f) * (rm->yb_d + A1 * mcos(rm->phi2)) - rm->yc * (rm->xb_d - A1 * msin(rm->phi2))) /
      (vm->length * vm->length);

  if (!leg->update_flag.is_initialized) {
    vm->last_length_d = vm->length_d;
    vm->last_phi_d = vm->phi_d;
    leg->update_flag.is_initialized = 1;
  }

  vm->phi_dd = (vm->phi_d - vm->last_phi_d) / leg->dt;
  vm->length_dd = (vm->length_d - vm->last_length_d) / leg->dt;  // TODO：得滤波&最好别微分

  vm->last_length_d = vm->length_d;
  vm->last_phi_d = vm->phi_d;
}

static float degree;

/**
 * @brief   更新腿部机构的状态变量
 * @param   leg 指向腿部实例的指针
 * @param   imu_data 指向IMU数据的指针
 * @retval  无
 * @note    计算腿部机构的状态变量theta和theta_d
 */
static void StateVarUpdate(LegInstance* leg, INS_t* imu) {
  Virtual_Model_t* vm = &leg->virtual_model;
  if (leg->update_flag.is_controlled) {
    leg->state_var.x = 0;
  } else {
    leg->state_var.x += ((leg->state_var.x_d + leg->last_state_var.x_d) / 2) * leg->dt;  // 梯形积分
  }
  leg->state_var.phi = DEGREE_2_RAD * imu->Pitch;
  leg->state_var.phi_d = imu->Gyro[0];  // Todo: IMU应当有可在上层配置的旋转矩阵
  degree += leg->state_var.phi_d * leg->dt;
  leg->state_var.theta = PI / 2.0f - vm->phi - DEGREE_2_RAD * imu->Pitch;
  leg->state_var.theta_d = -vm->phi_d - imu->Gyro[0];
  // Todo:速度观测需要用的变量alpha暂时没处理
}

void ObserverVarUpdate(LegInstance* leg, INS_t* imu) {
  Observer_Var_t* ov = &leg->observer_var;
  Virtual_Model_t* vm = &leg->virtual_model;
  State_Var_t* sv = &leg->state_var;
  ov->w = -leg->wheel_motor->measure.speed_aps / wheel_reduction_ratio * DEGREE_2_RAD + vm->alpha_d -
          imu->Gyro[0];  // todo:Gyro极性不确定
  ov->vb = ov->w * wheel_radius + vm->length * sv->theta_d * mcos(sv->theta) + vm->length_d * msin(sv->theta);
}

/**
 * @brief   根据系数计算LQR增益K值
 * @param   coe 指向系数数组的指针
 * @param   len 当前长度值
 * @retval  计算得到的K值
 * @note    使用三次多项式计算K值
 */
static float LQR_K_Calc(const float* coe, float len) {
  return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}
/**
 * @brief   根据系数计算MPC增益K值 (指数拟合)
 * @note    公式: K = a * exp(b * len) + c * exp(d * len)
 *          coe[0]=a, coe[1]=b, coe[2]=c, coe[3]=d
 */
static float MPC_K_Calc(const float* coe, float len) {
  return coe[0] * expf(coe[1] * len) + coe[2] * expf(coe[3] * len);
}

static void OffGroundDetection(LegInstance* leg) {  // TODO: 旦说串腿这个很搞
  Virtual_Model_t* vm = &leg->virtual_model;
  float current_FN =
      vm->F * arm_cos_f32(leg->state_var.theta) + vm->Tp * arm_sin_f32(leg->state_var.theta) / vm->length + 6.0f;
#define FN_FILTER_COEF 0.2f
  // 应用一阶低通滤波
  vm->FN = vm->FN * (1.0f - FN_FILTER_COEF) + current_FN * FN_FILTER_COEF;
  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  if (vm->FN < 30.0f) {
    leg->update_flag.is_off_ground = 1;  // 离地了
  } else {
    leg->update_flag.is_off_ground = 0;  // 接地了
  }
}

void JointTorqueUpdate(LegInstance* leg) {
  // 简化代码量, 空间换时间, 减少指针引用
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  // 计算雅可比矩阵元素
  leg->J[0][0] = (rod_length[0] * msin(vm->phi - rm->phi3) * msin(rm->phi1 - rm->phi2)) / msin(rm->phi3 - rm->phi2);
  leg->J[0][1] =
      (rod_length[0] * mcos(vm->phi - rm->phi3) * msin(rm->phi1 - rm->phi2)) / (vm->length * msin(rm->phi3 - rm->phi2));
  leg->J[1][0] = (rod_length[3] * msin(vm->phi - rm->phi2) * msin(rm->phi3 - rm->phi4)) / msin(rm->phi3 - rm->phi2);
  leg->J[1][1] =
      (rod_length[3] * mcos(vm->phi - rm->phi2) * msin(rm->phi3 - rm->phi4)) / (vm->length * msin(rm->phi3 - rm->phi2));
  // VMC模型腿部虚拟转矩与推力, 转换为关节电机实际输出转矩
  rm->Tp_1 = leg->J[0][0] * vm->F + leg->J[0][1] * vm->Tp;
  rm->Tp_2 = leg->J[1][0] * vm->F + leg->J[1][1] * vm->Tp;
}

// Todo: x，x_d的更新没做
/**
 * @brief 初始化腿部结构体，设置连杆长度参数并初始化相关变量
 *
 * @param config 指向初始化配置结构体的指针
 * @return 指向新创建的腿部实例的指针
 */
LegInstance* LegInit(Leg_Init_Config_s* config) {
  LegInstance* leg_instance = (LegInstance*)zmalloc(sizeof(LegInstance));
  Virtual_Model_t* vm = &leg_instance->virtual_model;
  // 初始化腿长PID todo: 双环PID能用一个PID实例表示的，写得shit
  PIDInit(&leg_instance->virtual_model.length_PID, &config->length_PID_config);
  PIDInit(&leg_instance->virtual_model.length_d_PID, &config->length_d_PID_config);
  PIDInit(&leg_instance->state_var.phi_PID, &config->phi_PID_config);
  // 初始化腿部电机
  leg_instance->joint_motor[0] = DMMotorInit(&config->joint_motor_config[0]);
  leg_instance->joint_motor[1] = DMMotorInit(&config->joint_motor_config[1]);
  leg_instance->wheel_motor = DJIMotorInit(&config->wheel_motor_config);
  // 初始化连杆长度参数
  for (int i = 0; i < 5; i++) {
    rod_length[i] = config->leg_param.rod_length[i];
  }
  // 初始化电机零点较腿部坐标系x轴偏移量
  joint_motor_zero_offset[0] = config->leg_param.joint_motor_zero_offset[0];
  joint_motor_zero_offset[1] = config->leg_param.joint_motor_zero_offset[1];
  wheel_radius = config->leg_param.wheel_radius;
  wheel_reduction_ratio = config->leg_param.wheel_reduction_ratio;
  // 初始化LQR_K矩阵拟合系数
  memcpy(LQR_K_Coefficient, config->LQR_K_Coefficient, sizeof(LQR_K_Coefficient));
  // 新增：初始化MPC_K矩阵拟合系数
  memcpy(MPC_K_Coefficient, config->MPC_K_Coefficient, sizeof(MPC_K_Coefficient));
  // 初始化导数相关变量
  vm->last_phi_d = 0.0f;
  vm->last_length_d = 0.0f;
  // 初始化各更新标志
  leg_instance->update_flag.is_initialized = 0;
  leg_instance->update_flag.is_off_ground = 0;
  leg_instance->update_flag.is_controlled = 0;
  // 初始化DWT计数器
  DWT_GetDeltaT(&leg_instance->DWT_CNT);

  if (config->leg_cali_mode == LEG_CALI_MODE) {
    DMMotorCaliEncoder(leg_instance->joint_motor[0]);
    DMMotorCaliEncoder(leg_instance->joint_motor[1]);
  }

  return leg_instance;
}

/**
 * @brief   更新腿部控制参数
 * @param   leg 指向腿部实例的指针
 * @param   imu_data 指向IMU数据的指针
 * @retval  无
 * @note    包括更新真实模型、虚拟模型、状态变量，计算LQR增益和控制力矩
 */
void LegCtrlUpdate(LegInstance* leg, INS_t* imu) {
  // 获取时间间隔，用于状态量计算
  leg->dt = DWT_GetDeltaT(&leg->DWT_CNT);
  // 五连杆物理建模参数更新
  RealModelUpdate(leg);
  // VMC简化模型参数更新
  VirtualModelUpdate(leg);
  // 状态变量更新
  StateVarUpdate(leg, imu);
  // 根据腿长计算LQR_K矩阵, i->腿编号, j->状态变量编号
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      // 计算 LQR 增益 (多项式)
      leg->LQR_K[i][j] = LQR_K_Calc(&LQR_K_Coefficient[i][j][0], leg->virtual_model.length);
      // 计算 MPC 增益 (指数)
      leg->MPC_K[i][j] = MPC_K_Calc(&MPC_K_Coefficient[i][j][0], leg->virtual_model.length);
    }
  }

  OffGroundDetection(leg);

  leg->leg_ctrl_cmd.x_ref = 0;
  // 状态变量矩阵与LQR_K矩阵相乘得到控制力矩, T为轮毂电机转矩，Tp为VMC模型髋关节电机转矩

  state_err[0] = leg->state_var.theta - 0.0f;
  state_err[1] = leg->state_var.theta_d - 0.0f;
  state_err[2] = leg->state_var.x - leg->leg_ctrl_cmd.x_ref;
  state_err[3] = leg->state_var.x_d - leg->leg_ctrl_cmd.x_d_ref;
  state_err[4] = leg->state_var.phi - 0.0f;
  state_err[5] = leg->state_var.phi_d - 0.0f;
  // 计算状态增量 (MPC用: Delta_x = Current - Last)
  state_delta[0] = leg->state_var.theta - leg->last_state_var.theta;
  state_delta[1] = leg->state_var.theta_d - leg->last_state_var.theta_d;
  state_delta[2] = leg->state_var.x - leg->last_state_var.x;
  state_delta[3] = leg->state_var.x_d - leg->last_state_var.x_d;
  state_delta[4] = leg->state_var.phi - leg->last_state_var.phi;
  state_delta[5] = leg->state_var.phi_d - leg->last_state_var.phi_d;
  // 计算控制量
  T_LQR = 0.0f;
  Tp_LQR = 0.0f;
  Tp_MPC = 0.0f;
  // --- 轮子力矩 (T) 计算 ---
  if (!leg->update_flag.is_off_ground) {
    for (int k = 0; k < 6; k++) {
      // 运动时x项不参与计算
      state_err[2] *= (float)(!leg->update_flag.is_controlled);
      T_LQR += leg->LQR_K[0][k] * state_err[k];
      leg->real_model.T = T_LQR;
    }
  } else {
    leg->real_model.T = 0;  // 串联减去增量补偿
  }
  // --- 关节力矩 (Tp) 计算 ---
  if (!leg->update_flag.is_off_ground) {
    for (int k = 0; k < 6; k++) {
      state_err[2] *= (float)(!leg->update_flag.is_controlled);
      state_delta[2] *= (float)(!leg->update_flag.is_controlled);
      Tp_LQR += leg->LQR_K[1][k] * state_err[k];
      Tp_MPC += leg->MPC_K[1][k] * state_delta[k];
    }
  } else {
    for (int k = 0; k < 2; k++) {
      Tp_LQR += leg->LQR_K[1][k] * state_err[k];
      Tp_MPC += leg->MPC_K[1][k] * state_delta[k];
    }
  }
  leg->virtual_model.Tp = Tp_LQR - Tp_MPC;
  // leg->virtual_model.Tp = Tp_LQR;

  // 腿长双环PID
  // leg->leg_ctrl_cmd.length_d_ref =
  // PIDCalculate(&leg->virtual_model.length_PID, leg->virtual_model.length, leg->leg_ctrl_cmd.length_ref);
  // leg->virtual_model.F =
  //     PIDCalculate(&leg->virtual_model.length_d_PID, leg->virtual_model.length_d, leg->leg_ctrl_cmd.length_d_ref);

  leg->last_state_var = leg->state_var;

  leg->virtual_model.F =
      leg->leg_ctrl_cmd.F_ref +
      PIDCalculate(&leg->virtual_model.length_PID, leg->virtual_model.length, leg->leg_ctrl_cmd.length_ref);
}
