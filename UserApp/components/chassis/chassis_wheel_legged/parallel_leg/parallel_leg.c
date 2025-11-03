/**
 ******************************************************************************
 * @file    parallel_leg.c
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "parallel_leg.h"

#include "bsp_dwt.h"
#include "user_lib.h"

// todo: LegInstance应当以static形式在内部保存指针，内部态函数调用内部实例进行数据操作

// static float LQR_K[2][6];
// = {{-2.1954f, -0.2044f, -0.8826f, -1.3245f, 1.2784f, 0.1112f},
// {2.5538f, 0.2718f, 1.5728f, 2.2893f, 12.1973f, 0.4578f}};

// robot param
static float joint_motor_zero_offset[2];
// intermediate variables
static float LQR_K_Coefficient[2][6][4];
static float A0, B0, C0;
static float A1;

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
  rm->xb = rm->l1 * mcos(rm->phi1);
  rm->yb = rm->l1 * msin(rm->phi1);
  rm->xd = rm->l5 + rm->l4 * mcos(rm->phi4);
  rm->yd = rm->l4 * msin(rm->phi4);

  // Calculate intermediate variables
  A0 = 2.0f * rm->l2 * (rm->xd - rm->xb);
  B0 = 2.0f * rm->l2 * (rm->yd - rm->yb);
  C0 =
      rm->l2 * rm->l2 + (rm->xb - rm->xd) * (rm->xb - rm->xd) + (rm->yb - rm->yd) * (rm->yb - rm->yd) - rm->l3 * rm->l3;

  // Calculate joint angle phi2
  rm->phi2 = 2 * atan2f((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
  rm->phi3 = atan2f(rm->yb - rm->yd + rm->l2 * msin(rm->phi2), rm->xb - rm->xd + rm->l2 * mcos(rm->phi2));
  // Calculate C coordinates
  rm->xc = rm->xb + rm->l2 * mcos(rm->phi2);
  rm->yc = rm->yb + rm->l2 * msin(rm->phi2);
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
  vm->length = sqrtf((rm->xc - rm->l5 / 2.0f) * (rm->xc - rm->l5 / 2.0f) + rm->yc * rm->yc);
  vm->phi = atan2f(rm->yc, rm->xc - rm->l5 / 2.0f);

  // Calculate A1, xB_dot, yB_dot
  A1 = (rm->l1 * rm->phi1_d * msin(rm->phi1 - rm->phi3) + rm->l4 * rm->phi4_d * msin(rm->phi3 - rm->phi4)) /
       msin(rm->phi3 - rm->phi2);
  rm->xb_d = -rm->l1 * rm->phi1_d * msin(rm->phi1);
  rm->yb_d = rm->l1 * rm->phi1_d * mcos(rm->phi1);

  // Calculate length_d, phi_d
  vm->length_d =
      (rm->yc * (rm->yb_d + A1 * mcos(rm->phi2)) + (rm->xc - rm->l5 / 2.0f) * (rm->xb_d - A1 * msin(rm->phi2))) /
      vm->length;
  vm->phi_d =
      ((rm->xc - rm->l5 / 2.0f) * (rm->yb_d + A1 * mcos(rm->phi2)) - rm->yc * (rm->xb_d - A1 * msin(rm->phi2))) /
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

/**
 * @brief   更新腿部机构的状态变量
 * @param   leg 指向腿部实例的指针
 * @param   imu_data 指向IMU数据的指针
 * @retval  无
 * @note    计算腿部机构的状态变量theta和theta_d
 */
static void StateVarUpdate(LegInstance* leg, const attitude_t* imu_data) {
  Virtual_Model_t* vm = &leg->virtual_model;
  float last_x_d = leg->state_var.x_d;
  leg->state_var.x_d = leg->wheel_motor->measure.velocity;
  leg->state_var.x += ((leg->state_var.x_d + last_x_d) / 2) * leg->dt;  // 梯形积分
  leg->state_var.phi = imu_data->Pitch;
  leg->state_var.phi_d = imu_data->Gyro[0];  // Todo: IMU应当有可在上层配置的旋转矩阵
  leg->state_var.theta = PI / 2.0f - vm->phi - imu_data->Pitch;
  leg->state_var.theta_d = -vm->phi_d - imu_data->Gyro[0];
  // Todo:速度观测需要用的变量alpha暂时没处理
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

// static void OffGroundDetection(LegInstance* leg) {
//   VirtualModel_t* vm = &leg->virtual_model;
//   vm->FN = vm->F * arm_cos_f32(leg->state_var.theta) + vm->Tp * arm_sin_f32(leg->state_var.theta) / vm->length
//   + 6.0f;
//   //腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
//   if (vm->FN < 5.0f) {
//     leg->update_flag.is_grounded = 0; //离地了
//   } else {
//     leg->update_flag.is_grounded = 1; //接地了
//   }
// }

/**
 * @brief 初始化腿部结构体，设置连杆长度参数并初始化相关变量
 *
 * @param config 指向初始化配置结构体的指针
 * @return 指向新创建的腿部实例的指针
 */
LegInstance* LegInit(Leg_Init_Config_s* config) {
  LegInstance* leg_instance = (LegInstance*)zmalloc(sizeof(LegInstance));
  Real_Model_t* rm = &leg_instance->real_model;
  Virtual_Model_t* vm = &leg_instance->virtual_model;
  // 初始化腿长PID todo: 双环PID能用一个PID实例表示的，写得shit
  PIDInit(&leg_instance->virtual_model.length_PID, &config->length_PID_config);
  PIDInit(&leg_instance->virtual_model.length_d_PID, &config->length_d_PID_config);
  // 初始化腿部电机
  leg_instance->joint_motor[0] = DMMotorInit(&config->joint_motor_config[0]);
  leg_instance->joint_motor[1] = DMMotorInit(&config->joint_motor_config[1]);
  leg_instance->wheel_motor = DMMotorInit(&config->wheel_motor_config);
  // 初始化连杆长度参数
  rm->l1 = config->leg_param.rod_length[0];
  rm->l2 = config->leg_param.rod_length[1];
  rm->l3 = config->leg_param.rod_length[2];
  rm->l4 = config->leg_param.rod_length[3];
  rm->l5 = config->leg_param.rod_length[4];
  // 初始化电机零点较腿部坐标系x轴偏移量
  joint_motor_zero_offset[0] = config->leg_param.joint_motor_zero_offset[0];
  joint_motor_zero_offset[1] = config->leg_param.joint_motor_zero_offset[1];
  // 初始化LQR_K矩阵拟合系数
  memcpy(LQR_K_Coefficient, config->LQR_K_Coefficient, sizeof(LQR_K_Coefficient));
  // 初始化导数相关变量
  vm->last_phi_d = 0.0f;
  vm->last_length_d = 0.0f;
  // 初始化各更新标志
  leg_instance->update_flag.is_initialized = 0;
  leg_instance->update_flag.is_grounded = 1;
  // 初始化DWT计数器
  DWT_GetDeltaT(&leg_instance->DWT_CNT);

  if (config->leg_mode == LEG_CALI_MODE) {
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
void LegCtrlUpdate(LegInstance* leg, const attitude_t* imu_data) {
  // 获取时间间隔，用于状态量计算
  leg->dt = DWT_GetDeltaT(&leg->DWT_CNT);
  // 五连杆物理建模参数更新
  RealModelUpdate(leg);
  // VMC简化模型参数更新
  VirtualModelUpdate(leg);
  // 状态变量更新
  StateVarUpdate(leg, imu_data);
  // 根据腿长计算LQR_K矩阵
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      leg->LQR_K[i][j] = LQR_K_Calc(&LQR_K_Coefficient[i][j][0], leg->virtual_model.length);
    }
  }
  // 状态变量矩阵与LQR_K矩阵相乘得到控制力矩, T为轮毂电机转矩，Tp为VMC模型髋关节电机转矩
  leg->real_model.T = (leg->LQR_K[0][0] * leg->state_var.theta + leg->LQR_K[0][1] * leg->state_var.theta_d +
                       leg->LQR_K[0][2] * leg->state_var.x + leg->LQR_K[0][3] * leg->state_var.x_d +
                       leg->LQR_K[0][4] * leg->state_var.phi + leg->LQR_K[0][5] * leg->state_var.phi_d);
  leg->virtual_model.Tp = (leg->LQR_K[1][0] * leg->state_var.theta + leg->LQR_K[1][1] * leg->state_var.theta_d +
                           leg->LQR_K[1][2] * leg->state_var.x + leg->LQR_K[1][3] * leg->state_var.x_d +
                           leg->LQR_K[1][4] * leg->state_var.phi + leg->LQR_K[1][5] * leg->state_var.phi_d);
  // 腿长双环PID
  leg->leg_ctrl_cmd.length_d_ref =
      PIDCalculate(&leg->virtual_model.length_PID, leg->virtual_model.length, leg->leg_ctrl_cmd.length_ref);
  leg->virtual_model.F =
      PIDCalculate(&leg->virtual_model.length_d_PID, leg->virtual_model.length_d, leg->leg_ctrl_cmd.length_d_ref);
}

void JointTorqueUpdate(LegInstance* leg) {
  // 简化代码量, 空间换时间, 减少指针引用
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;
  // 计算雅可比矩阵元素
  leg->J[0][0] = (rm->l1 * msin(vm->phi - rm->phi4) * msin(rm->phi1 - rm->phi2)) / msin(rm->phi4 - rm->phi2);
  leg->J[0][1] =
      (rm->l1 * mcos(vm->phi - rm->phi4) * msin(rm->phi1 - rm->phi2)) / (vm->length * msin(rm->phi4 - rm->phi2));
  leg->J[1][0] = (rm->l4 * msin(vm->phi - rm->phi2) * msin(rm->phi4 - rm->phi3)) / msin(rm->phi4 - rm->phi2);
  leg->J[1][1] =
      (rm->l4 * mcos(vm->phi - rm->phi2) * msin(rm->phi4 - rm->phi3)) / (vm->length * msin(rm->phi4 - rm->phi2));
  // VMC模型腿部虚拟转矩与推力, 转换为关节电机实际输出转矩
  rm->Tp_1 = leg->J[0][0] * vm->F + leg->J[0][1] * vm->Tp;
  rm->Tp_2 = leg->J[1][0] * vm->F + leg->J[1][1] * vm->Tp;
}

// Todo: x，x_d的更新没做
