/**
 ******************************************************************************
 * @file    parallel_leg.c
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "parallel_leg.h"
#include "user_lib.h"
#include "bsp_dwt.h"

const float LQR_K_R[2][6] = {
    {-2.1954f, -0.2044f, -0.8826f, -1.3245f, 1.2784f, 0.1112f},
    {2.5538f, 0.2718f, 1.5728f, 2.2893f, 12.1973f, 0.4578f}};

const float LQR_K_Coefficient[2][6][4] = {
    {{-88.3079710751263f, 68.9068310796955f, -30.0003802287502f, -0.197774178106864f},
     {1.52414598059982f, -1.09343038036609f, -2.82688593867512f, 0.0281973842051861f},
     {-21.8700750609220f, 12.7421672466682f, -2.58779676995074f, -0.750848242540331f},
     {-29.3271263750692f, 17.6067629457167f, -4.23484645974363f, -1.08976980288501f},
     {-147.771748892911f, 94.0665615939814f, -22.5139626085997f, 2.53224765312440f},
     {-6.72857056332562f, 4.46216499907277f, -1.14328671767927f, 0.176775242328476f},},
    {{-43.1495035855057f, 35.1427890165576f, -12.7617044245710f, 3.36940801739176f},
     {4.14428184617563f, -2.56933858132474f, 0.479050092243477f, 0.248175261724735f},
     {-229.898177881547f, 144.949258291255f, -33.9196587052128f, 3.44291788865558f},
     {-329.509693153293f, 207.219295206736f, -48.3799707459102f, 4.952560575479143f},
     {380.589246401548f, -223.660017597103f, 46.1696952431268f, 9.82308882692083f},
     {26.1010681824798f, -15.7241310513153f, 3.39175554658673f, 0.278568898146322f}}};

/**
 * @brief   更新腿部机构VMC真实模型参数
 * @param   leg 指向腿部实例的指针
 * @retval  无
 * @note    计算各个关节坐标、中间变量和关节角度phi2
 */
static void RealModelUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  // Get motor angle
  rm->phi1 = PHI1_OFFSET + leg->joint_motor[0]->measure.position;
  rm->phi4 = PHI2_OFFSET + leg->joint_motor[1]->measure.position;
  rm->phi1_d = leg->joint_motor[0]->measure.velocity;
  rm->phi4_d = leg->joint_motor[1]->measure.velocity;

  // Calculate joint B\D coordinates
  rm->xb = rm->l1 * mcos(rm->phi1);
  rm->yb = rm->l1 * msin(rm->phi1);
  rm->xd = rm->l5 + rm->l4 * mcos(rm->phi4);
  rm->yd = rm->l4 * msin(rm->phi4);

  // Calculate intermediate variables
  rm->A0 = 2.0f * rm->l2 * (rm->xd - rm->xb);
  rm->B0 = 2.0f * rm->l2 * (rm->yd - rm->yb);
  rm->C0 = rm->l2 * rm->l2 + (rm->xb - rm->xd) * (rm->xb - rm->xd) +
           (rm->yb - rm->yd) * (rm->yb - rm->yd) - rm->l3 * rm->l3;

  // Calculate joint angle phi2
  rm->phi2 = 2 * atan2f((rm->B0 + sqrtf(rm->A0 * rm->A0 + rm->B0 * rm->B0 - rm->C0 * rm->C0)), (rm->A0 + rm->C0));
  rm->phi3 = atan2f(rm->yb - rm->yd + rm->l2 * msin(rm->phi2),
                    rm->xb - rm->xd + rm->l2 * mcos(rm->phi2));
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
  vm->A1 = (rm->l1 * rm->phi1_d * msin(rm->phi1 - rm->phi3) + rm->l4 * rm->phi4_d * msin(rm->phi3 - rm->phi4)) / msin(rm->phi3 - rm->phi2);
  rm->xb_d = -rm->l1 * rm->phi1_d * msin(rm->phi1);
  rm->yb_d = rm->l1 * rm->phi1_d * mcos(rm->phi1);

  // Calculate length_d, phi_d
  vm->length_d = (rm->yc * (rm->yb_d + vm->A1 * mcos(rm->phi2)) + (rm->xc - rm->l5 / 2.0f) * (rm->xb_d - vm->A1 * msin(rm->phi2))) / vm->length;
  vm->phi_d = ((rm->xc - rm->l5 / 2.0f) * (rm->yb_d + vm->A1 * mcos(rm->phi2)) - rm->yc * (rm->xb_d - vm->A1 * msin(rm->phi2))) / (vm->length * vm->length);

  if (!leg->update_flag.is_initialized) {
    vm->last_length_d = vm->length_d;
    vm->last_phi_d = vm->phi_d;
    leg->update_flag.is_initialized = 1;
  }

  vm->phi_dd = (vm->phi_d - vm->last_phi_d) / leg->dt;
  vm->length_dd = (vm->length_d - vm->last_length_d) / leg->dt; // TODO：得滤波&最好别微分

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
  leg->state_var.x_d = leg->wheel_motor->measure.velocity; //Todo: 直接读电机转速还是微分？
  leg->state_var.x = leg->state_var.x + ((leg->state_var.x_d + last_x_d) / 2) * leg->dt; // 梯形积分
  leg->state_var.phi = imu_data->Pitch;
  leg->state_var.phi_d = imu_data->Gyro[0]; //Todo: 不确定是不是Gyro[1];
  leg->state_var.theta = PI / 2.0f - vm->phi - imu_data->Pitch;
  leg->state_var.theta_d = -vm->phi_d - imu_data->Gyro[0]; //Todo: 不确定是不是Gyro[1]
  // Todo:速度观测有个变量alpha暂时没处理
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
//   vm->FN = vm->F * arm_cos_f32(leg->state_var.theta) + vm->Tp * arm_sin_f32(leg->state_var.theta) / vm->length + 6.0f;
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

  PIDInit(&leg_instance->virtual_model.length_PID, &config->length_PID_config);
  PIDInit(&leg_instance->virtual_model.length_d_PID, &config->length_d_PID_config);

  leg_instance->joint_motor[0] = DMMotorInit(&config->joint_motor_config[0]);
  leg_instance->joint_motor[1] = DMMotorInit(&config->joint_motor_config[1]);
  leg_instance->wheel_motor = DMMotorInit(&config->wheel_motor_config);

  // 初始化连杆长度参数
  rm->l1 = CONNECTING_ROD_L1;
  rm->l2 = CONNECTING_ROD_L2;
  rm->l3 = CONNECTING_ROD_L3;
  rm->l4 = CONNECTING_ROD_L4;
  rm->l5 = CONNECTING_ROD_L5;

  // 初始化导数相关变量
  vm->last_phi_d = 0.0f;
  vm->last_length_d = 0.0f;
  leg_instance->update_flag.is_initialized = 0;
  leg_instance->update_flag.is_grounded = 1;
  DWT_GetDeltaT(&leg_instance->DWT_CNT);
  return leg_instance;
}

/**
 * @brief   更新腿部控制参数
 * @param   leg 指向腿部实例的指针
 * @param   imu_data 指向IMU数据的指针
 * @retval  无
 * @note    包括更新真实模型、虚拟模型、状态变量，计算LQR增益和控制力矩
 */
void LegControlUpdate(LegInstance* leg, const attitude_t* imu_data) {
  leg->dt = DWT_GetDeltaT(&leg->DWT_CNT);
  RealModelUpdate(leg);
  VirtualModelUpdate(leg);
  StateVarUpdate(leg, imu_data);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      leg->LQR_K[i][j] = LQR_K_Calc(&LQR_K_Coefficient[i][j][0], leg->virtual_model.length);
    }
  }
  leg->real_model.T = (leg->LQR_K[0][0] * leg->state_var.theta +
                       leg->LQR_K[0][1] * leg->state_var.theta_d +
                       leg->LQR_K[0][2] * leg->state_var.x +
                       leg->LQR_K[0][3] * leg->state_var.x_d +
                       leg->LQR_K[0][4] * leg->state_var.phi +
                       leg->LQR_K[0][5] * leg->state_var.phi_d);
  leg->virtual_model.Tp = (leg->LQR_K[1][0] * leg->state_var.theta +
                           leg->LQR_K[1][1] * leg->state_var.theta_d +
                           leg->LQR_K[1][2] * leg->state_var.x +
                           leg->LQR_K[1][3] * leg->state_var.x_d +
                           leg->LQR_K[1][4] * leg->state_var.phi +
                           leg->LQR_K[1][5] * leg->state_var.phi_d);
  // 腿长双环PID

  leg->leg_ctrl_cmd.length_d_ref = PIDCalculate(&leg->virtual_model.length_PID, leg->virtual_model.length,
                                                 leg->leg_ctrl_cmd.length_ref);
  leg->virtual_model.F = PIDCalculate(&leg->virtual_model.length_d_PID, leg->virtual_model.length_d,
                                      leg->leg_ctrl_cmd.length_d_ref);
}

void JointTorqueUpdate(LegInstance* leg) {
  Real_Model_t* rm = &leg->real_model;
  Virtual_Model_t* vm = &leg->virtual_model;

  // 计算雅可比矩阵元素
  leg->J[0][0] = (rm->l1 * msin(vm->phi - rm->phi4) * msin(rm->phi1 - rm->phi2)) / msin(
                     rm->phi4 - rm->phi2);
  leg->J[0][1] = (rm->l1 * mcos(vm->phi - rm->phi4) * msin(rm->phi1 - rm->phi2)) / (
                   vm->length * msin(rm->phi4 - rm->phi2));
  leg->J[1][0] = (rm->l4 * msin(vm->phi - rm->phi2) * msin(rm->phi4 - rm->phi3)) / msin(
                     rm->phi4 - rm->phi2);
  leg->J[1][1] = (rm->l4 * mcos(vm->phi - rm->phi2) * msin(rm->phi4 - rm->phi3)) / (
                   vm->length * msin(rm->phi4 - rm->phi2));

  rm->Tp_1 = leg->J[0][0] * vm->F + leg->J[0][1] * vm->Tp;
  rm->Tp_2 = leg->J[1][0] * vm->F + leg->J[1][1] * vm->Tp;
}

// Todo: 雅可比和x，x_d的更新没做
