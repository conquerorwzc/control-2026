/**
 * @file    chassis.c
 * @author  Enhao Zhang
 * @date    2026/2
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   SJTU Full-Chassis Wheel-Legged Module
 * @todo    unify motor control and value limit to fully decouple this module (also probably for imu)
 */
#include "chassis.h"

#include <math.h>
#include <string.h>

#include "general_def.h"
#include "parallel_leg.h"
#include "speed_observer.h"
#include "user_lib.h"

static ChassisInstance* chassis;
static LegInstance* leg[2];
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;

/**
 * @brief  计算LQR增益矩阵K
 *
 * 根据左腿长度l_l和右腿长度l_r，通过二维二次多项式插值系数coef，计算每个K[i][j]的值。
 * K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
 *
 * @param[out] K      计算得到的LQR增益矩阵，大小为4x10
 * @param[in]  coef   二次多项式系数数组，每个元素为6个系数，长度共40（4 * 10）
 * @param[in]  l_l    左腿长度
 * @param[in]  l_r    右腿长度
 */
static void LQR_K_Calc(float K[4][10], const float coef[40][6], float l_l, float l_r) {
  for (int n = 0; n < 40; n++) {
    int row = n / 10;
    int col = n % 10;
    K[row][col] = coef[n][0] + coef[n][1] * l_l + coef[n][2] * l_r + coef[n][3] * l_l * l_l + coef[n][4] * l_l * l_r +
                  coef[n][5] * l_r * l_r;
  }
  // float K_static[4][10] = {
  //     {-10.215290f, -8.848348f, -21.428168f, -4.233207f, 5.299104f, 0.213924f, -23.529793f, -2.118162f, 41.806390f,
  //      3.236183f},  // T_r_to_b
  //     {-10.215290f, -8.848348f, 21.428168f, 4.233207f, -23.529793f, -2.118162f, 5.299104f, 0.213924f, 41.806390f,
  //      3.236183f},  // T_l_to_b
  //     {3.816384f, 3.495197f, -5.392899f, -0.783657f, 4.301872f, 0.133936f, 8.740520f, 0.828690f, 11.190289f,
  //      1.421862f},  // T_wr_to_r
  //     {3.816384f, 3.495197f, 5.392899f, 0.783657f, 8.740520f, 0.828690f, 4.301872f, 0.133936f, 11.190289f, 1.421862f}
  //     // T_wl_to_l
  // };
  // for (int i = 0; i < 4; i++) {
  //   for (int j = 0; j < 10; j++) {
  //     K[i][j] = K_static[i][j];
  //   }
  // }
}

static void SpeedEstimate(void) {
  float chassis_aver_v;
  ObserverVarUpdate(leg[0], chassis->imu);
  ObserverVarUpdate(leg[1], chassis->imu);
  chassis_aver_v = (leg[0]->observer_var.vb + leg[1]->observer_var.vb) * 0.5f;
  xvEstimateKF_Update(&chassis->vaEstimateKF, chassis->imu->MotionAccel_n[1], chassis_aver_v);
}

/**
 * SJTU 状态与 VMC/IMU 映射（须与 MATLAB 线性化/推导文档约定一致）:
 *   phi, dphi     ← chassis_IMU->YawTotalAngle (rad), IMU Gyro[2] (yaw, rad/s)
 *   theta_b      ← IMU Pitch (rad)，俯仰角
 *   dtheta_b      ← IMU Gyro[0] (pitch 角速度, rad/s)
 *   theta_l       ← leg[1]->state_var.theta (左腿与 Z 负方向夹角，VMC 解算)
 *   theta_r       ← leg[0]->state_var.theta (右腿)
 * 若 IMU 或 VMC 的符号/零点与推导文档不一致，LQR 反馈可能反号，需在文档或测试中核对。
 */
static void StateVarUpdate(void) {
  State_Var_t* sv = &chassis->state_var;
  INS_t* imu = chassis->imu;
  if (chassis->update_flag.is_restart) {
    chassis->state_var.x_b_h = 0.0f;
    chassis->last_state_var = chassis->state_var;
  }
  /* x_b_h: no position sensor, keep 0 or integrate in observer path */
  SpeedEstimate();
  sv->v_b_h = chassis->vaEstimateKF.FilteredValue[0];
  // 位移积分
  if (chassis->update_flag.is_controlled || sv->v_b_h > 0.15f ||
      (leg[0]->update_flag.is_off_ground && leg[1]->update_flag.is_off_ground)) {
    sv->x_b_h = 0.0f;
  } else {
    sv->x_b_h += ((sv->v_b_h + chassis->last_state_var.v_b_h) / 2.0f) * chassis->dt;
  }
  sv->phi = imu->YawTotalAngle * DEGREE_2_RAD;
  sv->dphi = imu->Gyro[2];
  /* Left leg = leg[1], Right leg = leg[0] */
  sv->theta_r = leg[0]->virtual_model.theta;
  sv->dtheta_r = leg[0]->virtual_model.theta_d;
  sv->theta_l = leg[1]->virtual_model.theta;
  sv->dtheta_l = leg[1]->virtual_model.theta_d;
  sv->theta_b = -imu->Pitch * DEGREE_2_RAD;
  sv->dtheta_b = -imu->Gyro[0];

  chassis->last_state_var = *sv;
}

static void LocomotionController(void) {
  State_Var_t* sv = &chassis->state_var;
  chassis->update_flag.is_controlled = chassis_ctrl_cmd->vx != 0;

  float l_l = leg[1]->virtual_model.length;
  float l_r = leg[0]->virtual_model.length;
  LQR_K_Calc(chassis->LQR_K, chassis->param.LQR_K_Coefficients, l_l, l_r);

  // 减小phi和dphi的权重
  chassis->LQR_K[0][2] *= 0.2f;
  chassis->LQR_K[0][3] *= 0.2f;
  chassis->LQR_K[1][2] *= 0.2f;
  chassis->LQR_K[1][3] *= 0.2f;

  // 离地时只保留 Tp theta/theta_b (列4~9) 的作用，把 x_b_h/v_b_h/phi/dphi (列0~3) 置零, T置零
  for (int i = 0; i < 2; ++i) {
    if (leg[i]->update_flag.is_off_ground) {
      for (int j = 0; j < 4; ++j) {
        chassis->LQR_K[0][j] = 0.0f;
        chassis->LQR_K[1][j] = 0.0f;
      }
      for (int j = 0; j < 10; ++j) {
        chassis->LQR_K[2][j] = 0.0f;
        chassis->LQR_K[3][j] = 0.0f;
      }
    }
  }

  // TODO: 状态误差限幅
  float state_err[10];
  state_err[0] = (sv->x_b_h - 0.0f) * 1;
  state_err[1] = (sv->v_b_h - chassis_ctrl_cmd->vx) * 1;
  // 限幅 phi 到 [-pi/3, pi/3]
  state_err[2] = sv->phi - chassis_ctrl_cmd->target_yaw * 1;
  state_err[3] = sv->dphi - chassis_ctrl_cmd->wz * 1;
  state_err[4] = sv->theta_l - chassis_ctrl_cmd->theta_ff * 1;
  state_err[5] = sv->dtheta_l * 1;
  state_err[6] = sv->theta_r - chassis_ctrl_cmd->theta_ff * 1;
  state_err[7] = sv->dtheta_r * 1;
  state_err[8] = sv->theta_b * 1;
  state_err[9] = sv->dtheta_b * 1;

  /* u[0] = T_{r→b} (hip torque on body, same convention as virtual_model.Tp)
   * u[1] = T_{l→b}
   * u[2] = T_{wr→r} (reaction on leg; motor drive torque = -T_{wr→r})
   * u[3] = T_{wl→l} */
  float u[4];
  for (int i = 0; i < 4; i++) {
    u[i] = 0.0f;
    for (int j = 0; j < 10; j++) {
      u[i] -= chassis->LQR_K[i][j] * state_err[j];
    }
  }
  leg[0]->virtual_model.Tp = u[0];
  leg[1]->virtual_model.Tp = u[1];
  leg[0]->real_model.T = u[2];
  leg[1]->real_model.T = u[3];
}

/*
 * Force distribution (plan formulas 3.6–3.8):
 *   F_bl,l = +F_psi + F_l + F_gravity - F_inertial   (LEFT = leg[1])
 *   F_bl,r = -F_psi + F_l + F_gravity + F_inertial   (RIGHT = leg[0])
 * F_psi = roll_comp: positive when body rolls left → boost left support.
 * 离心力补偿符号：dphi>0 为左转，机身向外（右）倾，右腿(leg[0])需更大支撑 → leg[0] +f_inertial, leg[1]
 * -f_inertial，当前实现正确。
 */
static void LegController(void) {
  float f_psi = PIDCalculate(&chassis->roll_PID, chassis->imu->Roll * DEGREE_2_RAD, chassis_ctrl_cmd->roll);
  float l_avg = (leg[0]->virtual_model.length + leg[1]->virtual_model.length) * 0.5f;
  float f_l_r =
      PIDCalculate(&chassis->leg[0]->length_PID, leg[0]->virtual_model.length, chassis->chassis_ctrl_cmd.leg_length);
  float f_l_l =
      PIDCalculate(&chassis->leg[1]->length_PID, leg[1]->virtual_model.length, chassis->chassis_ctrl_cmd.leg_length);
  float f_gravity = 0.5f * chassis->param.body_mass * 9.81f;
  float f_inertial = 0.5f * chassis->param.body_mass * (l_avg / chassis->param.track_width) * chassis->state_var.dphi *
                     chassis->state_var.v_b_h;
  leg[0]->virtual_model.F = -f_psi + f_l_r + f_gravity + f_inertial;
  leg[1]->virtual_model.F = f_psi + f_l_l + f_gravity - f_inertial;
}

static void ChassisCtrlUpdate(void) {
  float dt_raw = DWT_GetDeltaT(&chassis->DWT_CNT);

  if (dt_raw > 0.05f) {
    chassis->update_flag.is_restart = 1;
  } else {
    chassis->dt = dt_raw;
    chassis->update_flag.is_restart = 0;
  }

  LegModelUpdate(leg[0], chassis->imu);
  LegModelUpdate(leg[1], chassis->imu);

  StateVarUpdate();
  LocomotionController();
  LegController();

  // 状态机形式处理不同跳跃状态下的腿部力分配
  switch (chassis->jump_state) {
    case JUMP_STATE_EXTEND: {
      // 起跳时给两条腿注入跳跃力
      float jump_f = chassis_ctrl_cmd->jump_force;
      leg[0]->virtual_model.F = jump_f;
      leg[1]->virtual_model.F = jump_f;
      break;
    }
    case JUMP_STATE_RETRACT: {
      // 收腿时加入前馈力以更快速收腿
      const float retract_feedforward = -250.0f;  // 可调整收腿前馈力大小
      leg[0]->virtual_model.F += retract_feedforward;
      leg[1]->virtual_model.F += retract_feedforward;
      break;
    }
    default: {
      break;
    }
  }
  VAL_LIMIT(leg[0]->virtual_model.F, -1900.0f, 1900.0f);
  VAL_LIMIT(leg[1]->virtual_model.F, -1900.0f, 1900.0f);

  chassis->delta_theta_comp =
      PIDCalculate(&chassis->delta_theta_PID, leg[0]->virtual_model.theta - leg[1]->virtual_model.theta, 0);
  for (int i = 0; i < 2; i++) {
    leg[i]->virtual_model.Tp -= (float)(1 - 2 * i) * chassis->delta_theta_comp;
  }

  JointTorqueUpdate(leg[0]);
  JointTorqueUpdate(leg[1]);
}

static void ChassisRecovery(void) {
  // 1. 清除状态并设置外环为角度环，参考值为 -0.1/0.1，准备复位
  chassis->update_flag.is_controlled = 0;
  for (int i = 0; i < 2; i++) {
    leg[i]->update_flag.is_off_ground = 0;

    DMMotorOuterLoop(leg[i]->joint_motor[0], ANGLE_LOOP);
    DMMotorOuterLoop(leg[i]->joint_motor[1], ANGLE_LOOP);

    DMMotorSetPIDRef(leg[i]->joint_motor[0], -0.1f);
    DMMotorSetPIDRef(leg[i]->joint_motor[1], 0.1f);

    leg[i]->real_model.Tp_1 = leg[i]->joint_motor[0]->motor_controller.final_output;
    leg[i]->real_model.Tp_2 = leg[i]->joint_motor[1]->motor_controller.final_output;
  }

  // 2. 判断所有关节是否均到达目标位，若到位则正常进行 ChassisCtrlUpdate
  uint8_t all_in_position = 1;
  for (int i = 0; i < 2; i++) {
    if (fabsf(leg[i]->joint_motor[0]->measure.position - (-0.1f)) > 0.5f ||
        fabsf(leg[i]->joint_motor[1]->measure.position - (0.1f)) > 0.5f) {
      all_in_position = 0;
      break;
    }
  }

  if (all_in_position) {
    ChassisCtrlUpdate();
  } else {
    // 3. 未到位则挂零轮力输出，防止不稳定
    for (int i = 0; i < 2; i++) {
      leg[i]->real_model.T = 0.0f;
    }
  }
}

static void ChassisJump(void) {
  switch (chassis->jump_state) {
    case JUMP_STATE_COMPRESS:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_min_length;
      ChassisCtrlUpdate();
      break;
    case JUMP_STATE_EXTEND:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_max_length;
      ChassisCtrlUpdate();
      if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_max_length) <= 0.05f &&
          fabsf(leg[1]->virtual_model.length - chassis->param.leg_max_length) <= 0.05f) {
        chassis->jump_state = JUMP_STATE_RETRACT;
      }
      break;
    case JUMP_STATE_RETRACT:
      chassis->chassis_ctrl_cmd.leg_length = chassis->param.leg_min_length;
      ChassisCtrlUpdate();
      if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f &&
          fabsf(leg[1]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f) {
        chassis->jump_state = JUMP_STATE_IDLE;
      }
      break;
    case JUMP_STATE_IDLE:
    default:
      ChassisCtrlUpdate();
      break;
  }
}

static void LimitChassisOutput(void) {
  for (int i = 0; i < 2; i++) {
    VAL_LIMIT(leg[i]->real_model.Tp_1, -33.0f, 33.0f);
    VAL_LIMIT(leg[i]->real_model.Tp_2, -33.0f, 33.0f);
    VAL_LIMIT(leg[i]->real_model.T, -2.45f, 2.45f);
    DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
    DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);
    // DMMotorSetRef(leg[i]->joint_motor[0], 0);
    // DMMotorSetRef(leg[i]->joint_motor[1], 0);
    if (leg[i]->update_flag.is_off_ground) {
      DJIMotorSetRef(leg[i]->wheel_motor, 0);
    } else {
      DJIMotorSetRef(leg[i]->wheel_motor, leg[i]->real_model.T * (3591.0f / 187.0f) /
                                              chassis->leg[i]->param.wheel_reduction_ratio / 0.3f * (16384.0f / 20.0f));
    }
    // DJIMotorSetRef(leg[i]->wheel_motor, 0);
  }
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

  chassis_instance->leg[0] = LegInit(&chassis_init_config->leg_init_config[0]);
  chassis_instance->leg[1] = LegInit(&chassis_init_config->leg_init_config[1]);

  PIDInit(&chassis_instance->delta_theta_PID, &chassis_init_config->delta_theta_PID_config);
  PIDInit(&chassis_instance->roll_PID, &chassis_init_config->roll_PID_config);
  PIDInit(&chassis_instance->leg[0]->length_PID, &chassis_init_config->length_PID_config);
  PIDInit(&chassis_instance->leg[1]->length_PID, &chassis_init_config->length_PID_config);
  chassis_instance->imu = INS_Init(&chassis_init_config->imu_init_config);
  xvEstimateKF_Init(&chassis_instance->vaEstimateKF);

  chassis_instance->param = chassis_init_config->param;

  chassis_instance->jump_state = JUMP_STATE_IDLE;

  chassis_instance->update_flag.is_first_update = 1;
  chassis_instance->update_flag.is_restart = 1;
  chassis_instance->update_flag.is_controlled = 0;

  chassis = chassis_instance;
  leg[0] = chassis->leg[0];
  leg[1] = chassis->leg[1];
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;

  DWT_GetDeltaT(&chassis->DWT_CNT);
  return chassis_instance;
}

void ChassisTask(void) {
  if (chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_POWER_OFF) {
    for (int i = 0; i < 2; i++) {
      DMMotorStop(chassis->leg[i]->joint_motor[0]);
      DMMotorStop(chassis->leg[i]->joint_motor[1]);
      DJIMotorStop(chassis->leg[i]->wheel_motor);
    }
    chassis->jump_state = JUMP_STATE_IDLE;
  } else {
    for (int i = 0; i < 2; i++) {
      DMMotorEnable(chassis->leg[i]->joint_motor[0]);
      DMMotorEnable(chassis->leg[i]->joint_motor[1]);
      DJIMotorEnable(chassis->leg[i]->wheel_motor);
    }
  }

  switch (chassis->chassis_ctrl_cmd.chassis_mode) {
    case CHASSIS_RECOVERY:
      ChassisRecovery();
      chassis->jump_state = JUMP_STATE_IDLE;
      break;
    case CHASSIS_ON:
      ChassisCtrlUpdate();
      chassis->jump_state = JUMP_STATE_IDLE;
      break;
    case CHASSIS_JUMP_READY:
      if (chassis->jump_state == JUMP_STATE_IDLE || chassis->jump_state == JUMP_STATE_COMPRESS) {
        chassis->jump_state = JUMP_STATE_COMPRESS;
      }
      ChassisJump();
      break;
    case CHASSIS_JUMP_START:
      if (chassis->jump_state == JUMP_STATE_COMPRESS) {
        if (fabsf(leg[0]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f &&
            fabsf(leg[1]->virtual_model.length - chassis->param.leg_min_length) <= 0.05f) {
          chassis->jump_state = JUMP_STATE_EXTEND;
        }
      }
      ChassisJump();
      break;
    default:
      break;
  }

  LimitChassisOutput();
}