#include "power_control.h"

#include <math.h>

#include "chassis.h"
#include "general_def.h"
#include "user_lib.h"

/**
 * @brief  单电机 6 参数功率估算（中科大模型，正运算）
 *         P = k0 + k1·|I| + k2·|ω| + k3·|I|·|ω| + k4·I² + k5·ω²
 *
 * @param[in]  k  模型系数数组 [k0 … k5]
 * @param[in]  I  电调电流 (A)，由 final_output / (16384/20) 换算得到
 * @param[in]  w  转子角速度 (rad/s)，由 speed_aps × DEGREE_2_RAD 换算得到
 *                注意：speed_aps 为转子侧 deg/s，不除减速比
 * @return     估算电功率 (W)，钳位到 [0, +∞)
 */
static float MotorEstimatePower(const float k[6], float I, float w) {
  float P = k[0] + k[1] * I + k[2] * w + k[3] * I * w + k[4] * I * I + k[5] * w * w;
  return fmaxf(P, 0.0f);
}

/**
 * @brief  根据目标功率反解允许的最大电流（中科大模型，逆运算）
 *
 *         在功率模型中令 x = |I| ≥ 0，得到一元二次方程
 *             k4·x² + (k1 + k3·|ω|)·x + (k0 + k2·|ω| + k5·ω² − P) = 0
 *         物理约束下 A = k4 > 0，B = k1 + k3·|ω| ≥ 0，
 *         较小根恒 ≤ 0，故只需考虑较大根。
 *
 * @param[in]  k          模型系数数组 [k0 … k5]，与 MotorEstimatePower 共用
 * @param[in]  P_target   目标电功率 (W)，应 ≥ 0
 * @param[in]  w          转子角速度 (rad/s)
 * @param[in]  I_current  当前电流 (A)，仅用于确定返回值的符号方向
 * @return     满足功率约束的最大电流 (A)，符号与 I_current 一致；
 *             无有效解时返回 0
 *
 * @note   要求 k4 > 0，k1 ≥ 0，k3 ≥ 0（物理上通常成立）
 */
static float MotorEstimateCurrent(const float k[6], float P_target, float w, float I_current) {
  float A = k[4];
  float B = k[1] + k[3] * w;
  float C = k[0] + k[2] * w + k[5] * w * w - fabsf(P_target);

  float discriminant = B * B - 4.0f * A * C;

  if (discriminant < 0.0f) {
    /* 目标功率过低，无实根；取顶点作为近似解 */
    return -B / (2.0f * A);
  }

  float sqrt_disc = sqrtf(discriminant);

  /* 唯一需要考虑的根 */
  float temp1 = (-B + sqrt_disc) / (2.0f * A);
  float temp2 = (-B - sqrt_disc) / (2.0f * A);

  if (I_current > 0.0f) {
    return (fabsf(temp1 - I_current) < fabsf(temp2 - I_current) ? fminf(16000.f, temp1) : fminf(16000.f, temp2));

  } else {
    return (fabsf(temp1 - I_current) < fabsf(temp2 - I_current) ? fmaxf(-16000.f, temp1) : fmaxf(-16000.f, temp2));
  }
}

/**
 * @brief 功率控制器
 *
 * 对应SJTU文档第7节:
 *   s_d_max = Kp * sqrt(P_ref)
 *   P_ref   = P_limit  (无超级电容，f1=f2=0)
 *
 * 斜率限制对应文档4-(b)(c):
 *   (b) 加速中段: Ks/(2Rω) * ṡ*(s_d-s) → 限制加速斜率
 *   (c) 急减速:   Ks/(2Rω) * ṡ*(s-s_d) → 限制减速斜率
 */
void PowerControl(ChassisInstance* chassis) {
  State_Var_t* sv = &chassis->state_var;
  PowerCtrl_t* pc = &chassis->power_ctrl;

  float state_err[10];
  state_err[0] = sv->x_b - 0.0f;
  state_err[1] = sv->x_b_d - chassis->chassis_ctrl_cmd.vx;
  VAL_LIMIT(state_err[1], -2.7f, 2.7f);
  state_err[2] = sv->phi - chassis->chassis_ctrl_cmd.target_yaw;
  VAL_LIMIT(state_err[2], -0.52f, 0.52f);  // ±30°
  state_err[3] = sv->phi_d - chassis->chassis_ctrl_cmd.wz;
  VAL_LIMIT(state_err[3], -2.0f, 2.0f);

  pc->T_total[0] = chassis->leg[0]->real_model.T;
  pc->T_total[1] = chassis->leg[1]->real_model.T;

  pc->T_motion[0] = 0.0f;
  pc->T_motion[1] = 0.0f;
  for (int i = 0; i < 4; i++) {
    pc->T_motion[0] -= chassis->LQR_K[2][i] * state_err[i];
    pc->T_motion[1] -= chassis->LQR_K[3][i] * state_err[i];
  }

  for (int i = 0; i < 2; i++) {
    pc->T_balance[i] = pc->T_total[i] - pc->T_motion[i];
    pc->I_balance[i] = t2i(pc->T_balance[i]);
    pc->I_motion[i] = t2i(pc->T_motion[i]);

    pc->w[i] = chassis->leg[i]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;

    pc->P_motion[i] = MotorEstimatePower(pc->k, pc->I_motion[i], pc->w[i]);
    pc->P_balance[i] = MotorEstimatePower(pc->k, pc->I_balance[i], pc->w[i]);
  }

  pc->P_motion_total = pc->P_motion[0] + pc->P_motion[1];
  pc->P_balance_total = pc->P_balance[0] + pc->P_balance[1];
  pc->P_total_ref = chassis->chassis_ctrl_cmd.max_power;
  pc->P_motion_total_ref = chassis->chassis_ctrl_cmd.max_power - pc->P_balance_total;

  if (pc->P_motion_total > pc->P_motion_total_ref) {
    for (int i = 0; i < 2; i++) {
      // 功率分配：许用功率 * 当前功率用量比例
      pc->P_motion_ref[i] = pc->P_motion[i] / (pc->P_motion_total) * pc->P_motion_total_ref;
      pc->I_motion_ref[i] = MotorEstimateCurrent(pc->k, pc->P_motion_ref[i], pc->w[i], pc->I_motion[i]);
      pc->T_motion_ref[i] = i2t(pc->I_motion_ref[i]);
    }

    // 通过 LQR_K 的伪逆矩阵反解限制后的状态误差 (x, x_d, phi, phi_d)
    // A = [ K[2][0..3] ]
    //     [ K[3][0..3] ]
    float A[2][4];
    for (int j = 0; j < 4; j++) {
      A[0][j] = chassis->LQR_K[2][j];
      A[1][j] = chassis->LQR_K[3][j];
    }

    // 计算 A * A^T
    float M[2][2] = {0};
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 4; k++) {
          M[i][j] += A[i][k] * A[j][k];
        }
      }
    }

    // 计算 M 的逆
    float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    if (fabsf(det) > 1e-6f) {
      float invM[2][2];
      invM[0][0] = M[1][1] / det;
      invM[0][1] = -M[0][1] / det;
      invM[1][0] = -M[1][0] / det;
      invM[1][1] = M[0][0] / det;

      // 计算伪逆 A^+ = A^T * invM (维度 4x2)
      float A_pinv[4][2];
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
          A_pinv[i][j] = A[0][i] * invM[0][j] + A[1][i] * invM[1][j];
        }
      }

      // 计算新的 state_err' = - A^+ * T_motion_ref
      float new_state_err[4];
      for (int i = 0; i < 4; i++) {
        new_state_err[i] = -(A_pinv[i][0] * pc->T_motion_ref[0] + A_pinv[i][1] * pc->T_motion_ref[1]);
      }

      // 反解出对应的控制指令
      // state_err[1] = x_b_d - vx => vx = x_b_d - state_err[1]
      // state_err[2] = phi - target_yaw => target_yaw = phi - state_err[2]
      // state_err[3] = phi_d - wz => wz = phi_d - state_err[3]
      chassis->chassis_ctrl_cmd.vx = sv->x_b_d - new_state_err[1];
      chassis->chassis_ctrl_cmd.target_yaw = sv->phi - new_state_err[2];
      chassis->chassis_ctrl_cmd.wz = sv->phi_d - new_state_err[3];
    }
  }
}

void PowerControlInit(ChassisInstance* chassis) {
  PowerCtrl_t* pc = &chassis->power_ctrl;
  // 6参数模型系数
  pc->k[0] = WHEEL_K0;
  pc->k[1] = WHEEL_K1;
  pc->k[2] = WHEEL_K2;
  pc->k[3] = WHEEL_K3;
  pc->k[4] = WHEEL_K4;
  pc->k[5] = WHEEL_K5;
}