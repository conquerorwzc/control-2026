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

/**
 * @brief  单电机6参数功率估算（中科大模型）
 *         P = k0 + k1*|I| + k2*|w| + k3*|I|*|w| + k4*I^2 + k5*w^2
 *
 * @param  k  系数数组 [k0..k5]
 * @param  I  电调电流 (A)，由 final_output / (16384/20) 换算得到
 * @param  w  转子角速度 (rad/s)，由 speed_aps * DEGREE_2_RAD 换算得到
 *            注意：speed_aps 是转子侧 deg/s，不除减速比
 * @return 估算电功率 (W)，钳位到 [0, +inf)
 */
static float MotorPowerCalc(const float k[6], float I, float w) {
  float abs_I = fabsf(I);
  float abs_w = fabsf(w);
  float P = k[0] + k[1] * abs_I + k[2] * abs_w + k[3] * abs_I * abs_w + k[4] * I * I + k[5] * w * w;
  return (P > 0.0f) ? P : 0.0f;
}

/**
 * @brief  功率控制主逻辑（无超级电容版本）
 *
 * 对应SJTU文档中的功率控制思路（简化版，无 f1(Vcap) 和 f2(Ebuffer)）：
 *
 *   P_ref = P_limit                          （无超级电容，直接使用上限）
 *   ṡd_max = Kp * sqrt(P_ref)               （开环基准速度）
 *           + PI(P_limit - P_filtered)       （闭环修正）
 *
 * 执行流程：
 *   Step 1: 同步功率上限 P_limit
 *   Step 2: 估算当前总功率 P_total，低通滤波得 P_filtered
 *   Step 3: 计算开环基准最大速度 vel_max_base
 *   Step 4: PI闭环计算速度修正量 delta_vel
 *   Step 5: vel_max = vel_max_base + delta_vel，非对称低通滤波
 *   Step 6: 对目标速度做幅值限制
 *   Step 7: 斜率限制，输出 limited_vx
 *
 * @note 无超级电容时，不再需要 E_buffer、buffer_ratio 等虚拟缓冲量。
 *       功率超限完全靠 PI 闭环 + vel_max 非对称低通来抑制。
 */
static void PowerControl(void) {
  PowerCtrl_t* pc = &chassis->power_ctrl;

  // ── 重启保护 ────────────────────────────────────────
  if (chassis->update_flag.is_restart) {
    chassis->limited_vx = 0.0f;
    pc->vx_ramp = 0.0f;
    pc->pi_integral = 0.0f;
    pc->P_filtered = 0.0f;
    // 重启时给一个保守的初始 vel_max
    float P_init = (chassis_ctrl_cmd->max_power > 0) ? (float)chassis_ctrl_cmd->max_power : POWER_DEFAULT_LIMIT;
    pc->vel_max = pc->Kp_vel * sqrtf(P_init * 0.8f);
    return;
  }

  // ── Step 1: 同步功率上限 ────────────────────────────
  // 优先使用裁判系统给定的功率上限，否则用默认值
  pc->P_limit = (chassis_ctrl_cmd->max_power > 0) ? (float)chassis_ctrl_cmd->max_power : POWER_DEFAULT_LIMIT;

  // ── Step 2: 估算当前总功率 + 低通滤波 ───────────────
  // 用 final_output（电调给定值）反算电流，再乘上转速估算功率
  // 注意：这里用的是"给定电流"而非"实测电流"
  // 好处：响应快，不受电流传感器延迟影响
  // 缺点：与实际功率有误差（电机特性非理想线性）
  // float I_L = (float)leg[1]->wheel_motor->motor_controller.final_output / DJI_CURRENT_SCALE;
  // float I_R = (float)leg[0]->wheel_motor->motor_controller.final_output / DJI_CURRENT_SCALE;
  float I_L = (float)leg[1]->wheel_motor->measure.real_current / DJI_CURRENT_SCALE;
  float I_R = (float)leg[0]->wheel_motor->measure.real_current / DJI_CURRENT_SCALE;
  // speed_aps 是转子侧角速度 (deg/s)，转换为 rad/s
  float w_L = leg[1]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;
  float w_R = leg[0]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;

  pc->P_wheel_L = MotorPowerCalc(pc->wheel_k, I_L, w_L);
  pc->P_wheel_R = MotorPowerCalc(pc->wheel_k, I_R, w_R);
  pc->P_total = pc->P_wheel_L + pc->P_wheel_R;

  // 一阶低通滤波，抑制电流噪声导致的 vel_max 抖动
  // P_filtered = (1-alpha)*P_filtered + alpha*P_total
  // alpha=0.1, dt=1ms → 时间常数约 9ms
  pc->P_filtered = (1.0f - POWER_LPF_ALPHA) * pc->P_filtered + POWER_LPF_ALPHA * pc->P_total;

  // ── Step 3: 开环基准最大速度 ────────────────────────
  // 根据文档：ṡd_max = Kp * sqrt(P_ref)，P_ref = P_limit（无超级电容）
  // 这是稳态的理论最大速度，实际还需要 PI 修正
  float vel_max_base = pc->Kp_vel * sqrtf(pc->P_limit);
  VAL_LIMIT(vel_max_base, 0.0f, POWER_VEL_HARD_LIMIT);

  // ── Step 4: PI 闭环速度修正 ─────────────────────────
  // 功率误差：正值表示还有余量，可以加速；负值表示超功率，需要减速
  float power_error = pc->P_limit - pc->P_filtered;

  // 积分项更新
  // 注意：积分只在速度未达到硬限幅时累积（防止 windup）
  //       当 vel_max 已经在硬限幅时，多余的正误差积分没有意义
  uint8_t at_hard_limit = (pc->vel_max >= POWER_VEL_HARD_LIMIT - 0.01f);
  if (!(at_hard_limit && power_error > 0.0f)) {
    pc->pi_integral += pc->Ki_power * power_error * chassis->dt;
  }
  // 积分限幅：防止长期超功率导致积分过大，也防止积分过负
  VAL_LIMIT(pc->pi_integral, -POWER_PI_INTEGRAL_MAX, POWER_PI_INTEGRAL_MAX);

  // PI 输出：速度修正量
  // 功率富余(error>0) → delta_vel > 0 → vel_max 增大
  // 功率超限(error<0) → delta_vel < 0 → vel_max 减小
  float delta_vel = pc->Kp_power * power_error + pc->pi_integral;

  // ── Step 5: 计算目标 vel_max_raw，非对称低通滤波 ─────
  // 目标速度 = 开环基准 + PI修正
  float vel_max_raw = vel_max_base + delta_vel;
  VAL_LIMIT(vel_max_raw, POWER_VEL_MIN, POWER_VEL_HARD_LIMIT);

  // 非对称低通：
  //   超功率时（vel_max_raw < vel_max）：快速压制，防止裁判系统扣血
  //   功率富余时（vel_max_raw > vel_max）：缓慢恢复，防止功率振荡
  // 对应SJTU文档中"合理目标速度的平方与功率成正比"的思想：
  //   功率超限 → 立即降速；功率富余 → 慢慢升速
  if (vel_max_raw < pc->vel_max) {
    // 快速降：alpha_down 大，响应快
    pc->vel_max = pc->vel_max * (1.0f - VEL_MAX_ALPHA_DOWN) + vel_max_raw * VEL_MAX_ALPHA_DOWN;
  } else {
    // 缓慢升：alpha_up 小，恢复慢
    pc->vel_max = pc->vel_max * (1.0f - VEL_MAX_ALPHA_UP) + vel_max_raw * VEL_MAX_ALPHA_UP;
  }
  VAL_LIMIT(pc->vel_max, POWER_VEL_MIN, POWER_VEL_HARD_LIMIT);

  // ── Step 6: 幅值限制 ────────────────────────────────
  // 对遥控器输入的目标速度进行限幅
  float vx_target = chassis_ctrl_cmd->vx;
  VAL_LIMIT(vx_target, -pc->vel_max, pc->vel_max);

  // ── Step 7: 斜率限制 ────────────────────────────────
  // 对应SJTU文档中的峰值场景(b)(c)：
  //   (b) 加速中段：斜率限制防止(ṡd-ṡ)过大，控制 K_s*(ṡd-ṡ) 产生的功率峰值
  //   (c) 急减速：斜率限制防止突然减速时轮子惯性前冲产生大力矩
  // 判断加减速：目标速度幅值 > 当前斜率速度幅值 → 加速；否则 → 减速
  float vx_diff = vx_target - pc->vx_ramp;
  uint8_t is_accel = (fabsf(vx_target) >= fabsf(pc->vx_ramp) - 0.02f);
  float ramp_rate = is_accel ? pc->vx_ramp_acc : pc->vx_ramp_dec;
  float max_step = ramp_rate * chassis->dt;

  if (vx_diff > max_step)
    pc->vx_ramp += max_step;
  else if (vx_diff < -max_step)
    pc->vx_ramp -= max_step;
  else
    pc->vx_ramp = vx_target;

  VAL_LIMIT(pc->vx_ramp, -pc->vel_max, pc->vel_max);

  // 输出最终限速后的目标速度给 LQR
  chassis->limited_vx = pc->vx_ramp;
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
    chassis->dt = 0.001f;
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

  for (int i = 0; i < 2; i++) {
    JointTorqueUpdate(leg[i]);
    SpringCompensation(leg[i]);
    JointLimitBarrier(leg[i]);
  }
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
  chassis_instance->imu = INS_Init(&chassis_init_config->imu_init_config);
  xvEstimateKF_Init(&chassis_instance->vaEstimateKF);

  // =========== 功率控制初始化（无超级电容版本）===========
  PowerCtrl_t* pc = &chassis_instance->power_ctrl;
  // 6参数模型系数
  pc->wheel_k[0] = WHEEL_K0;
  pc->wheel_k[1] = WHEEL_K1;
  pc->wheel_k[2] = WHEEL_K2;
  pc->wheel_k[3] = WHEEL_K3;
  pc->wheel_k[4] = WHEEL_K4;
  pc->wheel_k[5] = WHEEL_K5;
  // 控制参数
  pc->Kp_vel = POWER_KP_VEL;
  pc->Kp_power = POWER_PI_KP;
  pc->Ki_power = POWER_PI_KI;
  // 斜率限制
  pc->vx_ramp_acc = POWER_VX_RAMP_ACC;
  pc->vx_ramp_dec = POWER_VX_RAMP_DEC;
  // 初始状态：保守初始化，防止上电冲击
  pc->P_limit = POWER_DEFAULT_LIMIT;
  pc->P_filtered = 0.0f;
  pc->pi_integral = 0.0f;
  pc->vx_ramp = 0.0f;
  // 初始 vel_max 用 80% 功率估算，留余量
  pc->vel_max = POWER_KP_VEL * sqrtf(POWER_DEFAULT_LIMIT * 0.8f);
  chassis_instance->limited_vx = 0.0f;
  // ========================================================

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