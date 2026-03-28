#pragma once

// 中科大的功率模型
// ===== M3508轮毂电机 6参数模型系数 =====
#define WHEEL_K0 0.7441993412640775f
#define WHEEL_K1 0.0090164284468539646f
#define WHEEL_K2 0.0001988857226262331f
#define WHEEL_K3 0.024694430204543864f
#define WHEEL_K4 0.20160143850678086f
#define WHEEL_K5 3.715221772539512e-05f

// ===== DJI M3508 =====
#define DJI_CURRENT_SCALE (20.0f / 16384.0)

// 3508改268/17减速比 扭矩-电流转换
#define t2i(torque) ((torque) * (3591.0f / 187.0f) / (268.0f / 17.0f) / 0.3f)
#define i2t(current) ((current) * 0.3f * (268.0f / 17.0f) / (3591.0f / 187.0f))

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
  float k[6]; // 功率控制拟合参数
  float w[2]; // 轮电机角速度

  float I_motion[2]; // 运动分量电流预测值
  float I_balance[2]; // 平衡分量电流预测值
  float I_total[2]; // 总电流预测值

  float T_motion[2]; // 运动分量扭矩计算值（由 LQR 算出）
  float T_balance[2]; // 平衡分量扭矩计算值（由 LQR 算出）
  float T_total[2]; // 总扭矩计算值（由 LQR 算出）

  float P_motion[2]; // 运动分量功率预测值
  float P_motion_total;
  float P_balance[2]; // 平衡分量功率预测值
  float P_balance_total;
  float P_total[2];

  float P_motion_total_ref;
  float P_motion_ref[2];
  float I_motion_ref[2];
  float T_motion_ref[2];
  float P_total_ref;
} PowerCtrl_t;


typedef struct ChassisInstance_t ChassisInstance;

void PowerControl(ChassisInstance* chassis);
void PowerControlInit(ChassisInstance* chassis);