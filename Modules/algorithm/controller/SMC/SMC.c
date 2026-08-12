/**
* @file SMC.c
 * @author 梅峻炜@FDU-EGA
 * @author modified by YZC
 * @brief  SMC控制器定义
 * @version release
 * @date 2026-4-14
 **/

#include "SMC.h"

// 全局偏航控制器实例
SMC YawSMC;
/**
 * @brief 符号函数
 */
static int8_t signal(float y) {
  if (y > 0.0f) {
    return 1;
  } else if (y == 0.0f) {
    return 0;
  } else {
    return -1;
  }
}

/**
 * @brief 饱和函数（原Sat）
 */
static float sat(float y) {
  if (fabsf(y) <= 1.0f) {
    return y;
  } else {
    return (float)signal(y);
  }
}

/**
 * @brief 滑模控制器初始化实现
 */
void smc_init(SMC *smc, float C, float K, float ref, float error_eps,
              float u_max, float J, float epsilon) {
  // 初始化公有参数
  smc->C = C;
  smc->K = K;
  smc->ref = ref;
  smc->error_eps = error_eps;
  smc->u_max = u_max;
  smc->J = J;
  smc->epsilon = epsilon;

  // 初始化状态变量
  smc->angle = 0.0f;
  smc->ang_vel = 0.0f;
  smc->u = 0.0f;
  smc->error = 0.0f;
  smc->error_last = 0.0f;
  smc->dref = 0.0f;
  smc->ddref = 0.0f;
  smc->refl = 0.0f;
  smc->s = 0.0f;
}

/**
 * @brief 滑模控制器周期执行实现
 */
void smc_tick(SMC *smc, float angle_now, float angle_vel,float angle_ref) {
  // 更新当前角度/角速度
  smc->angle = angle_now;
  smc->ang_vel = angle_vel;
  smc->ref = angle_ref;
  // 计算角度误差
  smc->error = smc->angle - smc->ref;

  // 误差阈值内输出0
  if (fabsf(smc->error) < smc->error_eps) {
    smc->u = 0.0f;
    return;
  }

  // 计算目标值的一阶/二阶导数
  smc->ddref = (smc->ref - smc->refl) - smc->dref;
  smc->dref = (smc->ref - smc->refl);

  // 计算滑模面
  smc->s = smc->C * smc->error + (smc->ang_vel - smc->dref);

  // 计算控制输出
  smc->u = smc->J * (smc->ddref - smc->C * (smc->ang_vel - smc->dref) -
                     smc->epsilon * sat(smc->s) - smc->K * smc->s);

  // 输出限幅
  if (smc->u > smc->u_max) {
    smc->u = smc->u_max;
  }
  if (smc->u < -smc->u_max) {
    smc->u = -smc->u_max;
  }

  // 更新上一帧目标值
  smc->refl = smc->ref;
}

// /**
//  * @brief 全局YawSMC初始化（参数与原C++一致）
//  */
// void YawSMC_Init(void) {
//   smc_init(&YawSMC, 20.0f, 120.0f, 0.0f, 0.001f, 25000.0f, 0.8f, 0.5f);
// }