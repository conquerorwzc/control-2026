//
// Created by 独梦幻想 on 2026/1/25.
//

#include "SecondOrderDiffFF.h"
static DiffFFContext ctx;

/**
 * @brief  二阶差分前馈计算函数
 * @param  u_curr: 当前系统输入值
 * @retval 二阶差分前馈量
 */
float secondOrderDiffFF(float u_curr)
{
  float ff_out = 0.0f;

  // 初始化阶段：缓存前两拍输入，前馈量为0
  if (ctx.init_flag == 0) {
    ctx.u_prev1 = u_curr;
    ctx.u_prev2 = u_curr;
    ctx.init_flag = 1;
    return ff_out;
  }

  // 计算二阶差分：Δ²u(k) = u(k) - 2u(k-1) + u(k-2)
  float diff2 = u_curr - 2 * ctx.u_prev1 + ctx.u_prev2;

  // 计算前馈输出：前馈量 = 前馈增益 × 二阶差分
  ff_out = ctx.k_ff * diff2;

  // 更新历史输入缓存
  ctx.u_prev2 = ctx.u_prev1;
  ctx.u_prev1 = u_curr;

  return ff_out;
}

// 初始化上下文
DiffFFContext* diffFFContextInit(float k_ff) {
  ctx.u_prev1 = 0.0f;
  ctx.u_prev2 = 0.0f;
  ctx.k_ff = k_ff;
  ctx.init_flag = 0;
  return &ctx;
}
