//
// Created by 独梦幻想 on 2026/1/25.
//

#ifndef CONTROL_2026_SECONDORDERDIFFFF_H
#define CONTROL_2026_SECONDORDERDIFFFF_H
#include "stdint.h"
// 二阶差分前馈函数的上下文结构体，用于缓存历史输入值
typedef struct {
  float u_prev1;  // 前1拍输入
  float u_prev2;  // 前2拍输入
  float k_ff;     // 前馈增益系数
  uint8_t init_flag; // 初始化标志，0-未初始化，1-已初始化
} DiffFFContext;

// 函数声明
float secondOrderDiffFF(float u_curr);
DiffFFContext* diffFFContextInit(float k_ff);

#endif  // CONTROL_2026_SECONDORDERDIFFFF_H