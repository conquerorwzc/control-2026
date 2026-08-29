/**
******************************************************************************
 * @file SMC.h
 * @author 梅峻炜@FDU-EGA
 * @author modified by YZC
 * @brief  SMC控制器定义
 * @version release
 * @date 2026-4-14
 * ******************************************************************************
 **/
#ifndef _SMC_H_
#define _SMC_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include <stdint.h>

// 滑模控制器结构体（对应原C++类的所有成员）
typedef struct {
  // 原公有成员
  float C;
  float K;
  float ref;          // 目标角度
  float error_eps;    // 误差阈值
  float u_max;        // 输出最大值
  float J;            // 转动惯量
  float angle;        // 当前角度(度)
  float ang_vel;      // 当前角速度(度/s)
  float epsilon;      // 滑模控制参数
  float u;            // 最终控制输出

  // 原私有成员（C无访问控制，逻辑上仅通过函数操作）
  float error;        // 角度误差
  float error_last;   // 上一帧误差
  float dref;         // 目标值一阶导
  float ddref;        // 目标值二阶导
  float refl;         // 上一帧目标值
  float s;            // 滑模面
} SMC;

/**
 * @brief 滑模控制器初始化（替代C++构造函数）
 * @param smc: 控制器实例指针
 * @param C: 滑模面参数
 * @param K: 滑模增益
 * @param ref: 初始目标值
 * @param error_eps: 误差阈值
 * @param u_max: 输出限幅最大值
 * @param J: 转动惯量
 * @param epsilon: 饱和函数增益
 */
void smc_init(SMC *smc, float C, float K, float ref, float error_eps,
              float u_max, float J, float epsilon);

/**
 * @brief 滑模控制器周期执行
 * @param smc: 控制器实例指针
 * @param angle_now: 当前角度
 * @param angle_vel: 当前角速度
 */
void smc_tick(SMC *smc, float angle_now, float angle_vel,float angle_ref);


#endif // _SMC_H_
