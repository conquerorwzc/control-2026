// trajectory_planner.h
#pragma once
#include "math.h"
#include "stdint.h"

/* ========================================================================= */
/* 梯形规划器 (原有)                                                          */
/* ========================================================================= */
typedef struct
{
    float current_ref; // 当前虚拟参考位置
    float target_pos;  // 最终目标位置
    float current_vel; // 当前虚拟速度
    float max_vel;     // 最大限制速度
    float accel;       // 加/减速度
    float ff_speed;    // 换算好的前馈速度
    uint8_t is_moving; // 运动状态标志位
} TrapezoidalPlanner_t;

void Planner_Update(TrapezoidalPlanner_t *planner, float freq);

/* ========================================================================= */
/* S曲线规划器 (3层级联限幅: jerk → accel → vel → pos)                        */
/*                                                                            */
/* 原理：每tick对加速度做变化率限幅(jerk limit)，使得加速度曲线连续光滑，       */
/* 从而速度曲线呈S形、位置曲线无冲击。适用于半自动步进轨迹的关节平滑过渡。      */
/* ========================================================================= */
typedef struct
{
    // ---- 状态量 ----
    float pos;       // 当前输出位置
    float vel;       // 当前输出速度
    float accel;     // 当前输出加速度

    // ---- 目标 ----
    float target;    // 目标位置 (由模式函数写入)

    // ---- 限制参数 ----
    float max_vel;   // 最大速度限制 (单位/秒)
    float max_accel; // 最大加速度限制 (单位/秒²)
    float max_jerk;  // 最大加加速度限制 (单位/秒³) ← S曲线核心参数

    // ---- 状态 ----
    uint8_t is_moving; // 1=正在运动  0=已到达
} SCurvePlanner_t;

/**
 * @brief S曲线规划器单步更新
 * @param planner  规划器实例
 * @param freq     调用频率 (Hz), 如500Hz则传500
 */
void SCurvePlanner_Update(SCurvePlanner_t *planner, float freq);

/**
 * @brief 重置规划器状态 (用于模式切换时清零)
 * @param planner  规划器实例
 * @param init_pos 初始位置 (通常传当前关节角度)
 */
void SCurvePlanner_Reset(SCurvePlanner_t *planner, float init_pos);