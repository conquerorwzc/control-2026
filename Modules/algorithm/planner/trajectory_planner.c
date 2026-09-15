//
// Created by JiaHaoRuan on 2026/4/10.
//

#include "trajectory_planner.h"
#include <math.h>

/* ========================================================================= */
/* 梯形规划器 (原有)                                                          */
/* ========================================================================= */
void Planner_Update(TrapezoidalPlanner_t *planner, float freq)
{
    float remain_dist = fabsf(planner->target_pos - planner->current_ref);
    float dt = 1.0f / freq;

    // 防零除死机
    if (planner->accel <= 0.0001f)
    {
        planner->current_vel = 0.0f;
        planner->current_ref = planner->target_pos;
        planner->ff_speed = 0.0f;
        planner->is_moving = 0;
        return;
    }

    // 计算减速距离 decel_dist = v^2 / 2a
    float decel_dist = (planner->current_vel * planner->current_vel) / (2.0f * planner->accel);

    if (remain_dist <= 0.5f)
    { // 逼近阈值 (0.5度误差内直接吸附)
        planner->current_vel = 0.0f;
        planner->current_ref = planner->target_pos;
        planner->ff_speed = 0.0f;
        planner->is_moving = 0;
        return;
    }
    else if (remain_dist <= decel_dist)
    {
        // 需要减速
        planner->current_vel -= planner->accel * dt;
        if (planner->current_vel < 5.0f) // 最小保底速度防卡死
            planner->current_vel = 5.0f;
    }
    else
    {
        // 需要加速或匀速
        planner->current_vel += planner->accel * dt;
        if (planner->current_vel > planner->max_vel)
            planner->current_vel = planner->max_vel;
    }

    // 根据方向更新当前参考位置和前馈速度
    float step = planner->current_vel * dt;
    if (planner->target_pos > planner->current_ref)
    {
        planner->current_ref += step;
        planner->ff_speed = planner->current_vel;
    }
    else
    {
        planner->current_ref -= step;
        planner->ff_speed = -planner->current_vel;
    }
    planner->is_moving = 1;
}

/* ========================================================================= */
/* S曲线规划器                                                                */
/*                                                                            */
/* 算法：3层级联限幅器                                                        */
/*   1. 计算期望加速度 = (期望速度 - 当前速度) / dt                            */
/*   2. 对加速度做 jerk 限幅：Δa = clamp(a_desired - a_current, -jerk*dt, +jerk*dt) */
/*   3. 对速度做 accel 限幅 (隐含在加速度限幅中)                               */
/*   4. 对位置做 vel 限幅 (隐含在速度限幅中)                                   */
/*                                                                            */
/* 效果：加速度曲线连续 → 速度曲线光滑S形 → 位置曲线无无穷大冲击              */
/* ========================================================================= */
void SCurvePlanner_Update(SCurvePlanner_t *planner, float freq)
{
    float dt = 1.0f / freq;
    float error = planner->target - planner->pos;

    // ---- 到达判定 ----
    if (fabsf(error) < 0.01f && fabsf(planner->vel) < 0.1f)
    {
        planner->pos = planner->target;
        planner->vel = 0.0f;
        planner->accel = 0.0f;
        planner->is_moving = 0;
        return;
    }

    // ---- 计算期望速度 (基于剩余距离的减速曲线) ----
    // 利用 v² = 2*a*s 反推：在当前加速度下，从当前速度刹停需要的距离
    float stop_dist = (planner->vel * planner->vel) / (2.0f * planner->max_accel);
    float abs_error = fabsf(error);

    float desired_vel;
    if (abs_error < 0.5f)
    {
        // 极近距离：直接吸附
        desired_vel = 0.0f;
    }
    else if (abs_error <= stop_dist * 1.1f)
    {
        // 进入减速区：根据剩余距离反推目标速度
        // v_target = sqrt(2 * a * s)，保留方向
        float sign = (error > 0.0f) ? 1.0f : -1.0f;
        float brake_vel = sqrtf(2.0f * planner->max_accel * abs_error);
        // 限制最小减速速度，防止过早停死
        if (brake_vel < 1.0f) brake_vel = 1.0f;
        desired_vel = sign * brake_vel;
    }
    else
    {
        // 匀速/加速区：朝最大速度冲刺
        float sign = (error > 0.0f) ? 1.0f : -1.0f;
        desired_vel = sign * planner->max_vel;
    }

    // ---- 第1层：加速度限幅 (jerk limit) ----
    // 期望加速度 = (期望速度 - 当前速度) / dt
    float desired_accel = (desired_vel - planner->vel) / dt;

    // 限制期望加速度在 [-max_accel, +max_accel] 范围内
    if (desired_accel > planner->max_accel)
        desired_accel = planner->max_accel;
    else if (desired_accel < -planner->max_accel)
        desired_accel = -planner->max_accel;

    // 对加速度变化率做 jerk 限幅 (S曲线核心！)
    float accel_error = desired_accel - planner->accel;
    float max_delta_a = planner->max_jerk * dt;

    if (accel_error > max_delta_a)
        planner->accel += max_delta_a;
    else if (accel_error < -max_delta_a)
        planner->accel -= max_delta_a;
    else
        planner->accel = desired_accel;

    // ---- 第2层：速度更新 (加速度已平滑) ----
    planner->vel += planner->accel * dt;

    // 速度限幅
    if (planner->vel > planner->max_vel)
        planner->vel = planner->max_vel;
    else if (planner->vel < -planner->max_vel)
        planner->vel = -planner->max_vel;

    // ---- 第3层：位置更新 ----
    planner->pos += planner->vel * dt + 0.5f * planner->accel * dt * dt;

    // 防止过冲：如果方向反转了，说明已经越过目标
    float new_error = planner->target - planner->pos;
    if ((error > 0.0f && new_error < 0.0f) || (error < 0.0f && new_error > 0.0f))
    {
        // 已过冲，直接吸附到目标并清零
        planner->pos = planner->target;
        planner->vel = 0.0f;
        planner->accel = 0.0f;
    }

    planner->is_moving = 1;
}

void SCurvePlanner_Reset(SCurvePlanner_t *planner, float init_pos)
{
    planner->pos = init_pos;
    planner->target = init_pos;
    planner->vel = 0.0f;
    planner->accel = 0.0f;
    planner->is_moving = 0;
}