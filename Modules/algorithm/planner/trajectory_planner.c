//
// Created by JiaHaoRuan on 2026/4/10.
//

#include "trajectory_planner.h"
#include <math.h>

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