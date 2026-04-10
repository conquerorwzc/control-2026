// trajectory_planner.h
#pragma once
#include "math.h"
#include "stdint.h"

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