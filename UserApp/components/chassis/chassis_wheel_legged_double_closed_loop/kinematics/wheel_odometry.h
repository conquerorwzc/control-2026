/**
 ******************************************************************************
 * @file    wheel_odometry.h
 * @brief   轮腿底盘纯轮式纵向里程计的纯数学计算
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* 左右轮平均纵向里程计的同一帧输入。 */
typedef struct
{
    float left_wheel_radius;  /* 左轮有效半径，单位 m。 */
    float right_wheel_radius; /* 右轮有效半径，单位 m。 */
    float left_wheel_angle;   /* 左轮端累计转角，前进方向为正，单位 rad。 */
    float right_wheel_angle;  /* 右轮端累计转角，前进方向为正，单位 rad。 */
    float left_wheel_speed;   /* 左轮端角速度，前进方向为正，单位 rad/s。 */
    float right_wheel_speed;  /* 右轮端角速度，前进方向为正，单位 rad/s。 */
} WheelLeggedWheelOdometryInput_t;

/* 左右轮平均纵向里程计输出。 */
typedef struct
{
    float raw_position; /* 未减初始零点的位置，单位 m。 */
    float velocity;     /* 纵向速度，单位 m/s。 */
} WheelLeggedWheelOdometryOutput_t;

/* 计算纯轮式平均纵向位置和速度；腿长、腿角不参与此计算。 */
uint8_t WheelLeggedWheelOdometryCalculate(const WheelLeggedWheelOdometryInput_t *input,
                                          WheelLeggedWheelOdometryOutput_t *output);
