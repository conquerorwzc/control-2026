/**
 ******************************************************************************
 * @file    hip_odometry.h
 * @brief   轮腿髋点纵向里程计的纯数学计算
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* 单条腿计算髋点纵向里程计所需的同一帧输入。 */
typedef struct
{
    float wheel_radius;       /* 轮半径，单位 m。 */
    float wheel_angle;        /* 轮端累计转角，前进方向为正，单位 rad。 */
    float wheel_speed;        /* 轮端角速度，前进方向为正，单位 rad/s。 */
    float leg_length;         /* 虚拟腿长，单位 m。 */
    float leg_length_dot;     /* 虚拟腿长度变化率，单位 m/s。 */
    float leg_theta;          /* 包含机身俯仰的纵向平面腿角，单位 rad。 */
    float leg_theta_dot;      /* 纵向平面腿角速度，单位 rad/s。 */
} WheelLeggedHipOdometryLegInput_t;

/* 单条腿推得的髋点纵向位置和速度。 */
typedef struct
{
    float raw_position; /* 未减初始零点的髋点纵向位置，单位 m。 */
    float velocity;     /* 髋点纵向速度，单位 m/s。 */
} WheelLeggedHipOdometryLegOutput_t;

/* 计算单条腿推得的髋点纵向原始位置和速度。 */
uint8_t WheelLeggedHipOdometryCalculate(const WheelLeggedHipOdometryLegInput_t *input,
                                        WheelLeggedHipOdometryLegOutput_t *output);
