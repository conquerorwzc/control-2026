/**
 ******************************************************************************
 * @file    double_closed_loop_leg.h
 * @brief   平面双闭环腿纯正运动学
 *
 * 坐标系：O 为原点，x 向右为正，y 向下为正。
 * 输入：phi1 是 x 轴到 O-A 的角，phi2 是 x 轴到 O-C 的角。
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "joint_transmission.h"

typedef struct
{
    float x; /* 点在 O 坐标系中向右为正的横坐标，单位 m。 */
    float y; /* 点在 O 坐标系中向下为正的纵坐标，单位 m。 */
} DoubleClosedLoopLegVec2_t;

typedef enum
{
    DOUBLE_CLOSED_LOOP_LEG_OK = 0,
    DOUBLE_CLOSED_LOOP_LEG_NOT_CONFIGURED,
    DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT,
    DOUBLE_CLOSED_LOOP_LEG_INVALID_GEOMETRY,
    DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_UNREACHABLE,
    DOUBLE_CLOSED_LOOP_LEG_SECOND_LOOP_UNREACHABLE,
    DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR,
    DOUBLE_CLOSED_LOOP_LEG_SECOND_LOOP_SINGULAR,
    DOUBLE_CLOSED_LOOP_LEG_BRANCH_MISMATCH,
    DOUBLE_CLOSED_LOOP_LEG_NUMERIC_ERROR,
    DOUBLE_CLOSED_LOOP_LEG_INVERSE_UNREACHABLE,
    DOUBLE_CLOSED_LOOP_LEG_INVERSE_SINGULAR,
} DoubleClosedLoopLegStatus_e;

typedef struct
{
    uint8_t configured; /* 几何数据是否完成 CAD/实物标定：1 为可用，0 为禁止求解。 */

    float oa_length; /* 杆 O-A 的转轴中心距，单位 m。 */
    float oc_length; /* 杆 O-C 的转轴中心距，单位 m。 */
    float ab_length; /* 杆 A-B 的转轴中心距，单位 m。 */
    float bc_length; /* 刚体 BCD 中 B-C 的转轴中心距，单位 m。 */
    float bd_length; /* 刚体 BCD 中 B-D 的转轴中心距，单位 m。 */
    float cd_length; /* 刚体 BCD 中 C-D 的转轴中心距，单位 m。 */
    float of_length; /* 直杆 O-C-F 中 O-F 的转轴中心距，单位 m。 */
    float cf_length; /* 直杆 O-C-F 中 C-F 的转轴中心距，单位 m；C 位于 O、F 之间。 */
    float de_length; /* 杆 D-E 的转轴中心距，单位 m。 */
    float ef_length; /* 刚体 EFP 中 E-F 的转轴中心距，单位 m。 */
    float ep_length; /* 刚体 EFP 中 E-P 的转轴中心距，单位 m。 */
    float fp_length; /* 刚体 EFP 中 F-P 的转轴中心距，单位 m。 */

    int8_t first_loop_branch_sign;  /* B 的叉积符号：cross(A-C, B-C) 为正取 +1，为负取 -1。 */
    int8_t bcd_branch_sign;         /* D 的叉积符号：cross(B-C, D-C) 为正取 +1，为负取 -1。 */
    int8_t second_loop_branch_sign; /* E 的叉积符号：cross(D-F, E-F) 为正取 +1，为负取 -1。 */
    int8_t efp_branch_sign;         /* P 的叉积符号：cross(E-F, P-F) 为正取 +1，为负取 -1。 */
    float geometry_consistency_epsilon; /* 共线杆尺寸一致性允许误差，单位 m。 */
    float singular_epsilon;         /* 两圆心距离接近零时判定为奇异的阈值，单位 m。 */
} DoubleClosedLoopLegGeometry_t;

typedef struct
{
    float phi1; /* x 轴转到 O-A 的主动轴角，单位 rad。 */
    float phi2; /* x 轴转到 O-C 的主动轴角，单位 rad。 */
} DoubleClosedLoopLegInput_t;

typedef struct
{
    DoubleClosedLoopLegVec2_t a; /* 铰点 A 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t b; /* 铰点 B 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t c; /* 铰点 C 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t d; /* 刚体 BCD 上特征点 D 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t e; /* 铰点 E 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t f; /* 刚体 OCF 上特征点 F 的坐标，单位 m。 */
    DoubleClosedLoopLegVec2_t p; /* 足端 P 的坐标，单位 m。 */

    float beta;                 /* 有向线 C->D 相对 x 正方向的绝对角，单位 rad。 */
    float epsilon;              /* 有向线 F->P 相对 x 正方向的绝对角，单位 rad。 */
    float length;               /* O 到足端 P 的虚拟腿长，单位 m。 */
    float theta;                /* 虚拟腿相对 y 正方向的俯仰角，单位 rad。 */
    float first_loop_residual;  /* 第一闭环杆 A-B 的长度残差，单位 m。 */
    float second_loop_residual; /* 第二闭环杆 D-E 的长度残差，单位 m。 */
} DoubleClosedLoopLegState_t;

typedef struct
{
    JointTransmissionConfig_t transmission[2];  /* 两台执行器到两个主动轴的传动标定参数。 */
    DoubleClosedLoopLegGeometry_t geometry;     /* 双闭环机构的 CAD 和装配参数。 */
} DoubleClosedLoopLegConfig_t;

typedef struct
{
    const DoubleClosedLoopLegConfig_t *config;              /* 只读的机构与传动配置。 */
    float actuator_angle[2];                                 /* 最近一次输入的两台执行器转子角，单位 rad。 */
    float phi[2];                                            /* 换算后的两个主动轴角 phi1、phi2，单位 rad。 */
    JointTransmissionStatus_e transmission_status[2];        /* 两条传动换算的状态码。 */
    DoubleClosedLoopLegStatus_e forward_kinematics_status;   /* 最近一次正运动学计算的状态码。 */
    DoubleClosedLoopLegState_t state;                        /* 最近一次成功或失败时的机构状态。 */
} DoubleClosedLoopLegInstance_t;

DoubleClosedLoopLegStatus_e DoubleClosedLoopLegForwardKinematics(const DoubleClosedLoopLegGeometry_t *geometry,
                                                                 const DoubleClosedLoopLegInput_t *input,
                                                                 DoubleClosedLoopLegState_t *state);
/* 由足端目标坐标解析反算两个主动轴角；previous_input 用于选取连续装配解。 */
DoubleClosedLoopLegStatus_e DoubleClosedLoopLegInverseKinematics(const DoubleClosedLoopLegGeometry_t *geometry,
                                                                 const DoubleClosedLoopLegVec2_t *target_p,
                                                                 const DoubleClosedLoopLegInput_t *previous_input,
                                                                 DoubleClosedLoopLegInput_t *solution);

/* 绑定只读配置并清空该腿实例的运行状态。 */
void DoubleClosedLoopLegInit(DoubleClosedLoopLegInstance_t *instance, const DoubleClosedLoopLegConfig_t *config);
/* 从两个执行器角依次完成链传动换算和双闭环正运动学。 */
void DoubleClosedLoopLegUpdate(DoubleClosedLoopLegInstance_t *instance, float actuator_angle_0, float actuator_angle_1);
