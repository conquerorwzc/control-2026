/**
 ******************************************************************************
 * @file    parallel_leg.h
 * @brief   ACE 显式比例缩放同心五连杆纯运动学
 *
 * 坐标系：O1 为原点，x 向右为正，y 向下为正。
 * O2=(l0,0)；本车当前采用同心主动轴，因此 l0 固定为 0。
 *
 * 虚拟五连杆末端为 C，真实轮轴/虚拟腿末端为 J，满足 J=C/k，
 * 其中 k=CE/HJ。所有腿长、腿角和雅可比均以真实末端 J 为准。
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "joint_transmission.h"

typedef struct
{
    float x; /* 点在机构坐标系中的 x 坐标，单位 m。 */
    float y; /* 点在机构坐标系中的 y 坐标，单位 m。 */
} ParallelLegVec2_t;

typedef enum
{
    PARALLEL_LEG_OK = 0,           /* 本次机构学计算成功。 */
    PARALLEL_LEG_NOT_CONFIGURED,   /* 几何参数尚未完成标定。 */
    PARALLEL_LEG_INVALID_ARGUMENT, /* 调用参数为空或数值非法。 */
    PARALLEL_LEG_INVALID_GEOMETRY, /* 杆长或装配支路参数非法。 */
    PARALLEL_LEG_UNREACHABLE,      /* 当前主动角下两被动杆无法闭合。 */
    PARALLEL_LEG_SINGULAR,         /* 两圆重合或相切，末端/雅可比不唯一。 */
    PARALLEL_LEG_BRANCH_MISMATCH,  /* 逆解候选不属于配置的装配支路。 */
    PARALLEL_LEG_NUMERIC_ERROR,    /* 浮点计算出现非有限数。 */
} ParallelLegStatus_e;

typedef struct
{
    uint8_t configured;             /* l0、AH、HJ、CE 和装配支路均已填写且复核后为 1。 */
    float l0;                       /* O1 到 O2 的机架中心距，单位 m；ACE 同心模型必须填写 0。 */
    float real_first_link_ah;       /* AH：真实主动杆长度，单位 m。 */
    float real_second_link_hj;      /* HJ：真实末端杆长度，单位 m；用于计算比例 k。 */
    float virtual_second_link_ce;   /* CE：虚拟五连杆被动杆长度，单位 m。 */
    int8_t virtual_end_branch_sign; /* C 的支路：cross(D->E, D->C) 正取 +1，负取 -1。 */
    float singular_epsilon;         /* 圆心重合、相切和雅可比奇异判据，单位 m。 */
} ParallelLegGeometry_t;

typedef struct
{
    float phi1; /* O1 到第一主动杆的输入角，单位 rad。 */
    float phi2; /* O2 到第二主动杆的输入角，单位 rad。 */
} ParallelLegInput_t;

typedef struct
{
    ParallelLegVec2_t o1;                   /* 第一主动轴 O1，单位 m。 */
    ParallelLegVec2_t o2;                   /* 第二主动轴 O2，单位 m。 */
    float scale_k;                          /* k=CE/HJ，无量纲；虚拟 C 到真实 J 的位置缩放比。 */
    ParallelLegVec2_t virtual_first_end_d;  /* 虚拟第一主动杆末端 D，单位 m。 */
    ParallelLegVec2_t virtual_second_end_e; /* 虚拟第二主动杆末端 E，单位 m。 */
    ParallelLegVec2_t virtual_end_c;        /* 虚拟五连杆末端 C，单位 m。 */
    ParallelLegVec2_t real_second_end_h;    /* 真实第二主动杆末端 H，单位 m。 */
    ParallelLegVec2_t real_end_j;           /* 真实轮轴/虚拟腿末端 J，单位 m。 */
    float length;                           /* O1 到真实末端 J 的虚拟腿长，单位 m。 */
    float theta;                            /* 真实末端 J 相对 y 正方向的角，单位 rad。 */
    float virtual_leg_theta; /* J 指向 O1 的虚拟腿摆角，正值表示髋点向 x 正方向摆，单位 rad。 */
    float real_leg_jacobian[2][2];   /* 真实 J 的 [length, virtual_leg_theta] 对 [phi1, phi2] 的雅可比。 */
    float real_leg_jacobian_det;     /* 真实腿雅可比行列式，工程单位 m/rad^2。 */
    uint8_t real_leg_jacobian_valid; /* 真实腿雅可比有效时为 1。 */
    float first_loop_residual;       /* |D-C|-CE，单位 m。 */
    float second_loop_residual;      /* |E-C|-CE，单位 m。 */
} ParallelLegState_t;

typedef struct
{
    JointTransmissionConfig_t transmission[2]; /* 两台电机到 phi1、phi2 的传动标定。 */
    ParallelLegGeometry_t geometry;            /* 同心五连杆几何参数。 */
} ParallelLegConfig_t;

typedef struct
{
    const ParallelLegConfig_t *config;                /* 机构学只读配置。 */
    float actuator_angle[2];                          /* 两台电机的累计转子角，单位 rad。 */
    float phi[2];                                     /* 传动换算后的主动轴角 phi1、phi2，单位 rad。 */
    JointTransmissionStatus_e transmission_status[2]; /* 两条传动换算状态。 */
    ParallelLegStatus_e forward_kinematics_status;    /* 最近一次 FK 状态。 */
    ParallelLegState_t state;                         /* 最近一次机构学状态。 */
} ParallelLegInstance_t;

ParallelLegStatus_e ParallelLegForwardKinematics(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input,
                                                 ParallelLegState_t *state);
ParallelLegStatus_e ParallelLegInverseKinematics(const ParallelLegGeometry_t *geometry,
                                                 const ParallelLegVec2_t *target_j,
                                                 const ParallelLegInput_t *previous_input,
                                                 ParallelLegInput_t *solution);
void ParallelLegInit(ParallelLegInstance_t *instance, const ParallelLegConfig_t *config);
void ParallelLegUpdate(ParallelLegInstance_t *instance, float actuator_angle_0, float actuator_angle_1);
