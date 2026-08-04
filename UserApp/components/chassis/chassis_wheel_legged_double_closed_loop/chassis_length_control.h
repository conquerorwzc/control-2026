/**
 ******************************************************************************
 * @file    chassis_length_control.h
 * @brief   双闭环轮腿单腿腿长 PID 纯计算接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "controller.h"

/* robot 层提供的单腿腿长 PID 标定参数；不保存运行期测量和积分状态。 */
typedef struct
{
    PID_Init_Config_s force_pid_config; /* 通用 PID 的 PI 和积分限幅配置；Kd 必须为 0。 */
    float velocity_damping_kd;          /* 基于 Jacobian 腿长速度的物理阻尼增益，单位 N*s/m。 */
    float length_reference;             /* 默认目标腿长，单位 m。 */
    float minimum_length;               /* 允许进入力控的最小腿长，单位 m。 */
    float maximum_length;               /* 允许进入力控的最大腿长，单位 m。 */
} WheelLeggedLegLengthControlInitConfig_t;

/* 每条腿独立保存的腿长 PID 运行状态、测量值和计算结果。 */
typedef struct
{
    uint8_t valid; /* 本周期 FK、雅可比、前馈和 PID 参数均有效时为 1。 */

    PID_Init_Config_s force_pid_config; /* 从 robot 层复制的 PID 初始化配置。 */
    PIDInstance force_pid;              /* 项目通用 PID 的每腿独立运行实例。 */
    float velocity_damping_kd;          /* 基于 Jacobian 腿长速度的物理阻尼增益，单位 N*s/m。 */

    float length_reference;    /* 目标腿长，单位 m。 */
    float measured_length;     /* 本周期 FK 发布的实际腿长，单位 m。 */
    float measured_length_dot; /* 本周期 FK/Jacobian 发布的腿长速度，单位 m/s。 */

    float length_error;       /* 本周期腿长误差，目标减实际，单位 m。 */
    float force_pid_command;  /* PI 与 Jacobian 速度阻尼合成的轴向力，单位 N。 */
    float gravity_feedforward; /* 本周期分配到本腿的重力前馈，单位 N。 */
    float force_command;      /* PID 与重力前馈合成并限幅后的虚拟轴向力 F，单位 N。 */

    float minimum_length; /* 允许进入力控的最小腿长，单位 m。 */
    float maximum_length; /* 允许进入力控的最大腿长，单位 m。 */
} WheelLeggedLegLengthControl_t;

void WheelLeggedLegLengthControlInit(WheelLeggedLegLengthControl_t *control,
                                     const WheelLeggedLegLengthControlInitConfig_t *config);
void WheelLeggedLegLengthControlUpdate(WheelLeggedLegLengthControl_t *control, uint8_t feedback_valid,
                                       float measured_length, float measured_length_dot, float gravity_feedforward);
