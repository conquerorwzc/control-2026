/**
 ******************************************************************************
 * @file    chassis.h
 * @brief   双闭环轮腿底盘对象、命令接口与任务接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "bsp_can.h"
#include "dmmotor.h"
#include "double_closed_loop_leg.h"
#include "ins_task.h"

/* 双闭环机构学中两个主动轴角的固定输入槽位。 */
typedef enum
{
    LEG_KINEMATICS_INPUT_PHI1 = 0, /* phi1，即 x 轴到 O-A 的角。 */
    LEG_KINEMATICS_INPUT_PHI2,     /* phi2，即 x 轴到 O-C 的角。 */
    LEG_KINEMATICS_INPUT_COUNT,    /* 主动轴角总数，不代表实际输入。 */
} LegKinematicsInput_e;

/* 底盘工作模式；平衡、跳跃等模式在对应控制功能接入后再扩展。 */
typedef enum
{
    CHASSIS_POWER_OFF = 0, /* 关节电机保持零力矩。 */
    CHASSIS_ON,            /* 允许关节电机进入使能状态。 */
} WheelLeggedChassisMode_e;

/* 上层命令写入底盘的最小控制接口。 */
typedef struct
{
    WheelLeggedChassisMode_e chassis_mode; /* 上层请求的底盘模式。 */
} WheelLeggedChassisControlCommand_t;

/* 一台主动关节电机在底盘对象中的运行实例。 */
typedef struct
{
    DMMotorInstance *motor; /* 达妙电机驱动实例；初始化失败时为 NULL。 */
    uint8_t feedback_ready; /* 是否已经收到电机反馈帧。 */
} WheelLeggedJointInstance_t;

/* 一条腿的 VMC 命令和实时映射结果；默认只计算，手动打开测试开关后才允许受限下发。 */
typedef struct
{
    float force_command;        /* F：虚拟腿轴向广义力命令，单位 N；正值表示伸腿、撑开髋点与轮轴。 */
    float pitch_torque_command; /* Tp：虚拟腿摆动广义力矩命令，单位 N·m；正值使虚拟腿角增大。 */

    float phi1_torque; /* 映射得到的 phi1 主动轴目标力矩，单位 N·m。 */
    float phi2_torque; /* 映射得到的 phi2 主动轴目标力矩，单位 N·m。 */

    float front_motor_torque; /* 映射到前关节电机轴的目标力矩，单位 N·m。 */
    float rear_motor_torque;  /* 映射到后关节电机轴的目标力矩，单位 N·m。 */

    float front_motor_torque_output; /* 经过测试限幅后实际写入前关节电机的力矩参考，单位 N·m。 */
    float rear_motor_torque_output;  /* 经过测试限幅后实际写入后关节电机的力矩参考，单位 N·m。 */

    float virtual_jacobian_det; /* 当前虚拟腿雅可比行列式，工程单位 m/rad^2，用于观察接近奇异的位置。 */
    uint8_t torque_test_enable; /* 手动 VMC 小力矩测试使能；仅本腿为 1 且计算有效时允许下发力矩。 */
    uint8_t valid;              /* FK、雅可比、反馈和传动换算均有效时为 1。 */
} WheelLeggedLegVmc_t;

/* 一台关节的链轮传动标定参数；只需填写链轮齿数，程序自动计算角度传动比。 */
typedef struct
{
    uint8_t configured;              /* 链传动方向和零位已标定时为 1，未完成标定时必须为 0。 */
    uint16_t driving_sprocket_teeth; /* 电机侧主动链轮齿数。 */
    uint16_t driven_sprocket_teeth;  /* 主动轴侧从动链轮齿数。 */
    float direction;                 /* 电机正转对应 phi 增大时为 +1，反向时为 -1。 */
    float motor_zero_angle;          /* 机械主动轴 phi=0 时读到的电机累计角，单位 rad。 */
} WheelLeggedChainTransmissionConfig_t;

/* 一条腿的运行实例：两个主动关节和一个纯机械双闭环模型。 */
typedef struct
{
    WheelLeggedJointInstance_t front_joint; /* 前主动关节。 */
    WheelLeggedJointInstance_t rear_joint;  /* 后主动关节。 */

    LegKinematicsInput_e front_joint_kinematics_input; /* 前关节反馈送入 phi1 或 phi2。 */
    LegKinematicsInput_e rear_joint_kinematics_input;  /* 后关节反馈送入 phi1 或 phi2。 */

    DoubleClosedLoopLegConfig_t kinematics_runtime_config; /* 由链轮齿数生成的机构学运行配置。 */
    DoubleClosedLoopLegInstance_t kinematics;               /* 执行器角到足端状态的纯机构学实例。 */
    WheelLeggedLegVmc_t vmc;                                 /* 本腿 VMC 命令和实时计算状态。 */
    volatile uint32_t update_count;                          /* 该腿已完成的更新次数。 */
    volatile uint32_t sequence;                              /* 奇数表示写入中，偶数表示写入完成。 */
} WheelLeggedLegInstance_t;

/* 轮毂电机在底盘对象中的实例。当前只记录接线，待确认电机型号后再初始化。 */
typedef struct
{
    DMMotorInstance *motor;       /* 预留的轮毂达妙实例；当前未初始化，保持 NULL。 */
    CAN_Init_Config_s can_config; /* 轮毂 CAN 接线配置；兼容 F4 的 CAN 与 H7 的 FDCAN。 */
} WheelLeggedWheelInstance_t;

/* 一条腿的硬件和机构学初始化配置。 */
typedef struct
{
    Motor_Init_Config_s front_joint_motor_config; /* 前关节达妙的驱动配置。 */
    Motor_Init_Config_s rear_joint_motor_config;  /* 后关节达妙的驱动配置。 */
    const WheelLeggedChainTransmissionConfig_t *front_joint_chain_config; /* 前关节链轮传动配置。 */
    const WheelLeggedChainTransmissionConfig_t *rear_joint_chain_config;  /* 后关节链轮传动配置。 */
    LegKinematicsInput_e front_joint_kinematics_input; /* 前关节对应 phi1 或 phi2。 */
    LegKinematicsInput_e rear_joint_kinematics_input;  /* 后关节对应 phi1 或 phi2。 */
    const DoubleClosedLoopLegGeometry_t *geometry_config;                 /* 对应腿的双闭环 CAD 几何配置。 */
} WheelLeggedLegInitConfig_t;

/* 一个轮毂的 CAN 接线配置；因电机型号未知，当前不注册驱动。 */
typedef CAN_Init_Config_s WheelLeggedWheelCanConfig_t;

/* 底盘初始化配置；由 robot 层提供本车的电机、传动和几何参数。 */
typedef struct
{
    WheelLeggedLegInitConfig_t *left_leg_init_config;  /* 左腿专属初始化配置。 */
    WheelLeggedLegInitConfig_t *right_leg_init_config; /* 右腿专属初始化配置。 */
    WheelLeggedWheelCanConfig_t left_wheel_can_config; /* 左轮毂接线配置。 */
    WheelLeggedWheelCanConfig_t right_wheel_can_config;/* 右轮毂接线配置。 */
} WheelLeggedChassisInitConfig_t;

/* 底盘对象：具名左右字段保证 J-Link 可直接看出左右腿和左右轮。 */
typedef struct
{
    WheelLeggedChassisControlCommand_t chassis_ctrl_cmd; /* 上层下发的底盘命令。 */
    WheelLeggedLegInstance_t left_leg;                   /* 左腿。 */
    WheelLeggedLegInstance_t right_leg;                  /* 右腿。 */
    WheelLeggedWheelInstance_t left_wheel;               /* 左轮毂。 */
    WheelLeggedWheelInstance_t right_wheel;              /* 右轮毂。 */
    INS_t *imu;                                          /* 底盘 IMU 数据；当前未初始化。 */
    uint8_t joint_motor_enabled;                         /* 上次实际下发的关节使能状态。 */
} WheelLeggedChassisInstance_t;

void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis,
                            WheelLeggedChassisInitConfig_t *init_config);
void ChassisTask(WheelLeggedChassisInstance_t *chassis);
