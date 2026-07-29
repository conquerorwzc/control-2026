/**
 ******************************************************************************
 * @file    robot.h
 * @brief   双闭环轮腿机器人板级对象定义
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "bsp_can.h"
#include "dmmotor.h"
#include "double_closed_loop_leg.h"
#include "gimbal.h"
#include "ins_task.h"
#include "remote_control.h"

/* 双闭环机构学中两个主动轴角的固定输入槽位。 */
typedef enum
{
    LEG_KINEMATICS_INPUT_PHI1 = 0, /* phi1，即 x 轴到 O-A 的角。 */
    LEG_KINEMATICS_INPUT_PHI2,     /* phi2，即 x 轴到 O-C 的角。 */
    LEG_KINEMATICS_INPUT_COUNT,    /* 主动轴角总数，不代表实际输入。 */
} LegKinematicsInput_e;

/* 机器人当前的基础工作状态，控制器接入后可在此扩展状态机。 */
typedef enum
{
    ROBOT_MODE_POWER_OFF = 0, /* 主动关节调用停止接口，不允许输出驱动力矩。 */
    ROBOT_MODE_ZERO_TORQUE,   /* 达妙在线，但始终下发零力矩。 */
    ROBOT_MODE_CONTROL,       /* 后续接入整车控制器后的受控状态。 */
} RobotMode_e;

/* 一台主动关节电机在机器人对象中的运行实例。 */
typedef struct
{
    DMMotorInstance *motor; /* 达妙电机驱动实例；初始化失败时为 NULL。 */
    uint8_t feedback_ready; /* 是否已经收到电机反馈帧。 */
} WheelLeggedJointInstance_t;

/* 用于真机逆运动学验证的调试对象；默认只观测，显式使能后才允许受限电机测试。 */
typedef struct
{
    uint8_t calculation_enabled; /* 置 1 后计算目标足端的逆解；置 0 时目标自动保持当前足端位置。 */
    uint8_t target_ready;        /* 已获得可供手工微调的目标 P 时为 1。 */
    uint8_t motor_output_enabled; /* 显式置 1 后才允许本测试向两个关节下发位置 PID 参考；默认 0。 */
    uint8_t command_initialized;  /* 电机命令已从当前反馈角初始化时为 1，防止首次使能发生跳变。 */
    uint8_t output_active;        /* 本周期两个关节均通过安全检查且已经下发 PID 参考时为 1。 */
    DoubleClosedLoopLegVec2_t target_p; /* 足端 P 目标坐标，单位 m。 */
    DoubleClosedLoopLegStatus_e status; /* 最近一次逆运动学状态码。 */
    DoubleClosedLoopLegInput_t solution; /* 最近一次成功的 phi1、phi2 逆解，单位 rad。 */
    float actuator_angle_reference[LEG_KINEMATICS_INPUT_COUNT]; /* phi1、phi2 对应的电机累计角参考，单位 rad。 */
    float actuator_angle_command[LEG_KINEMATICS_INPUT_COUNT]; /* 经过速度限制后实际送入 PID 的电机累计角参考，单位 rad。 */
    float maximum_motor_reference_error; /* 目标参考相对当前电机角的最大允许偏差，单位 rad。 */
    float maximum_motor_reference_speed; /* 测试时电机累计角参考的最大变化速度，单位 rad/s。 */
    float maximum_motor_torque;           /* 测试时速度环最终输出的最大绝对力矩，单位 N·m。 */
    JointTransmissionStatus_e transmission_status[LEG_KINEMATICS_INPUT_COUNT]; /* 两个电机参考角反算状态。 */
} WheelLeggedLegInverseKinematicsTest_t;

/* 一台关节的链轮传动标定参数；人只需填写链轮齿数，程序自动计算角度传动比。 */
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

    DoubleClosedLoopLegConfig_t kinematics_runtime_config; /* 由链轮齿数生成的底层机构学运行配置。 */
    DoubleClosedLoopLegInstance_t kinematics;               /* 执行器角到足端状态的纯机构学实例。 */
    WheelLeggedLegInverseKinematicsTest_t inverse_kinematics_test; /* 不发力的 IK 调试状态。 */
    volatile uint32_t update_count;                          /* 该腿已完成的更新次数。 */
    volatile uint32_t sequence;                              /* 调试一致性序号：奇数表示写入中，偶数表示写入完成。 */
} WheelLeggedLegInstance_t;

/* 轮毂电机在机器人对象中的实例。当前只记录接线，待确认电机型号后再初始化。 */
typedef struct
{
    DMMotorInstance *motor;       /* 预留的轮毂达妙实例；当前未初始化，保持 NULL。 */
    CAN_Init_Config_s can_config; /* 轮毂 CAN 接线配置；兼容 F4 的 CAN 与 H7 的 FDCAN。 */
} WheelLeggedWheelInstance_t;

/* 底盘对象：用具名字段保证 J-Link 可直接看出左右腿和左右轮。 */
typedef struct
{
    WheelLeggedLegInstance_t left_leg;     /* 左腿。 */
    WheelLeggedLegInstance_t right_leg;    /* 右腿。 */
    WheelLeggedWheelInstance_t left_wheel; /* 左轮毂。 */
    WheelLeggedWheelInstance_t right_wheel;/* 右轮毂。 */
    INS_t *imu;                            /* 底盘 IMU 数据；当前未初始化。 */
} WheelLeggedChassisInstance_t;

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

/* 一个轮毂的 CAN 接线配置；使用 BSP 的跨芯片 CAN 抽象，因电机型号未知不注册驱动。 */
typedef CAN_Init_Config_s WheelLeggedWheelCanConfig_t;

/* 顶层机器人对象；所有板级状态必须从 robot 指针向下归属。 */
typedef struct
{
    RobotMode_e robot_mode;                     /* 机器人基础工作状态。 */
    WheelLeggedChassisInstance_t *chassis;      /* 轮腿底盘对象。 */
    GimbalInstance *gimbal;                     /* 云台对象；当前未初始化。 */
    RC_ctrl_t *rc_data;                         /* 遥控器数据；当前未初始化。 */
} RobotInstance;

void RobotInit(void);
void RobotTask(void);
