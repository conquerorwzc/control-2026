/**
 ******************************************************************************
 * @file    chassis.h
 * @brief   同心五连杆轮腿底盘对象、命令接口与任务接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

#include "bsp_can.h"
#include "chassis_length_control.h"
#include "chassis_lqr.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "parallel_leg.h"

/* 同心五连杆机构学中两个主动轴角的固定输入槽位。 */
typedef enum
{
    LEG_KINEMATICS_INPUT_PHI1 = 0, /* phi1，即前关节主动杆相对 x 轴的角。 */
    LEG_KINEMATICS_INPUT_PHI2,     /* phi2，即后关节主动杆相对 x 轴的角。 */
    LEG_KINEMATICS_INPUT_COUNT,    /* 主动轴角总数，不代表实际输入。 */
} LegKinematicsInput_e;

/* 底盘工作模式；平衡、跳跃等模式在对应控制功能接入后再扩展。 */
/* TODO：接入正式平衡、跳跃和故障锁定状态机时扩展底盘模式。 */
typedef enum
{
    CHASSIS_POWER_OFF = 0, /* 关节电机保持零力矩。 */
    CHASSIS_ON,            /* 允许通过完整闭环安全检查后使能六台底盘电机。 */
} WheelLeggedChassisMode_e;

/* 十维状态数组的固定下标；后续 LQR 必须严格沿用该顺序。 */
typedef enum
{
    WHEEL_LEGGED_STATE_S = WHEEL_LEGGED_LQR_STATE_S, /* 整车前进相对位移，单位 m。 */
    WHEEL_LEGGED_STATE_S_DOT = WHEEL_LEGGED_LQR_STATE_S_DOT, /* 整车前进速度，单位 m/s。 */
    WHEEL_LEGGED_STATE_PHI = WHEEL_LEGGED_LQR_STATE_PHI, /* 整车偏航角，单位 rad。 */
    WHEEL_LEGGED_STATE_PHI_DOT = WHEEL_LEGGED_LQR_STATE_PHI_DOT, /* 整车偏航角速度，单位 rad/s。 */
    WHEEL_LEGGED_STATE_THETA_LEFT = WHEEL_LEGGED_LQR_STATE_THETA_LEFT, /* 左虚拟腿世界系摆角，单位 rad。 */
    WHEEL_LEGGED_STATE_THETA_LEFT_DOT = WHEEL_LEGGED_LQR_STATE_THETA_LEFT_DOT, /* 左虚拟腿世界系摆角速度，单位 rad/s。 */
    WHEEL_LEGGED_STATE_THETA_RIGHT = WHEEL_LEGGED_LQR_STATE_THETA_RIGHT, /* 右虚拟腿世界系摆角，单位 rad。 */
    WHEEL_LEGGED_STATE_THETA_RIGHT_DOT = WHEEL_LEGGED_LQR_STATE_THETA_RIGHT_DOT, /* 右虚拟腿世界系摆角速度，单位 rad/s。 */
    WHEEL_LEGGED_STATE_THETA_BODY = WHEEL_LEGGED_LQR_STATE_THETA_BODY, /* 机身俯仰角，单位 rad。 */
    WHEEL_LEGGED_STATE_THETA_BODY_DOT = WHEEL_LEGGED_LQR_STATE_THETA_BODY_DOT, /* 机身俯仰角速度，单位 rad/s。 */
    WHEEL_LEGGED_STATE_COUNT,           /* 十维状态总数，不代表一个实际状态。 */
} WheelLeggedChassisStateIndex_e;

_Static_assert(WHEEL_LEGGED_STATE_COUNT == 10, "五连杆底盘状态向量必须保持十维");
_Static_assert(WHEEL_LEGGED_STATE_COUNT == WHEEL_LEGGED_LQR_STATE_COUNT, "十维状态和 LQR 状态必须保持一致");

/* 十维状态各原始来源的有效标志位。 */
typedef enum
{
    WHEEL_LEGGED_STATE_VALID_LEFT_WHEEL = (1u << 0),   /* 左轮端反馈和传动参数有效。 */
    WHEEL_LEGGED_STATE_VALID_RIGHT_WHEEL = (1u << 1),  /* 右轮端反馈和传动参数有效。 */
    WHEEL_LEGGED_STATE_VALID_IMU = (1u << 2),          /* IMU 已初始化且姿态数据有限。 */
    WHEEL_LEGGED_STATE_VALID_LEFT_LEG = (1u << 3),     /* 左腿反馈、FK 和雅可比有效。 */
    WHEEL_LEGGED_STATE_VALID_RIGHT_LEG = (1u << 4),    /* 右腿反馈、FK 和雅可比有效。 */
    WHEEL_LEGGED_STATE_VALID_WHEEL_ODOMETRY = (1u << 5), /* 左右轮平均纵向里程计计算有效。 */
} WheelLeggedChassisStateValid_e;

/* 底盘状态变量；具名字段便于观察，state_vector 供后续控制器使用。 */
typedef struct
{
    float s;       /* 左右轮平均滚动位移相对初始位置的纵向里程计位移，单位 m。 */
    float s_dot;   /* 左右轮平均滚动纵向里程计速度，单位 m/s。 */
    float phi;     /* 整车相对初始方向的偏航角，单位 rad。 */
    float phi_dot; /* 整车偏航角速度，单位 rad/s。 */
    float theta_leg_left; /* 左虚拟腿包含相对初始姿态机身俯仰的纵向平面摆角，单位 rad；不含 yaw 投影。 */
    float theta_leg_left_dot; /* 左虚拟腿纵向平面摆角速度，单位 rad/s。 */
    float theta_leg_right; /* 右虚拟腿包含相对初始姿态机身俯仰的纵向平面摆角，单位 rad；不含 yaw 投影。 */
    float theta_leg_right_dot; /* 右虚拟腿纵向平面摆角速度，单位 rad/s。 */
    float theta_body;          /* 机身相对初始姿态的俯仰角，单位 rad。 */
    float theta_body_dot;      /* 机身俯仰角速度，单位 rad/s。 */

    float left_leg_relative_theta;      /* 左腿 FK 得到的相对机身摆角，单位 rad。 */
    float left_leg_relative_theta_dot;  /* 左腿相对机身摆角速度，单位 rad/s。 */
    float right_leg_relative_theta;     /* 右腿 FK 得到的相对机身摆角，单位 rad。 */
    float right_leg_relative_theta_dot; /* 右腿相对机身摆角速度，单位 rad/s。 */
    float left_wheel_angle;             /* 左轮换算到轮端的累计转角，单位 rad。 */
    float left_wheel_speed;             /* 左轮换算到轮端的角速度，单位 rad/s。 */
    float right_wheel_angle;            /* 右轮换算到轮端的累计转角，单位 rad。 */
    float right_wheel_speed;            /* 右轮换算到轮端的角速度，单位 rad/s。 */
    float left_leg_length;              /* 左虚拟腿长，单位 m。 */
    float left_leg_length_dot;          /* 左虚拟腿长度变化率，单位 m/s。 */
    float right_leg_length;             /* 右虚拟腿长，单位 m。 */
    float right_leg_length_dot;         /* 右虚拟腿长度变化率，单位 m/s。 */
    float wheel_odometry_s_raw;         /* 未减零点的左右轮平均纵向里程计，单位 m。 */
    float wheel_odometry_s_dot;         /* 左右轮平均滚动纵向里程计速度，单位 m/s。 */
    float hip_odometry_s_raw;           /* 未减零点的髋点 O 纵向里程计，单位 m。 */
    float left_hip_odometry_s_dot;      /* 左腿推得的髋点 O 纵向速度，单位 m/s。 */
    float right_hip_odometry_s_dot;     /* 右腿推得的髋点 O 纵向速度，单位 m/s。 */

    float state_vector[WHEEL_LEGGED_STATE_COUNT]; /* 固定顺序的整车十维状态。 */
    float s_origin;          /* 采集零点时的纯轮式纵向里程计原始值，单位 m。 */
    float yaw_origin;        /* 采集零点时的 IMU yaw，单位 rad。 */
    float pitch_origin;      /* 采集零点时的 IMU pitch，单位 rad。 */
    uint16_t valid_mask;     /* 本周期有效的状态来源位掩码。 */
    uint8_t origin_captured; /* 已在完整有效反馈下采集状态零点时为 1。 */
    uint32_t update_count;   /* 已执行的状态更新次数。 */
} WheelLeggedChassisState_t;

/* 底盘状态坐标和世界系腿角变换参数。 */
typedef struct
{
    float yaw_direction;                /* IMU yaw 正方向到 phi 正方向的符号，取 +1 或 -1。 */
    float body_pitch_direction;         /* IMU pitch 正方向到 theta_body 正方向的符号，取 +1 或 -1。 */
    float left_leg_relative_direction;  /* 左腿 FK 相对角到建模相对角的符号，取 +1 或 -1。 */
    float right_leg_relative_direction; /* 右腿 FK 相对角到建模相对角的符号，取 +1 或 -1。 */
    float leg_world_body_pitch_gain;    /* theta_body 对世界系腿角的系数，通常取 +1 或 -1。 */
    float left_leg_world_offset;        /* 左腿世界系摆角常量偏置，单位 rad。 */
    float right_leg_world_offset;       /* 右腿世界系摆角常量偏置，单位 rad。 */
} WheelLeggedChassisStateConfig_t;

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

/* 一条腿的 VMC 命令和实时映射结果；力矩下发只允许经过 chassis_output.c。 */
typedef struct
{
    float force_command; /* F：虚拟腿轴向广义力命令，单位 N；正值表示伸腿、撑开髋点与轮轴。 */
    float pitch_torque_command; /* Tp：虚拟腿摆动广义力矩命令，单位 N·m；正值使虚拟腿角增大。 */

    float phi1_torque; /* 映射得到的 phi1 主动轴目标力矩，单位 N·m。 */
    float phi2_torque; /* 映射得到的 phi2 主动轴目标力矩，单位 N·m。 */

    float front_motor_torque; /* 映射到前关节电机轴的目标力矩，单位 N·m。 */
    float rear_motor_torque;  /* 映射到后关节电机轴的目标力矩，单位 N·m。 */

    float front_motor_torque_output; /* 经过测试限幅后实际写入前关节电机的力矩参考，单位 N·m。 */
    float rear_motor_torque_output; /* 经过测试限幅后实际写入后关节电机的力矩参考，单位 N·m。 */

    float leg_jacobian_det; /* 当前真实末端 J 的腿雅可比行列式，工程单位 m/rad^2，用于观察接近奇异的位置。 */
    uint8_t valid; /* FK、雅可比、反馈和传动换算均有效时为 1。 */
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

/* 一条腿的运行实例：两个主动关节和一个纯同心五连杆模型。 */
typedef struct
{
    WheelLeggedJointInstance_t front_joint; /* 前主动关节。 */
    WheelLeggedJointInstance_t rear_joint;  /* 后主动关节。 */

    LegKinematicsInput_e front_joint_kinematics_input; /* 前关节反馈送入 phi1 或 phi2。 */
    LegKinematicsInput_e rear_joint_kinematics_input;  /* 后关节反馈送入 phi1 或 phi2。 */

    ParallelLegConfig_t kinematics_runtime_config; /* 由链轮齿数生成的机构学运行配置。 */
    ParallelLegInstance_t kinematics;              /* 执行器角到轮轴状态的纯机构学实例。 */
    WheelLeggedLegLengthControl_t length_control;  /* 本腿独立的腿长 PID 运行态、测量和力命令。 */
    WheelLeggedLegVmc_t vmc;                       /* 本腿 VMC 命令和实时计算状态。 */
    volatile uint32_t update_count;                /* 该腿已完成的更新次数。 */
    volatile uint32_t sequence;                    /* 奇数表示写入中，偶数表示写入完成。 */
} WheelLeggedLegInstance_t;

/* 轮毂电机在底盘对象中的运行实例。 */
typedef struct
{
    DMMotorInstance *motor; /* H6215 达妙电机实例；初始化失败时为 NULL。 */
    float wheel_radius;     /* 轮半径，单位 m。 */
    float reduction_ratio;  /* 电机轴转角与轮端转角的减速比。 */
    float direction;        /* 电机反馈正方向对应轮端前进正方向时为 +1，反向时为 -1。 */
    uint8_t configured;     /* 轮径、减速比和轮端方向均已标定时为 1。 */
    uint8_t feedback_ready; /* 本周期已收到轮毂有效反馈时为 1。 */
    float torque_command;       /* LQR 请求并经过轮端限幅的轮端力矩，单位 N*m。 */
    float motor_torque_output;  /* 经变化率限制后实际写入电机的力矩，单位 N*m。 */
    float previous_motor_torque; /* 上一周期实际写入的电机力矩，单位 N*m。 */
} WheelLeggedWheelInstance_t;

/* 一台主动关节的电机、传动和主动轴映射初始化配置。 */
typedef struct
{
    Motor_Init_Config_s motor_config;                  /* 达妙电机、CAN、控制器和方向配置。 */
    WheelLeggedChainTransmissionConfig_t chain_config; /* 电机轴到主动轴的链传动和零位配置。 */
    LegKinematicsInput_e kinematics_input;             /* 本关节接入 phi1 或 phi2。 */
} WheelLeggedJointInitConfig_t;

/* 一条腿的硬件和机构学初始化配置。 */
typedef struct
{
    WheelLeggedJointInitConfig_t front_joint;               /* 前关节完整配置。 */
    WheelLeggedJointInitConfig_t rear_joint;                /* 后关节完整配置。 */
    const ParallelLegGeometry_t *geometry_config;           /* 对应腿的同心五连杆几何配置。 */
    WheelLeggedLegLengthControlInitConfig_t length_control; /* 本腿默认关闭的腿长 PID 初始化配置。 */
} WheelLeggedLegInitConfig_t;

/* 一台轮毂的电机、轮端尺寸和正方向初始化配置。 */
typedef struct
{
    Motor_Init_Config_s motor_config; /* H6215 电机、CAN、控制器和方向配置。 */
    float wheel_radius;               /* 轮半径，单位 m。 */
    float reduction_ratio;            /* 电机轴到轮端的减速比。 */
    float direction;    /* 电机反馈正方向对应轮端前进正方向时为 +1，反向时为 -1。 */
    uint8_t configured; /* 轮端方向已通过手动推车验证时为 1。 */
} WheelLeggedWheelInitConfig_t;

/* LQR 四输入首次小量闭环的输出、安全和重力前馈配置。 */
typedef struct
{
    float supported_body_mass;       /* 由双腿支撑的机身质量，单位 kg；不含双腿、双轮和电池。 */
    float pitch_torque_limit;        /* 单腿虚拟摆动广义力矩 Tp 的绝对限幅，单位 N*m。 */
    float wheel_torque_limit;        /* 单个 H6215 轮端力矩绝对限幅，单位 N*m。 */
    float wheel_torque_rate_limit;   /* 单个 H6215 轮端力矩最大变化率，单位 N*m/s。 */
    float minimum_support_projection; /* cos(theta_L)+cos(theta_R) 的最小允许值。 */
} WheelLeggedChassisLqrOutputConfig_t;

/* 底盘未使能或被安全仲裁拦截时的原因位；允许多个原因同时置位。 */
typedef enum
{
    WHEEL_LEGGED_OUTPUT_BLOCK_NONE = 0u,                    /* 当前没有阻断原因。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_REMOTE_OFF = (1u << 0),      /* 右拨杆不在中档或底盘未请求 CHASSIS_ON。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_FAULT_LOCKED = (1u << 1),    /* 已出力后发生失效，当前处于故障锁定。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_ORIGIN = (1u << 2),           /* 状态零点尚未捕获。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_STATE = (1u << 3),            /* 状态来源 valid_mask 不完整。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_WHEEL_FEEDBACK = (1u << 4),  /* 左右轮反馈尚未准备好。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_LQR = (1u << 5),              /* LQR 本周期无效。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_LENGTH_CONTROL = (1u << 6),  /* 一侧或两侧腿长控制无效。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_VMC = (1u << 7),              /* 一侧或两侧 VMC 无效。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_CONFIG = (1u << 8),           /* 输出安全配置非法。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_MOTOR_INSTANCE = (1u << 9),  /* 六台电机实例不完整。 */
    WHEEL_LEGGED_OUTPUT_BLOCK_NONFINITE = (1u << 10),      /* 输出或力命令出现非有限数。 */
} WheelLeggedChassisOutputBlockReason_e;

/* 底盘初始化配置；由 robot 层提供本车的电机、传动和几何参数。 */
typedef struct
{
    WheelLeggedLegInitConfig_t *left_leg_init_config;      /* 左腿完整初始化配置。 */
    WheelLeggedLegInitConfig_t *right_leg_init_config;     /* 右腿完整初始化配置。 */
    WheelLeggedWheelInitConfig_t *left_wheel_init_config;  /* 左轮完整初始化配置。 */
    WheelLeggedWheelInitConfig_t *right_wheel_init_config; /* 右轮完整初始化配置。 */
    IMU_Init_Config_s *imu_init_config;                    /* 底盘 IMU 初始化配置。 */
    const WheelLeggedChassisStateConfig_t *state_config;   /* 十维状态坐标和符号配置。 */
    const WheelLeggedChassisLqrOutputConfig_t *lqr_output_config; /* LQR 输出和安全配置。 */
} WheelLeggedChassisInitConfig_t;

/* 底盘对象：具名左右字段保证 J-Link 可直接看出左右腿和左右轮。 */
typedef struct
{
    WheelLeggedChassisControlCommand_t chassis_ctrl_cmd; /* 上层下发的底盘命令。 */
    WheelLeggedChassisState_t chassis_state;             /* 底盘十维状态和调试量。 */
    WheelLeggedChassisLqr_t lqr;                         /* 十维 LQR 的实时计算结果。 */
    WheelLeggedLegInstance_t left_leg;                   /* 左腿。 */
    WheelLeggedLegInstance_t right_leg;                  /* 右腿。 */
    WheelLeggedWheelInstance_t left_wheel;               /* 左轮毂。 */
    WheelLeggedWheelInstance_t right_wheel;              /* 右轮毂。 */
    INS_t *imu;                                          /* 底盘 IMU 数据；仅用于状态读取。 */
    WheelLeggedChassisStateConfig_t state_config;        /* 底盘状态坐标和符号配置。 */
    WheelLeggedChassisLqrOutputConfig_t lqr_output_config; /* LQR 输出和安全配置。 */
    uint32_t output_dwt_count;                           /* 电机输出斜率限制使用的 DWT 时间基准。 */
    float output_dt;                                     /* 本周期输出仲裁实际使用的时间间隔，单位 s。 */
    uint8_t motor_enabled;                               /* 上次实际下发的六台底盘电机使能状态。 */
    uint8_t output_ever_enabled;                         /* 本次中档周期内曾成功闭环出力时为 1。 */
    uint8_t output_fault_locked;                         /* 已出力后失效时锁定；离开中档后才允许清除。 */
    uint16_t output_block_reason;                         /* 本周期未使能的原因位掩码，0 表示没有阻断。 */
} WheelLeggedChassisInstance_t;

void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis, WheelLeggedChassisInitConfig_t *init_config);
void ChassisTask(WheelLeggedChassisInstance_t *chassis);
void WheelLeggedChassisStateResetOrigin(WheelLeggedChassisInstance_t *chassis);
