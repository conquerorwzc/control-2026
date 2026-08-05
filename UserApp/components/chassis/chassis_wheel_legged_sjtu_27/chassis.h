/**
 ******************************************************************************
 * @file    chassis.h
 * @brief   SJTU 上层流程到 27demo ACE 底盘的安全适配接口
 ******************************************************************************
 */
#pragma once

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

#include "../chassis_wheel_legged_double_closed_loop/chassis.h"
#include "super_cap.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* 保留 SJTU 上层使用的模式枚举；只有 CHASSIS_ON 有机会进入新底盘闭环。 */
typedef enum
{
    SJTU27_CHASSIS_POWER_OFF = 0,
    SJTU27_CHASSIS_RECOVERY,
    SJTU27_CHASSIS_ON,
    SJTU27_CHASSIS_JUMP_READY,
    SJTU27_CHASSIS_JUMP_START,
    SJTU27_CHASSIS_PROSTRATE,
    SJTU27_CHASSIS_STAIR,
} Chassis_Mode_e;

/* core 已占用 CHASSIS_POWER_OFF/CHASSIS_ON 名称，兼容层在其后提供旧 SJTU 数值合同。 */
#define CHASSIS_POWER_OFF SJTU27_CHASSIS_POWER_OFF
#define CHASSIS_RECOVERY SJTU27_CHASSIS_RECOVERY
#define CHASSIS_ON SJTU27_CHASSIS_ON
#define CHASSIS_JUMP_READY SJTU27_CHASSIS_JUMP_READY
#define CHASSIS_JUMP_START SJTU27_CHASSIS_JUMP_START
#define CHASSIS_PROSTRATE SJTU27_CHASSIS_PROSTRATE
#define CHASSIS_STAIR SJTU27_CHASSIS_STAIR

/* 保留 SJTU 跳跃状态观察接口；本适配层当前只发布 IDLE。 */
typedef enum
{
    JUMP_STATE_IDLE = 0,
    JUMP_STATE_COMPRESS,
    JUMP_STATE_EXTEND,
    JUMP_STATE_RETRACT,
} Jump_State_e;

/* SJTU UI/comm 的超电模式兼容枚举；当前不参与 27demo 的电机输出。 */
typedef enum
{
    SAFETY_MODE = 0,
    PASSIVE_MODE,
    ACTIVE_MODE,
    CHARGING_MODE,
} SuperCap_Mode_e;

/* SJTU UI/comm 的超电请求兼容枚举；当前仅保留通信状态。 */
typedef enum
{
    NORMAL = 0,
    BOOST,
} SuperCap_Ctrl_Cmd_e;

/* 不向旧超电模块写命令的兼容状态，避免把未校验的功率路径接入本车。 */
typedef struct
{
    SuperCap_Measure_s cap_msg;            /* 供 UI 显示的电压、功率和错误状态。 */
    SuperCap_Mode_e super_cap_mode;        /* 当前固定为 SAFETY_MODE。 */
    SuperCap_Ctrl_Cmd_e super_cap_ctrl_cmd; /* SJTU 上层通信请求。 */
} WheelLeggedSjtu27SuperCap_t;

/* SJTU ctrl/comm 的原始命令合同；字段保留，但尚未接入的模式不会产生电机输出。 */
typedef struct
{
    float vx;                 /* 纵向速度目标，单位 m/s。 */
    float wz;                 /* 偏航角速度目标，单位 rad/s。 */
    float target_yaw;         /* 偏航角目标，单位 rad。 */
    float roll;               /* 横滚目标，当前仅保留通信语义。 */
    float roll_ff;            /* 横滚前馈，当前仅保留通信语义。 */
    float leg_length;         /* 双腿共同长度目标，单位 m。 */
    float jump_force;         /* 跳跃力，当前不接入。 */
    float theta_ff;           /* 腿角前馈，当前不接入。 */
    int chassis_speed_buff;   /* SJTU 上层速度缓冲字段。 */
    uint16_t max_power;       /* 裁判系统底盘功率上限。 */
    Chassis_Mode_e chassis_mode; /* SJTU 上层请求模式。 */
    uint8_t SuperCapBoost;    /* 超级电容请求。 */
    uint8_t is_rotate;        /* 小陀螺请求。 */
} Chassis_Ctrl_Cmd_s;

/* SJTU 上层可读的十维状态别名；数值每周期由 27demo chassis_state 同步。 */
typedef struct
{
    float x_b;       /* 对应纯轮式 s，单位 m。 */
    float x_b_d;     /* 对应纯轮式 s_dot，单位 m/s。 */
    float phi;       /* 偏航角，单位 rad。 */
    float phi_d;     /* 偏航角速度，单位 rad/s。 */
    float theta_l;   /* 左腿纵向平面角，单位 rad。 */
    float theta_l_d; /* 左腿纵向平面角速度，单位 rad/s。 */
    float theta_r;   /* 右腿纵向平面角，单位 rad。 */
    float theta_r_d; /* 右腿纵向平面角速度，单位 rad/s。 */
    float theta_b;   /* 机身俯仰角，单位 rad。 */
    float theta_b_d; /* 机身俯仰角速度，单位 rad/s。 */
} State_Var_t;

/* 当前车辆重新测量后才允许打开 LQR 的动力学参数集合。 */
typedef struct
{
    uint8_t configured;          /* 所有质量、惯量、质心、轨距和专属 K 均已复核时置 1。 */
    float body_mass;             /* 机身质量，单位 kg。 */
    float leg_mass;              /* 单条等效腿质量，单位 kg。 */
    float wheel_mass;            /* 单个轮质量，单位 kg。 */
    float body_pitch_inertia;    /* 机身绕俯仰轴转动惯量，单位 kg*m^2。 */
    float leg_inertia;           /* 单条等效腿转动惯量，单位 kg*m^2。 */
    float wheel_inertia;         /* 单个轮转动惯量，单位 kg*m^2。 */
    float yaw_inertia;           /* 整车绕 yaw 轴转动惯量，单位 kg*m^2。 */
    float leg_com_position;      /* 等效腿质心相对轮轴的距离，单位 m。 */
    float track_width;           /* 左右轮中心距，单位 m。 */
    uint8_t lqr_coefficients_configured; /* 本车参数重新生成并导出 K 后置 1。 */
} WheelLeggedSjtu27ModelConfig_t;

/* 新底盘适配层初始化配置：27demo 底层配置与 SJTU 上层公共配置分离。 */
typedef struct
{
    WheelLeggedChassisInitConfig_t wheel_legged_init_config; /* 当前 ACE/J4310/H6215 配置。 */
    WheelLeggedSjtu27ModelConfig_t model_config;             /* 本车专属动力学/LQR 许可。 */
    float initial_leg_length;                                /* SJTU 上层起始腿长，单位 m。 */
    float leg_min_length;                                    /* 上层命令最小腿长，单位 m。 */
    float leg_max_length;                                    /* 上层命令最大腿长，单位 m。 */
    SuperCap_Init_Config_s super_cap_config;                 /* 保留 SJTU 通信/UI 的超电对象。 */
} Chassis_Init_Config_s;

/* 旧 SJTU 外部对象保持为兼容视图；core 是唯一实际 FK/VMC/PID/输出拥有者。 */
typedef struct
{
    Jump_State_e jump_state;                         /* 当前保持 IDLE，跳跃底层尚未移植。 */
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;             /* SJTU ctrl/comm 写入的原始命令。 */
    State_Var_t state_var;                           /* 从 core.chassis_state 同步的兼容状态。 */
    State_Var_t last_state_var;                      /* 上周期兼容状态。 */
    WheelLeggedChassisInstance_t core;               /* 唯一实际底盘对象。 */
    WheelLeggedLegInstance_t *leg[2];                /* [0]=右腿、[1]=左腿，兼容旧 UI 顺序。 */
    INS_t *imu;                                      /* 指向 core.imu。 */
    WheelLeggedSjtu27SuperCap_t *super_cap;          /* SJTU comm/ui 兼容对象，不参与电机仲裁。 */
    WheelLeggedSjtu27ModelConfig_t model_config;     /* 车辆动力学配置快照。 */
    uint8_t model_ready;                             /* 动力学参数与本车 K 均完成时为 1。 */
    uint8_t lqr_valid;                               /* 仅 model_ready 后转发 core.lqr.valid。 */
    struct
    {
        uint8_t gimbal_aligned : 1; /* 保留双板同步状态。 */
    } update_flag;
} ChassisInstance;

/**
 * @brief 初始化 SJTU 上层兼容对象与当前 27demo 底盘内核。
 *
 * @param init_config robot_config.h 提供的适配层配置。
 * @return 成功分配后的兼容底盘对象；失败返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *init_config);

/**
 * @brief 执行一次 27demo 底盘更新，并同步 SJTU 上层可见状态。
 *
 * @param chassis 由 ChassisInit 返回的兼容底盘对象。
 */
void WheelLeggedSjtu27ChassisTask(ChassisInstance *chassis);
