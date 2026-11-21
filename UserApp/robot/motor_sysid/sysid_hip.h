#pragma once

#include "sysid_runner.h"

enum
{
    SYSID_HIP_LEFT_FRONT = 0,
    SYSID_HIP_LEFT_REAR,
    SYSID_HIP_RIGHT_FRONT,
    SYSID_HIP_RIGHT_REAR,
    SYSID_HIP_COUNT
};

enum
{
    SYSID_HIP_MASK_LEFT = (1u << SYSID_HIP_LEFT_FRONT) | (1u << SYSID_HIP_LEFT_REAR),
    SYSID_HIP_MASK_RIGHT = (1u << SYSID_HIP_RIGHT_FRONT) | (1u << SYSID_HIP_RIGHT_REAR),
    SYSID_HIP_MASK_ALL = SYSID_HIP_MASK_LEFT | SYSID_HIP_MASK_RIGHT
};

typedef enum
{
    SYSID_HIP_SIDE_NONE = 0,
    SYSID_HIP_SIDE_LEFT,
    SYSID_HIP_SIDE_RIGHT
} SysIdHipSide;

typedef enum
{
    SYSID_HIP_DISABLED = 0,
    SYSID_HIP_ARMED,
    SYSID_HIP_ACTIVE,
    SYSID_HIP_FAULT
} SysIdHipStatus;

typedef struct
{
    float q_min_rad;
    float q_max_rad;
    float center_rad;
    float kp_nm_per_rad;
    float kd_nm_per_rad_s;
    float max_abs_speed_rad_s;
    float max_abs_feedback_torque_nm;
    float max_abs_command_torque_nm;
    float max_temperature_c;
    // 起姿窗口；同时限制 STARTING 和首拍 RUNNING 的实际位置、参考位置及实际速度。
    float start_max_error_rad;
    float start_max_abs_speed_rad_s;
    // 首拍 RUNNING 的未限幅 PD 上限；0 表示只接受零起始力矩。
    float start_max_abs_torque_nm;
} SysIdHipJointConfig;

typedef struct
{
    unsigned measured_mask; // 必须为 MASK_LEFT、MASK_RIGHT 或 MASK_ALL；未配置通道不读、不出力。
    SysIdHipJointConfig joint[SYSID_HIP_COUNT]; // 左前、左后、右前、右后
} SysIdHip_Init_Config_s;

typedef struct
{
    bool valid; // 由调用方汇总该路在线、快照一致且未过期；本模块不访问时钟或总线。
    float q_rad;
    float qd_rad_s;
    float tau_nm;
    float temperature_c;
} SysIdHipFeedback;

typedef struct
{
    SysIdState state;
    SysIdHipSide side;
    SysIdHipFeedback feedback[SYSID_HIP_COUNT];
    float q_ref_rad[SYSID_LEG_COUNT]; // 选中侧前、后两个绝对关节角，不是电机轴角或偏移量。
} SysIdHipInput;

typedef struct
{
    SysIdHipStatus status;
    bool clipped; // 任一有效输出实际被限幅；全零故障结果不保留部分计算的标志。
    float torque_nm[SYSID_HIP_COUNT]; // 逻辑关节侧；非 ACTIVE 或非选中侧恒为零。
} SysIdHipOutput;

// 调用方分配，字段仅供本模块使用。
typedef struct
{
    const SysIdHip_Init_Config_s *config;
    SysIdHipStatus status;
    SysIdHipSide side;
} SysIdHipInstance;

bool SysIdHipConfigValid(const SysIdHip_Init_Config_s *config);

// 仅对 measured_mask 内的全部通道检查有效位、有限性和运行硬限位；不检查选侧、起姿或指令。
// true 不表示未配置通道有效；调用方应令未接通道的有效位保持 0。
bool SysIdHipFeedbackValid(const SysIdHip_Init_Config_s *config, const SysIdHipFeedback feedback[SYSID_HIP_COUNT]);

// 标定和反馈均已转换为同一个逻辑关节坐标系；不做零位、方向、减速比或驱动协议换算。
// config 须在实例使用期间存活且保持不变；Init 失败后须使用有效配置重新 Init。
bool SysIdHipInit(SysIdHipInstance *hip, const SysIdHip_Init_Config_s *config);

// 每拍都调用，包括禁用和故障拍；用当拍结果覆盖四路旧命令，不要保留前拍力矩。
// STARTING 仅锁定试验侧和检查起姿；同侧 RUNNING 才允许出力。试验侧须完全包含于 measured_mask。
// 故障全零并锁存至 ESTOP。
// 所有运行边界含等号；参考角越界拒绝，正常 PD 只对输出力矩限幅。后续参考连续性由调用方保证。
SysIdHipOutput SysIdHipStep(SysIdHipInstance *hip, const SysIdHipInput *input);
