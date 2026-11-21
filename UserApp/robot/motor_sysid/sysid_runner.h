#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sysid_signal.h"

enum
{
    SYSID_LEG_COUNT = 2,
    SYSID_RUNNER_PERIOD_US = 2000
};

// 由 robot.c 解码物理位置，不直接传遥控协议的原始数值。
typedef enum
{
    SYSID_SWITCH_INVALID = 0,
    SYSID_SWITCH_DOWN,
    SYSID_SWITCH_MIDDLE,
    SYSID_SWITCH_UP
} SysIdSwitch;

typedef enum
{
    SYSID_MODE_NONE = 0,
    SYSID_MODE_SLOW,  // 右上：慢速正弦往返
    SYSID_MODE_CHIRP, // 右中：线性扫频
    SYSID_MODE_HOLD   // 右下：余弦进出沿、平滑平台
} SysIdMode;

typedef enum
{
    SYSID_STATE_NEED_ESTOP = 0,
    SYSID_STATE_ESTOP,
    SYSID_STATE_READY,
    SYSID_STATE_STARTING,
    SYSID_STATE_RUNNING,     // 唯一允许调用方使用试验参考计算 PD 的状态
    SYSID_STATE_FINISHED,
    SYSID_STATE_FAULT,
    SYSID_STATE_DRY_RUNNING  // 仅生成标量，不允许出力
} SysIdState;

typedef struct
{
    float amplitude_rad;
    float frequency_start_hz; // slow 使用；hold 忽略频率
    float frequency_end_hz;   // 仅 chirp 使用
    float duration_s;         // 包含两端 ramp
    float ramp_s;
} SysIdTrialConfig;

typedef struct
{
    SysIdTrialConfig slow;
    SysIdTrialConfig chirp;
    SysIdTrialConfig hold;
    float min_angle_rad[SYSID_LEG_COUNT];
    float max_angle_rad[SYSID_LEG_COUNT];
    bool dry_run; // 固定到下一次 Init；true 时永不返回 RUNNING
} SysIdRunner_Init_Config_s;

typedef struct
{
    bool remote_online;
    SysIdSwitch left;
    SysIdSwitch right;
    bool feedback_valid;
    bool synthetic_input_valid; // 仅 dry_run 接受；不是电机/传感器反馈有效标志
    uint64_t now_us;             // DWT_GetTimeline_us()，必须严格递增
    // 按调用方约定的同一腿顺序。STARTING 锁存，直到本轮结束不得改变。
    float center_rad[SYSID_LEG_COUNT];
    int8_t direction[SYSID_LEG_COUNT]; // 各为 +1 或 -1
} SysIdRunnerInput;

typedef struct
{
    SysIdMode mode;
    SysIdState state;
    float excitation_rad; // 非 RUNNING / DRY_RUNNING 时为 0。
} SysIdRunnerOutput;

// 实例由调用方静态分配；以下字段仅供 runner 使用。
typedef struct
{
    const SysIdRunner_Init_Config_s *config;
    SysIdState state;
    SysIdMode mode;
    SysIdSignalConfig active_signal;
    uint64_t previous_us;
    uint64_t start_us;
    bool have_previous_time;
    float center_rad[SYSID_LEG_COUNT];
    int8_t direction[SYSID_LEG_COUNT];
} SysIdRunnerInstance;

// config 由 robot_config 提供，须在实例使用期间存活且保持不变。返回 false 后须重新 Init。
bool SysIdRunnerInit(SysIdRunnerInstance *runner, const SysIdRunner_Init_Config_s *config);

// 由 500 Hz robot 任务调用；波形按 now_us 差值计算。合法序列：左下 -> 左中 -> 左上，
// 左上的首拍只进入 STARTING，且右档必须与最后一次左中采样一致。
// 仅 RUNNING 可将 center_rad[i] + direction[i] * excitation_rad 送给外部 PD；其余状态须禁止出力。
// runner 不访问硬件，也不替代反馈新鲜度、实测关节角、速度和力矩等外部安全检查。
SysIdRunnerOutput SysIdRunnerStep(SysIdRunnerInstance *runner, const SysIdRunnerInput *input);
