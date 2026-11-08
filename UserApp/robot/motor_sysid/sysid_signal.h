#pragma once
#include <stdbool.h>

typedef enum
{
    SYSID_SIGNAL_HOLD,
    SYSID_SIGNAL_SINE,
    SYSID_SIGNAL_CHIRP
} SysIdSignalType;

typedef struct
{
    SysIdSignalType type;
    float offset; // 与被测输入同单位：rad / rad/s / N.m，由调用方明确
    float amplitude;
    float frequency_start_hz;
    float frequency_end_hz;
    float duration_s;
    float ramp_s; // 进入/退出激励的余弦包络，避免扫频端点阶跃
    float sample_period_s;
} SysIdSignalConfig;

// 仅数学边界校验；关节/速度/力矩约束由后续应用安全层负责。
bool SysIdSignalValid(const SysIdSignalConfig *config);
// 调用前必须通过 Valid；窗口外返回 offset。不驱动电机。
float SysIdSignalValue(const SysIdSignalConfig *config, float elapsed_s);
