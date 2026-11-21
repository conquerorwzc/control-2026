#pragma once

#include "gen_config.h"
#include "sysid_runner.h"
#include "usart.h"

// 仅本 robot 使用；不得用此处的空跑幅值和坐标作为上车标定。
#define SYSID_RC_PORT (&huart5)
#define SYSID_VOFA_PORT (&huart1)
#define SYSID_VOFA_BAUD 1000000u
#define SYSID_DAEMON_TICK_US 10000u
#define SYSID_VOFA_CHANNELS 24u

static const float kPreviewCenterRad[2] = {0.0f, 0.0f};
static const int8_t kPreviewDirection[2] = {1, -1};
static const SysIdRunner_Init_Config_s kSysIdConfig = {
    .slow = {.amplitude_rad = 0.05f, .frequency_start_hz = 0.5f, .frequency_end_hz = 0.5f,
             .duration_s = 8.0f, .ramp_s = 1.0f},
    .chirp = {.amplitude_rad = 0.05f, .frequency_start_hz = 0.5f, .frequency_end_hz = 5.0f,
              .duration_s = 20.0f, .ramp_s = 1.0f},
    .hold = {.amplitude_rad = 0.05f, .duration_s = 5.0f, .ramp_s = 1.0f},
    .min_angle_rad = {-0.1f, -0.1f}, .max_angle_rad = {0.1f, 0.1f},
    .dry_run = true,
};
