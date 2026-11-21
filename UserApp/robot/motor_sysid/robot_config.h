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
    .slow = {.amplitude_rad = 0.05f,
             .frequency_start_hz = 0.5f,
             .frequency_end_hz = 0.5f,
             .duration_s = 8.0f,
             .ramp_s = 1.0f},
    .chirp = {.amplitude_rad = 0.05f,
              .frequency_start_hz = 0.5f,
              .frequency_end_hz = 5.0f,
              .duration_s = 20.0f,
              .ramp_s = 1.0f},
    .hold = {.amplitude_rad = 0.05f, .duration_s = 5.0f, .ramp_s = 1.0f},
    .min_angle_rad = {-0.1f, -0.1f},
    .max_angle_rad = {0.1f, 0.1f},
    .dry_run = true,
};

// 实机专用标定。默认 0：绝不注册 DM。先填写表并核对台架电源急停，再允许真机模式。
// measured_mask: 0x3=只接左腿，0xC=只接右腿，0xF=四路全部采集；选侧由本次试验配置指定。
// 两关节若分别使用两路总线，现有 H723 每路一个标准滤槽即可；必须与实际接线相符。
#ifndef SYSID_REAL_HIPS
#define SYSID_REAL_HIPS 0
#endif
#include "dmmotor.h"
#include "sysid_hip.h"
#include <math.h>

typedef struct
{
    FDCAN_HandleTypeDef *bus;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t feedback_id; // 反馈载荷低四位，不从 CAN 帧 ID 推断
    int8_t sign;         // 正负方向同时用于队内驱动的命令/反馈翻转，不再乘第二遍
    float zero_rad;      // 驱动完成方向变换后的输出端零位
} SysIdHipWiring;

static const SysIdHipWiring kHipWiring[SYSID_HIP_COUNT] = {0}; // 左前、左后、右前、右后
static const SysIdHip_Init_Config_s kHipControl = {0};
static const SysIdHipSide kTestSide = SYSID_HIP_SIDE_NONE;
static const int8_t kRealDirection[2] = {0, 0};          // 每路 +1 / -1，在实机标定后填写
static const uint32_t kFeedbackTimeoutUs = 0;            // 从 CAN 反馈周期和失联试验确定
static const uint32_t kMaxControlGapUs = 0;              // 允许的最长控制拍间隔
static const SysIdRunner_Init_Config_s kRealTrial = {0}; // 三种轨迹、两主动关节安全角区间，dry_run=false
