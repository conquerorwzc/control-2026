#pragma once

#include <math.h>
#include <stddef.h>

#include "fdcan.h"
#include "robot.h"
#include "usart.h"

#define SYSID_RC_PORT (&huart5)
#define SYSID_VOFA_PORT (&huart1)
#define SYSID_VOFA_BAUD 1000000u

// 台架标定完成之前保持 0，空跑不注册任何执行器。
#ifndef SYSID_REAL_HARDWARE
#define SYSID_REAL_HARDWARE 0
#endif
#ifndef SYSID_EXPERIMENT
#define SYSID_EXPERIMENT SYSID_HIP
#endif
#ifndef SYSID_HIP_PATTERN
#define SYSID_HIP_PATTERN SYSID_HIP_FRONT
#endif

typedef struct
{
    float min_rad;
    float max_rad;
    float max_speed_rad_s;
    float max_feedback_torque_nm;
    float max_temperature_c;
    float start_error_rad;
    float start_speed_rad_s;
    float start_command_torque_nm;
} SysIdHipLimits;

typedef struct
{
    float max_speed_rad_s;
    float max_feedback_current_raw;
    float max_temperature_c;
    float start_speed_rad_s;
} SysIdWheelLimits;

typedef struct
{
    float hip_sweep_rad;
    float hip_step_rad;
    float wheel_sweep_rad_s;
    float triangle_rad;
    float hip_ramp_nm_s;
    float hip_ramp_max_nm;
    float hip_motion_rad_s;
    float wheel_ramp_a_s;
    float wheel_ramp_max_a;
    float wheel_motion_rad_s;
} SysIdAmplitude;

typedef struct
{
    float sweep_start_hz;
    float sweep_end_hz;
    float sweep_duration_s;
    float sweep_ramp_s;
    float step_baseline_s;
    float step_hold_s;
    float step_recovery_s;
    float steady_s;
    float coast_s;
    float hold_average_s;
    float motion_confirmation_s;
    unsigned triangle_cycles;
} SysIdTiming;

// 在实机试验中选择接入的侧。左右双腿反相试验需要 0x3 和足够的 CAN 滤槽。
#ifndef SYSID_TEST_SIDES
#define SYSID_TEST_SIDES (1u << CHASSIS_LEFT)
#endif

#ifndef SYSID_TRIAL_DIRECTION
#define SYSID_TRIAL_DIRECTION 1
#endif

#define SYSID_IS_HIP(trial)                                                                                            \
    ((trial) == SYSID_HIP || (trial) == SYSID_HIP_STEP || (trial) == SYSID_HIP_RAMP || (trial) == SYSID_HIP_TRIANGLE)

// 每次选择一个工作点，同时记录编号和各关节中心。TODO 按短、中、长腿及不同摆角测定。
static const unsigned kWorkpointId = 1u;
static const float kCenterRad[CHASSIS_JOINT_COUNT] = {
    [2 * CHASSIS_LEFT + CHASSIS_FRONT] = NAN,
    [2 * CHASSIS_LEFT + CHASSIS_REAR] = NAN,
    [2 * CHASSIS_RIGHT + CHASSIS_FRONT] = NAN,
    [2 * CHASSIS_RIGHT + CHASSIS_REAR] = NAN,
};

// §4.4 扫频范围。时长和进出沿为本固件的试验编排，可按计划调整。
static const SysIdTiming kTrialTiming = {
    .sweep_start_hz = 0.5f,
    .sweep_end_hz = 5.0f,
    .sweep_duration_s = 20.0f,
    .sweep_ramp_s = 1.0f,
    .step_baseline_s = 1.0f,
    .step_hold_s = 3.0f,
    .step_recovery_s = 1.0f,
    .steady_s = 5.0f,
    .coast_s = 5.0f,
    .hold_average_s = 0.25f,
    .motion_confirmation_s = 0.1f,
    .triangle_cycles = 3u,
};

static const SysIdAmplitude kPreviewAmplitude = {
    .hip_sweep_rad = 0.05f,
    .hip_step_rad = 0.05f,
    .wheel_sweep_rad_s = 10.0f,
    .triangle_rad = 0.1f,
    .hip_ramp_nm_s = 0.1f,
    .hip_ramp_max_nm = 0.5f,
    .wheel_ramp_a_s = 0.2f,
    .wheel_ramp_max_a = 1.0f,
};

static const SysIdAmplitude kRealAmplitude = {
    .hip_sweep_rad = 0.05f,    // §4.4 起点，按已确认的工作点行程和试验计划调整
    .hip_step_rad = 0.1f,      // §4.4 起点
    .wheel_sweep_rad_s = 0.0f, // TODO 台架轮扫频幅值
    .triangle_rad = 0.0f,      // TODO 各工作点的三角轨迹幅值
    .hip_ramp_nm_s = 0.0f,     // TODO 起动缓升斜率、增量上限和高于编码器噪声的速度阈值
    .hip_ramp_max_nm = 0.0f,
    .hip_motion_rad_s = 0.0f,
    .wheel_ramp_a_s = 0.0f, // TODO 空载电流斜率、上限和高于反馈噪声的起转速度阈值
    .wheel_ramp_max_a = 0.0f,
    .wheel_motion_rad_s = 0.0f,
};

// 底盘只为当前试验注册相关执行器。按腿分总线，左 CAN1、右 CAN2。
static const Chassis_Init_Config_s
    kSysIdChassisConfig =
        {
            .joint_mask = SYSID_IS_HIP(SYSID_EXPERIMENT)
                              ? (((SYSID_TEST_SIDES & 1u) ? 0x3u : 0u) | ((SYSID_TEST_SIDES & 2u) ? 0xCu : 0u))
                              : 0u,
            .wheel_mask = SYSID_IS_HIP(SYSID_EXPERIMENT) ? 0u : SYSID_TEST_SIDES,
            .feedback_timeout_ms = 0u, // TODO 按反馈周期与失联实测设定
            .joint =
                {
                    // OK 左前
                    [2 * CHASSIS_LEFT + CHASSIS_FRONT] =
                        {
                            .motor = {.motor_type = J8009P,
                                      .can_init_config = {.can_handle = &hcan1, .tx_id = 0x01, .rx_id = 0x11}},
                            .feedback_id = 1u,
                            .zero_rad = -1.731753157f,
                            .direction = -1.0f,
                            .motor_per_joint = 1.0f,
                            .kp_nm_per_rad = 10.0f, // §4.4 低增益起点，可换档复测
                            .kd_nm_per_rad_s = 0.5f,
                            .max_torque_nm = 0.0f, // TODO 辨识专用力矩上限
                        },
                    // OA 左后
                    [2 * CHASSIS_LEFT + CHASSIS_REAR] =
                        {
                            .motor = {.motor_type = J8009P,
                                      .can_init_config = {.can_handle = &hcan1, .tx_id = 0x02, .rx_id = 0x12}},
                            .feedback_id = 2u,
                            .zero_rad = 1.106499972f,
                            .direction = -1.0f,
                            .motor_per_joint = 1.0f,
                            .kp_nm_per_rad = 10.0f, // §4.4 低增益起点，可换档复测
                            .kd_nm_per_rad_s = 0.5f,
                            .max_torque_nm = 0.0f, // TODO 辨识专用力矩上限
                        },
                    [2 * CHASSIS_RIGHT + CHASSIS_FRONT] =
                        {
                            .motor = {.motor_type = J8009P,
                                      .can_init_config = {.can_handle = &hcan2, .tx_id = 0x03, .rx_id = 0x13}},
                            .feedback_id = 3u,
                            .zero_rad = NAN, // TODO 右前零位及方向
                            .motor_per_joint = 1.0f,
                            .kp_nm_per_rad = 10.0f, // §4.4 低增益起点，可换档复测
                            .kd_nm_per_rad_s = 0.5f,
                            .max_torque_nm = 0.0f, // TODO 辨识专用力矩上限
                        },
                    [2 * CHASSIS_RIGHT + CHASSIS_REAR] =
                        {
                            .motor =
                                {.motor_type = J8009P,
                                 .can_init_config = {.can_handle = &hcan2, .tx_id = 0x04, .rx_id = 0x14}},
                            .feedback_id = 4u,
                            .zero_rad = NAN, // TODO 右后零位及方向
                            .motor_per_joint = 1.0f,
                            .kp_nm_per_rad = 10.0f, // §4.4 低增益起点，可换档复测
                            .kd_nm_per_rad_s = 0.5f,
                            .max_torque_nm = 0.0f, // TODO 辨识专用力矩上限
                        },
                },
            .wheel =
                {
                    // M3508 + C620。TODO 台架确认方向、速度环增益 kp_a_per_rad_s 与 max_current_a。
                    [CHASSIS_LEFT] =
                        {
                            .motor = {.motor_type = M3508, .can_init_config = {.can_handle = &hcan1, .tx_id = 1}},
                            .motor_per_wheel = 15.76f,
                        },
                    [CHASSIS_RIGHT] =
                        {
                            .motor = {.motor_type = M3508, .can_init_config = {.can_handle = &hcan2, .tx_id = 2}},
                            .motor_per_wheel = 15.76f,
                        },
                },
            .geometry =
                {
                    .first_m = 0.210f,    // OK
                    .second_m = 0.24625f, // KW
                    .virtual_second_m = 0.13351f,
                    .singular_epsilon_m = 1.0e-6f,
                },
};

// TODO 各轴安全范围和起姿门槛。角度和转速均在逻辑坐标中。
static const SysIdHipLimits kHipLimits[CHASSIS_JOINT_COUNT] = {
    [2 * CHASSIS_LEFT + CHASSIS_FRONT] = {.min_rad = NAN},
    [2 * CHASSIS_LEFT + CHASSIS_REAR] = {.min_rad = NAN},
    [2 * CHASSIS_RIGHT + CHASSIS_FRONT] = {.min_rad = NAN},
    [2 * CHASSIS_RIGHT + CHASSIS_REAR] = {.min_rad = NAN},
};

// TODO 轮端超速、反馈电流原始计数、温度和启动转速门槛。
static const SysIdWheelLimits kWheelLimits[CHASSIS_LEG_COUNT] = {
    [CHASSIS_LEFT] = {.max_speed_rad_s = 0.0f},
    [CHASSIS_RIGHT] = {.max_speed_rad_s = 0.0f},
};
static const uint32_t kMaxControlGapUs = 0u; // TODO 按实测调度间隔设定
