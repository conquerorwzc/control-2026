#pragma once

#include <math.h>

#include "robot.h"
#include "usart.h"

#define SYSID_RC_PORT (&huart5)

#ifndef SYSID_REAL_HARDWARE
#define SYSID_REAL_HARDWARE 0
#endif
#ifndef SYSID_EXPERIMENT
#define SYSID_EXPERIMENT SYSID_HIP
#endif
#ifndef SYSID_HIP_PATTERN
#define SYSID_HIP_PATTERN SYSID_HIP_FRONT
#endif
#ifndef SYSID_TEST_SIDES
#define SYSID_TEST_SIDES (1u << CHASSIS_LEFT) // 左=1，右=2，双侧=3。
#endif
#ifndef SYSID_TRIAL_DIRECTION
#define SYSID_TRIAL_DIRECTION 1
#endif

#define SYSID_JOINT_MASK                                                                                               \
    (SYSID_IS_HIP(SYSID_EXPERIMENT) ? (((SYSID_TEST_SIDES & 1u) ? 0x3u : 0u) | ((SYSID_TEST_SIDES & 2u) ? 0xCu : 0u))  \
                                    : 0u)
#define SYSID_WHEEL_MASK (SYSID_IS_HIP(SYSID_EXPERIMENT) ? 0u : SYSID_TEST_SIDES)

// 队内 Motor_Init_Config_s。辨识控制使用 SetRef，驱动保持纯力矩/电流模式。
#define HIP_MOTOR_CONFIG(bus, id, master_id)                                                                           \
    {                                                                                                                  \
        .motor_type = J8009P, .can_init_config = {.can_handle = (bus), .tx_id = (id), .rx_id = (master_id) }           \
    }
#define WHEEL_MOTOR_CONFIG(bus, id)                                                                                    \
    {                                                                                                                  \
        .motor_type = M3508, .can_init_config = {.can_handle = (bus), .tx_id = (id) }                                  \
    }

#define HIP_SWEEP_AMPLITUDE_RAD 0.05f
#define HIP_STEP_RAD 0.1f
#define SWEEP_START_HZ 0.5f
#define SWEEP_END_HZ 5.0f
#define SWEEP_DURATION_S 20.0f
#define SWEEP_EDGE_S 1.0f
#define STEP_BASELINE_S 1.0f
#define STEP_HOLD_S 3.0f
#define STEP_RECOVERY_S 1.0f
#define WHEEL_STEADY_S 5.0f
#define WHEEL_COAST_S 5.0f
#define HIP_HOLD_AVERAGE_S 0.25f
#define MOTION_CONFIRMATION_S 0.1f
#define TRIANGLE_CYCLES 3u

// 空跑使用预览幅值。真实起动、三角往返和轮扫频的参数按台架填写。
#define HIP_TRIANGLE_RAD (SYSID_REAL_HARDWARE ? 0.0f : 0.1f)
#define HIP_RAMP_NM_S (SYSID_REAL_HARDWARE ? 0.0f : 0.1f)
#define HIP_RAMP_MAX_NM (SYSID_REAL_HARDWARE ? 0.0f : 0.5f)
#define HIP_MOTION_RAD_S 0.0f // TODO(待确认): 髋起动速度阈值。
#define WHEEL_SWEEP_RAD_S (SYSID_REAL_HARDWARE ? 0.0f : 10.0f)
#define WHEEL_RAMP_A_S (SYSID_REAL_HARDWARE ? 0.0f : 0.2f)
#define WHEEL_RAMP_MAX_A (SYSID_REAL_HARDWARE ? 0.0f : 1.0f)
#define WHEEL_MOTION_RAD_S 0.0f // TODO(待确认): 轮起转速度阈值。

// 顺序为左前 OK、左后 OA、右前、右后。中心角以已标定的逻辑坐标填写。
static const float kHipCenterRad[] = {-0.50282936f, 1.39661247f, NAN, NAN};
static const float kHipMinRad[] = {NAN, NAN, NAN, NAN}; // TODO(待确认): 各关节可用行程。
static const float kHipMaxRad[] = {NAN, NAN, NAN, NAN};
static const float kHipMaxSpeedRadS = 0.0f;      // TODO(待确认): 台架髋轴允许转速。
static const float kWheelMaxSpeedRadS = 0.0f;    // TODO(待确认): 离地空载允许轮速。
static const float kHipMaxTemperatureC = 0.0f;   // TODO(待确认): 髋电机停止温度。
static const float kWheelMaxTemperatureC = 0.0f; // TODO(待确认): 轮电机停止温度。
static const float kHipStartToleranceRad = 0.0f; // TODO(待确认): 接管姿态允许偏差。
static const float kHipStartSpeedRadS = 0.0f;    // TODO(待确认): 接管时髋轴应接近静止。
static const float kWheelStartSpeedRadS = 0.0f;  // TODO(待确认): 启动试验时允许的轮端速度。
static const float kWheelStepRadS[] = {10.0f, 30.0f, 60.0f};
static const float kTriangleSpeedRadS[] = {0.2f, 0.5f, 1.0f};

static Chassis_Init_Config_s chassis_init_config = {
    .joint_mask = SYSID_JOINT_MASK,
    .wheel_mask = SYSID_WHEEL_MASK,
    .feedback_timeout_ms = 20u, // 复用腿长台架的反馈超时。
    .joint =
        {
            HIP_MOTOR_CONFIG(&hcan1, 0x01u, 0x11u),
            HIP_MOTOR_CONFIG(&hcan1, 0x02u, 0x12u),
            HIP_MOTOR_CONFIG(&hcan2, 0x03u, 0x13u),
            HIP_MOTOR_CONFIG(&hcan2, 0x04u, 0x14u),
        },
    .wheel = {WHEEL_MOTOR_CONFIG(&hcan1, 1u), WHEEL_MOTOR_CONFIG(&hcan2, 2u)},
    .joint_feedback_id = {1u, 2u, 3u, 4u},
    .joint_zero_rad = {-1.731753157f, 1.106499972f, NAN, NAN},
    .joint_direction = {-1.0f, -1.0f, 0.0f, 0.0f}, // TODO(待确认): 右腿方向。
    .motor_per_joint = {1.0f, 1.0f, 1.0f, 1.0f},
    .joint_kp_nm_per_rad = {10.0f, 10.0f, 10.0f, 10.0f}, // 扫频指南 §4.4 的低增益起点。
    .joint_kd_nm_per_rad_s = {0.5f, 0.5f, 0.5f, 0.5f},
    .joint_max_torque_nm = {0}, // TODO(待确认): 台架用力矩上限。
    .wheel_direction = {0},     // TODO(待确认): 轮方向、速度比例增益及电流上限。
    .motor_per_wheel = {15.76f, 15.76f},
    .wheel_kp_a_per_rad_s = {0},
    .wheel_max_current_a = {0},
    .geometry = {.first_m = 0.210f,
                 .second_m = 0.24625f,
                 .virtual_second_m = 0.13351f,
                 .singular_epsilon_m = 1.0e-6f}, // OK、KW、CD。
};
