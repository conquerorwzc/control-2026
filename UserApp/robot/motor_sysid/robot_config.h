#pragma once

#include <math.h>

#include "length_control.h"
#include "robot.h"
#include "usart.h"

#define SYSID_RC_PORT (&huart5)

#ifndef SYSID_REAL_HARDWARE
#define SYSID_REAL_HARDWARE 1
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

static const float kWheelStepRadS[] = {10.0f, 30.0f, 60.0f};
static const float kTriangleSpeedRadS[] = {0.2f, 0.5f, 1.0f};
static const float kRemoteDialFullScale = 660.0f;

// 拨轮目标范围。机械止挡的平面腿长实测为 160.08～364.86 mm。
static LegLengthConfig length_pid_config = {
    .minimum_m = 0.160f,
    .maximum_m = 0.3648f,
    .damping = 5.0f,
    .torque_limit_nm = DM_T_MAX_J8009P, // 最终限幅使用各髋电机的 max_torque_nm。
    .pid = {.Kp = 800.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 40.0f,
            .DeadBand = 0.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            .IntegralLimit = 0.0f},
};

// 髋电机反馈以输出轴为基准。位置 PD 和轮速比例环在底盘计算，驱动收纯力矩/电流指令。
static Chassis_Init_Config_s chassis_init_config = {
    .joint_mask = SYSID_JOINT_MASK,
    .wheel_mask = SYSID_WHEEL_MASK,
    .feedback_timeout_ms = 20u,
    .hip[CHASSIS_LEFT_FRONT] =
        {
            .motor_config =
                {
                    .motor_type = J8009P,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 0x01u, .rx_id = 0x11u},
                },
            .zero_rad = -1.818558794f,
            .direction = -1.0f,
            .center_rad = -0.428652864f, // OK 长端标定角，-24.56°。
            .kp_nm_per_rad = 10.0f,
            .kd_nm_per_rad_s = 0.5f,
            .max_torque_nm = 40.0f,
        },
    .hip[CHASSIS_LEFT_REAR] =
        {
            .motor_config =
                {
                    .motor_type = J8009P,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 0x02u, .rx_id = 0x12u},
                },
            .zero_rad = 0.859433112f,
            .direction = -1.0f,
            .center_rad = 1.011069236f, // OA 长端标定角，57.93°。
            .kp_nm_per_rad = 10.0f,
            .kd_nm_per_rad_s = 0.5f,
            .max_torque_nm = 40.0f,
        },
    .hip[CHASSIS_RIGHT_FRONT] =
        {
            .motor_config =
                {
                    .motor_type = J8009P,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 0x03u, .rx_id = 0x13u},
                },
            .zero_rad = 0.945361238f,
            .direction = 1.0f, // 实测向车头转动时，编码器角度增大。
            .center_rad = -0.428652864f,
            .kp_nm_per_rad = 10.0f,
            .kd_nm_per_rad_s = 0.5f,
            .max_torque_nm = 40.0f,
        },
    .hip[CHASSIS_RIGHT_REAR] =
        {
            .motor_config =
                {
                    .motor_type = J8009P,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 0x04u, .rx_id = 0x14u},
                },
            .zero_rad = 2.210682934f,
            .direction = 1.0f, // 实测向车头转动时，编码器角度增大。
            .center_rad = 1.011069236f,
            .kp_nm_per_rad = 10.0f,
            .kd_nm_per_rad_s = 0.5f,
            .max_torque_nm = 40.0f,
        },
    .wheel[CHASSIS_LEFT] =
        {
            .motor_config =
                {
                    .motor_type = M3508,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 1u},
                },
            .direction = 0.0f, // TODO(待确认): 轮方向、增益和台架限值。
            .reduction = 15.76f,
            .kp_a_per_rad_s = 0.0f,
            .max_speed_rad_s = 0.0f,
            .max_current_a = 0.0f,
        },
    .wheel[CHASSIS_RIGHT] =
        {
            .motor_config =
                {
                    .motor_type = M3508,
                    .can_init_config = {.can_handle = &hcan2, .tx_id = 2u},
                },
            .direction = 0.0f,
            .reduction = 15.76f,
            .kp_a_per_rad_s = 0.0f,
            .max_speed_rad_s = 0.0f,
            .max_current_a = 0.0f,
        },
    .geometry =
        {
            .first_m = 0.210f,
            .second_m = 0.248f,
            .virtual_second_m = 0.135f, // OK、KW、CD。
            .singular_epsilon_m = 1.0e-6f,
        },
};
