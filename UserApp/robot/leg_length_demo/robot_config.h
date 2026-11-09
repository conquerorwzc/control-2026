#pragma once
#include "dmmotor.h"
#include "length_control.h"
#include "usart.h"

// 源 wheel_legged_SHU 的标定门控为 0；确认此 demo 的几何/零点后才改成 1。
// 不得把本 demo 的 105/125 mm 等参数当成正在设计的新车参数。
#ifndef LEG_LENGTH_CALIBRATED
#define LEG_LENGTH_CALIBRATED 0
#endif
#define LEG_LENGTH_FEEDBACK_TIMEOUT_MS 20u

static const float kRemoteFullScale = 660.0f;
static const float kHandoverSpeedMps = 0.05f;
static const float kJointZeroRad[4] = {0.815470527f, -2.469308355f, -3.051547223f, -2.642480681f};
static LegLengthConfig length_config = {
    .first_m = 0.105f,
    .second_m = 0.125f,
    .virtual_second_m = 0.0625f,
    .epsilon = 1e-5f,
    .minimum_m = 0.05619f,
    .maximum_m = 0.18192f,
    .damping = 5.0f,
    .torque_limit_nm = 2.0f,
    .pid = {.Kp = 800.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 40.0f,
            .DeadBand = 0.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            .IntegralLimit = 0.0f},
};

// 保留 SHU 的四关节 ID、左右方向。内部传动 direction=1, reduction_ratio=1。
#define LENGTH_MOTOR(bus, tx, rx, reverse, feedback_reverse)                                                           \
    {                                                                                                                  \
        .motor_type = J8009P, .can_init_config = {.can_handle = (bus), .tx_id = (tx), .rx_id = (rx)},                  \
        .controller_setting_init_config = {                                                                            \
            .motor_reverse_flag = (reverse),                                                                           \
            .feedback_reverse_flag = (feedback_reverse)                                                                \
        }                                                                                                              \
    }
static Motor_Init_Config_s joint_config[4] = {
    LENGTH_MOTOR(&hcan1, 0x0Bu, 0x0Au, MOTOR_DIRECTION_NORMAL, FEEDBACK_DIRECTION_NORMAL),
    LENGTH_MOTOR(&hcan1, 0x09u, 0x08u, MOTOR_DIRECTION_NORMAL, FEEDBACK_DIRECTION_NORMAL),
    LENGTH_MOTOR(&hcan2, 0x07u, 0x06u, MOTOR_DIRECTION_REVERSE, FEEDBACK_DIRECTION_REVERSE),
    LENGTH_MOTOR(&hcan2, 0x05u, 0x04u, MOTOR_DIRECTION_REVERSE, FEEDBACK_DIRECTION_REVERSE),
};
#undef LENGTH_MOTOR
