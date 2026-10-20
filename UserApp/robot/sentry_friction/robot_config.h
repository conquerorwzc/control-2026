#pragma once

#include "shoot.h"
#include "general_def.h"

// 单板控制 J4310 拨弹盘和三只无减速箱 M2006 摩擦轮。
#define ONE_BOARD

#if (defined(ONE_BOARD) + defined(CHASSIS_BOARD) + defined(GIMBAL_BOARD)) != 1
#error "Select exactly one board type."
#endif

// DJI 驱动把反馈 rpm 转为 °/s，速度目标也必须乘以 6。
#define FRICTION_SPEED_MIN_RPM 0.0f
#define FRICTION_SPEED_MAX_RPM 16000.0f

// 初版 PI：误差单位 °/s，输出为 C610 电流指令，1000 对应约 1A。
// 三个轮子使用相同增益；6A 输出上限和 1.5A 积分上限均为初调值。
#define FRICTION_MOTOR_CONFIG(id)                                      \
    {                                                                \
        .controller_param_init_config = {                            \
            .speed_PID = {                                           \
                .Kp = 0.12f,                                         \
                .Ki = 0.08f,                                         \
                .Kd = 0.0f,                                          \
                .MaxOut = 6000.0f,                                   \
                .IntegralLimit = 1500.0f,                            \
                .Improve = PID_Integral_Limit,                       \
            },                                                       \
        },                                                           \
        .controller_setting_init_config = {                          \
            .outer_loop_type = SPEED_LOOP,                           \
            .close_loop_type = SPEED_LOOP,                           \
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,            \
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,      \
            .angle_feedback_source = MOTOR_FEED,                     \
            .speed_feedback_source = MOTOR_FEED,                     \
            .feedforward_flag = FEEDFORWARD_NONE,                     \
        },                                                           \
        .motor_type = M2006,                                         \
        .can_init_config = {                                        \
            .can_handle = &hcan1,                                    \
            .tx_id = (id),                                          \
        },                                                           \
    }

// 保留当前遥控速度，单位为转子侧 °/s，不是 rpm。
#define FRICTION_SPEED_MID_APS 10000.0f
#define FRICTION_SPEED_UP_APS 30000.0f
#define LOADER_STEP_PERIOD_MID_MS 700U
#define LOADER_STEP_PERIOD_UP_MS 67U

static Shoot_Init_Config_s shoot_init_config = {
    .friction_motor_config = {
        FRICTION_MOTOR_CONFIG(5),
        FRICTION_MOTOR_CONFIG(6),
        FRICTION_MOTOR_CONFIG(7),
    },
    // 暂不区分轮序，同速同向；实际装配需要反转时修改对应系数的符号。
    .friction_coefficients = {1.0f, 1.0f, 1.0f},
    // 仅作为摩擦轮限幅，实际目标由遥控命令指定。
    .friction_speed_min = FRICTION_SPEED_MIN_RPM * 6.0f,
    .friction_speed_max = FRICTION_SPEED_MAX_RPM * 6.0f,
    .loader_step_angle = 30.0f * DEGREE_2_RAD,
    .loader_angle_tolerance = 0.03f,
    .loader_period_ms = LOADER_STEP_PERIOD_UP_MS,
    // 沿用 loader_test 已配置的 J4310 总线、ID、方向和串级 PID；角度单位 rad。
    .loader_motor_config = {
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 15.0f,
                .DeadBand = 0.005f,
            },
            .speed_PID = {
                .Kp = 0.6f,
                .Ki = 0.1f,
                .Kd = 0.0f,
                .MaxOut = 7.0f,
                .DeadBand = 0.01f,
                .IntegralLimit = 5.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
        },
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
        .motor_type = J4310,
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0x00,
        },
    },
};

// ID 5/6/7：控制帧 0x1FF 的字节 0..5，反馈分别为 0x205/0x206/0x207。
// 请勿与同一 CAN1 上使用这些反馈 ID 的设备重复注册。
