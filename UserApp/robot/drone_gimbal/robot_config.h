#pragma once
#include "dji_motor.h"

/* ================= 1. 硬件 ID 配置 ================= */
#define YAW_ID      3   // GM6020 CAN1
#define PITCH_ID    5   // GM6020 CAN1
#define FRIC_L_ID   1   // M3508 CAN1 (左)
#define FRIC_R_ID   2   // M3508 CAN1 (右)

/* ================= 2. 关键角度参数 ================= */
// 【1】Pitch轴 (ECD 2250 -> 98.88度, ECD 1250 -> 54.93度)
#define PITCH_INIT_ANGLE 98.88f  // 水平
#define PITCH_MAX_ANGLE  98.88f  // 0度
#define PITCH_MIN_ANGLE  54.93f  // -45度

// 【2】Yaw轴 (ECD 2701 -> 118.69度)
#define YAW_INIT_ANGLE   118.69f // 正前
#define YAW_MAX_ANGLE    208.69f // 右转极限
#define YAW_MIN_ANGLE    29.80f  // 左转极限

/* ================= 3. 电机初始化宏 (已拆分) ================= */

// --- Pitch 轴专用配置 ---
#define PITCH_CONFIG(can_h, _id) \
    { \
        .motor_type = GM6020, \
        .can_init_config = { .can_handle = can_h, .tx_id = _id }, \
        .controller_setting_init_config = { \
            .angle_feedback_source = MOTOR_FEED, \
            .speed_feedback_source = MOTOR_FEED, \
            .outer_loop_type = ANGLE_LOOP, \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, \
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
        }, \
        .controller_param_init_config = { \
            /* Angle Kp=12: 响应快 */ \
            .angle_PID = { .Kp = 80.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 30000.0f }, /* Speed Kp=45: 标准力度，足够应付 Pitch */ \
            .speed_PID = { .Kp = 11.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 30000.0f, .IntegralLimit = 3000.0f }, \
        }, \
    }

// --- Yaw 轴专用配置 (重负载/倒挂加强版) ---
#define YAW_CONFIG(can_h, _id) \
    { \
        .motor_type = GM6020, \
        .can_init_config = { .can_handle = can_h, .tx_id = _id }, \
        .controller_setting_init_config = { \
            .angle_feedback_source = MOTOR_FEED, \
            .speed_feedback_source = MOTOR_FEED, \
            .outer_loop_type = ANGLE_LOOP, \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, \
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
        }, \
        .controller_param_init_config = { \
            /* Angle Kp=12: 保持灵敏 */ \
            .angle_PID = { .Kp = 60.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 30000.0f }, /* Speed Kp=60: 【加强】力度更大，锁住重负载 */ /* 如果还不够硬，可以继续加到 80 */ \
            .speed_PID = { .Kp = 5.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 28000.0f, .IntegralLimit = 3000.0f }, \
        }, \
    }

// --- 摩擦轮配置 ---
#define SHOOT_MOTOR_CONFIG(can_h, _id, _reverse) \
    { \
        .motor_type = M3508, \
        .can_init_config = { .can_handle = can_h, .tx_id = _id }, \
        .controller_setting_init_config = { \
            .outer_loop_type = SPEED_LOOP, \
            .close_loop_type = SPEED_LOOP, \
            .motor_reverse_flag = _reverse, \
        }, \
        .controller_param_init_config = { \
            .speed_PID = { \
                .Kp = 5.0f, .Ki = 0.00f, .Kd = 0.0f, \
                .MaxOut = 16000.0f, .IntegralLimit = 5000.0f, \
            }, \
        }, \
    }