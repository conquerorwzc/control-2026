#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "dji_motor.h"

/* ================= 1. 硬件 ID 配置 ================= */
#define YAW_MOTOR_ID 2    // 确认是 ID 2
#define PITCH_MOTOR_ID 5  // 你的代码里 Pitch 是 5
#define FRIC_L_ID 1
#define FRIC_R_ID 2
#define LOADER_ID 3

/* ================= 2. 云台电机配置宏 ================= */

// Yaw 轴配置 (标准组件会强制使用 OTHER_FEED，但我们这里也写上)
#define YAW_CONFIG(can_h, _id) \
    { \
        .motor_type = GM6020, \
        .can_init_config = { .can_handle = can_h, .tx_id = _id }, \
        .controller_setting_init_config = { \
            .angle_feedback_source = OTHER_FEED, \
            .speed_feedback_source = OTHER_FEED, \
            .outer_loop_type = ANGLE_LOOP, \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, \
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
        }, \
        .controller_param_init_config = { \
            .angle_PID = { .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 30000.0f }, \
            .speed_PID = { .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 28000.0f, .IntegralLimit = 3000.0f }, \
        }, \
    }

// Pitch 轴配置
#define PITCH_CONFIG(can_h, _id) \
    { \
        .motor_type = GM6020, \
        .can_init_config = { .can_handle = can_h, .tx_id = _id }, \
        .controller_setting_init_config = { \
            .angle_feedback_source = OTHER_FEED, \
            .speed_feedback_source = OTHER_FEED, \
            .outer_loop_type = ANGLE_LOOP, \
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, \
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
        }, \
        .controller_param_init_config = { \
            .angle_PID = { .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 30000.0f }, \
            .speed_PID = { .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 28000.0f, .IntegralLimit = 3000.0f }, \
        }, \
    }

#endif