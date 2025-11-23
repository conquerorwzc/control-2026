//
// Created by PC on 2025/11/18.
//

#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

/* ================= 硬件 ID 配置(须修改） ================= */
#define YAW_MOTOR_ID        2   // 6020 Yaw轴
#define PUSH_VERT_ID        7   // 2006 垂直推杆
#define PUSH_HORI_ID        6   // 2006 水平推杆

// 摩擦轮 (3508 x4)
#define FRIC_MOTOR_LU     1   // 左上Left Upward
#define FRIC_MOTOR_LD     2   // 左下Left Downward
#define FRIC_MOTOR_RU     3   // 右上Right Upward
#define FRIC_MOTOR_RD     4   // 右下Right Downward


/* ================= 飞镖系统参数 ================= */
// 校准参数
#define CALI_SPEED              3000.0f   // 校准时的归位速度 (RPM/电流单位)
#define CALI_BACK_OFFSET        8000.0f   // 堵转后回退的编码器距离 (约一圈)
#define CALI_POS_THRESHOLD      100.0f    // 回退到位判断阈值

// 摩擦轮射速预设 (RPM)
#define FRIC_SPEED_HIGH         6000.0f
#define FRIC_SPEED_LOW          3000.0f


/* ================= 电机初始化宏定义 ================= */

/**
 * @brief 2006 推杆电机配置 (带堵转检测)
 * @note  关键点：speed_PID.Improve 必须包含 PID_ErrorHandle
 */
#define PUSH_ROD_CONFIG(can_h, _id, _reverse) \
{ \
    .motor_type = M2006, \
    .can_init_config = { \
        .can_handle = can_h, \
        .tx_id = _id, \
    }, \
    .controller_setting_init_config = { \
        .angle_feedback_source = MOTOR_FEED, \
        .speed_feedback_source = MOTOR_FEED, \
        .outer_loop_type = SPEED_LOOP, \
        .close_loop_type = SPEED_LOOP, \
        .motor_reverse_flag = _reverse, \
    }, \
    .controller_param_init_config = { \
        .speed_PID = { \
            .Kp = 0.0f, \
            .Ki = 0.0f, \
            .Kd = 0.0f, \
            .MaxOut = 10000.0f, \
            .IntegralLimit = 3000.0f, \
            /* 核心：开启堵转检测和积分限幅 */ \
            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_ErrorHandle, \
        }, \
    }, \
}

/**
 * @brief 6020 Yaw轴电机配置 (双环 PID)
 */
#define YAW_MOTOR_CONFIG(can_h, _id) \
{ \
    .motor_type = GM6020, \
    .can_init_config = { \
        .can_handle = can_h, \
        .tx_id = _id, \
    }, \
    .controller_setting_init_config = { \
        .angle_feedback_source = MOTOR_FEED, \
        .speed_feedback_source = MOTOR_FEED, \
        .outer_loop_type = ANGLE_LOOP, \
        .close_loop_type = SPEED_LOOP | ANGLE_LOOP, \
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, \
    }, \
    .controller_param_init_config = { \
        .speed_PID = { \
            .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, \
            .MaxOut = 30000.0f, .IntegralLimit = 10000.0f, \
            .Improve = PID_Integral_Limit, \
        }, \
        .angle_PID = { \
            .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, \
            .MaxOut = 500.0f, \
            .Improve = PID_Integral_Limit, \
        }, \
    }, \
}

/**
 * @brief 3508 摩擦轮电机配置 (单环速度 PID)
 */
#define FRIC_MOTOR_CONFIG(can_h, _id, _reverse) \
{ \
    .motor_type = M3508, \
    .can_init_config = { \
        .can_handle = can_h, \
        .tx_id = _id, \
    }, \
    .controller_setting_init_config = { \
        .outer_loop_type = SPEED_LOOP, \
        .close_loop_type = SPEED_LOOP, \
        .motor_reverse_flag = _reverse, \
    }, \
    .controller_param_init_config = { \
        .speed_PID = { \
            .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, \
            .MaxOut = 16000.0f, .IntegralLimit = 5000.0f, \
            .Improve = PID_Integral_Limit, \
        }, \
    }, \
}

#endif  // CONTROL_2026_ROBOT_CONFIG_H
