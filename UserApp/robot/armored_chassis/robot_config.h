/**
 * @file robot_config.h
 * @brief 重装底盘各电机的型号、CAN 配置、反馈方向与 PID 参数。
 */

#pragma once

#include "dji_motor.h"
#include "dmmotor.h"



// 四个麦轮在 CAN1 上使用 M3508。速度环的目标和反馈为转子角速度，输出为 C620 电流指令。
// 左侧两个麦轮的输出与反馈都反向，使正速度目标统一对应车体前进。
/// 左前麦轮配置。
static Motor_Init_Config_s wheel_lf_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 1},
    .controller_setting_init_config =
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .controller_param_init_config.speed_PID =
        {
            .Kp = 1.5f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 16384.0f,
        },
};

/// 右前麦轮配置。
static Motor_Init_Config_s wheel_rf_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 2},
    .controller_setting_init_config =
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.speed_PID =
        {
            .Kp = 1.5f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 16384.0f,
        },
};

/// 左后麦轮配置。
static Motor_Init_Config_s wheel_lr_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 3},
    .controller_setting_init_config =
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .controller_param_init_config.speed_PID =
        {
            .Kp = 1.4f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 16384.0f,
        },
};

/// 右后麦轮配置。
static Motor_Init_Config_s wheel_rr_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 4},
    .controller_setting_init_config =
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.speed_PID =
        {
            .Kp = 1.5f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .MaxOut = 16384.0f,
        },
};

// 左右引导机构在 CAN2 上使用 M2006。位置外环输出速度目标，速度内环输出 C610 电流指令。
/// 左引导电机配置。
static Motor_Init_Config_s guide_left_config = {
    .motor_type = M2006,
    .can_init_config = {.can_handle = &hfdcan2, .tx_id = 1},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 108000.0f},
    .controller_param_init_config.speed_PID = {.MaxOut = 10000.0f},
};

/// 右引导电机配置。
static Motor_Init_Config_s guide_right_config = {
    .motor_type = M2006,
    .can_init_config = {.can_handle = &hfdcan2, .tx_id = 2},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 108000.0f},
    .controller_param_init_config.speed_PID = {.MaxOut = 10000.0f},
};

// CAN2：前抬升 4340P，后抬升 4340
// 4340 和 4340P 使用 J4340 驱动配置，tx_id 为电机 ID，rx_id 为 Master ID。
// 位置环给出角速度目标，单位为 rad/s。速度环给出力矩指令，单位为 N·m。
// 右前和左后的输出与反馈同时反向，使四个齿条的正位移都对应抬升。
/// 左前抬升电机配置，对应 30 齿小齿轮。
static Motor_Init_Config_s lift_lf_config = {
    .motor_type = J4340,
    .can_init_config =
        {
            .can_handle = &hfdcan2,
            .tx_id = 0x03,
            .rx_id = 0x13,
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {
        .Kp = 15.0f,
        .MaxOut = 5.864306f,
    },
    .controller_param_init_config.speed_PID = {
        .Kp = 5.0f,
        .MaxOut = 28.0f,
    },
};

/// 右前抬升电机配置，对应 30 齿小齿轮。
static Motor_Init_Config_s lift_rf_config = {
    .motor_type = J4340,
    .can_init_config =
        {
            .can_handle = &hfdcan2,
            .tx_id = 0x04,
            .rx_id = 0x14,

        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .controller_param_init_config.angle_PID = {
        .Kp = 15.0f,
        .MaxOut = 5.864306f,
    },
    .controller_param_init_config.speed_PID = {
        .Kp = 5.0f,
        .MaxOut = 28.0f,
    },
};

/// 左后抬升电机配置，对应 50 齿大齿轮。
static Motor_Init_Config_s lift_lr_config = {
    .motor_type = J4340,
    .can_init_config =
        {
            .can_handle = &hfdcan2,
            .tx_id = 0x05,
            .rx_id = 0x15,
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
    .controller_param_init_config.angle_PID = {
        .Kp = 17.0f,
        .MaxOut = 5.864306f,
    },
    .controller_param_init_config.speed_PID = {
        .Kp = 3.0f,
        .MaxOut = 28.0f,
    },
};

/// 右后抬升电机配置，对应 50 齿大齿轮。
static Motor_Init_Config_s lift_rr_config = {
    .motor_type = J4340,
    .can_init_config =
        {
            .can_handle = &hfdcan2,
            .tx_id = 0x06,
            .rx_id = 0x16,
        },
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {
        .Kp = 19.0f,
        .MaxOut = 5.864306f,
    },
    .controller_param_init_config.speed_PID = {
        .Kp = 3.0f,
        .MaxOut = 28.0f,
    },
};
static const float kLiftMinTargetMm = -280.0f;

// 按四台电机和挡块实测填写。0 表示尚未配置。
static const float kLiftHomeTorqueLimitNm[4] = {2.0f, 2.0f, 2.0f, 2.0f};
static const float kLiftHomeContactTorqueNm[4] = {0.50f, 0.50f, 0.80f, 0.80f};

// 侧边接头机构采用位置外环和速度内环。下方各配置按实际接线指定 CAN1 或 CAN2。
/// 左侧接头机构的 M2006 电机配置。
static Motor_Init_Config_s side_left_2006_config = {
    .motor_type = M2006,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 7},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 108000.0f},
    .controller_param_init_config.speed_PID = {.MaxOut = 10000.0f},
};

/// 右侧接头机构的 M2006 电机配置。
static Motor_Init_Config_s side_right_2006_config = {
    .motor_type = M2006,
    .can_init_config = {.can_handle = &hfdcan2, .tx_id = 3},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 108000.0f},
    .controller_param_init_config.speed_PID = {.MaxOut = 10000.0f},
};

/// 左侧接头机构的 M3508 电机配置。
static Motor_Init_Config_s side_left_3508_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 5},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .angle_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 2.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = 16384.0f,
                },
            .angle_PID =
                {
                    .MaxOut = 55536.0f,
                },
        },
};

/// 右侧接头机构的 M3508 电机配置。
static Motor_Init_Config_s side_right_3508_config = {
    .motor_type = M3508,
    .can_init_config = {.can_handle = &hfdcan1, .tx_id = 6},
    .controller_setting_init_config =
        {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .angle_feedback_source = MOTOR_FEED,
        },
    .controller_param_init_config =
        {
            .speed_PID =
                {
                    .Kp = 2.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = 16384.0f,
                },
            .angle_PID =
                {
                    .MaxOut = 55536.0f,
                },
        },
};
