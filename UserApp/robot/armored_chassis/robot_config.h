#pragma once

#include "dji_motor.h"
#include "dmmotor.h"

// CAN1：四只麦轮
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

// CAN2：左右引导
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
    .controller_param_init_config.angle_PID = {.MaxOut = 5.864306f},
    .controller_param_init_config.speed_PID = {.MaxOut = 28.0f},
};

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
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 5.864306f},
    .controller_param_init_config.speed_PID = {.MaxOut = 28.0f},
};

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
        },
    .controller_param_init_config.angle_PID = {.MaxOut = 5.864306f},
    .controller_param_init_config.speed_PID = {.MaxOut = 28.0f},
};

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
    .controller_param_init_config.angle_PID = {.MaxOut = 5.864306f},
    .controller_param_init_config.speed_PID = {.MaxOut = 28.0f},
};

// 接头机构电机注册
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
