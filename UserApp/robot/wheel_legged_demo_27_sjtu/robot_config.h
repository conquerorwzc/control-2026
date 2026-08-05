/**
******************************************************************************
* @file    robot_config.h
* @author  Enhao Zhang
* @date    2025/8/8
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief Infantry wheeled-legged robot control module
******************************************************************************
* @attention
* None
*
******************************************************************************
*/
#pragma once

#include "general_def.h"
#include "robot.h"

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据

/* 27demo SJTU 适配工程：下方是唯一有效的本车底盘配置。 */
/* TODO：完成质量、惯量、质心和本车专属 K 后，同时置 model_config 两个标志为 1。 */
#define CENTER_GIMBAL_OFFSET_X 0.0f
#define CENTER_GIMBAL_OFFSET_Y 0.0f
#define WHEEL_RADIUS 0.060f
#define WHEEL_REDUCTION_RATIO 1.0f
#define TRACK_WIDTH 0.0f
#define ROBOT_MASS 0.0f
#define LEG_MAX_LENGTH 0.230f
#define LEG_MIN_LENGTH 0.030f
#define DELTA_LEG_LENGTH (LEG_MAX_LENGTH - LEG_MIN_LENGTH)
#define TARGET_JUMP_HEIGHT 0.0f
#define TARGET_JUMP_DISTANCE 0.0f
#define JUMP_SPEED 0.0f
#define JUMP_FORCE 0.0f

#define GIMBAL_COM_ANGLE_DEG 0.0f
#define ROLL_FF_BIAS 0.0f
#define ROLL_FF_AMP 0.0f
#define YAW_CHASSIS_ALIGN_ECD 0
#define PITCH_HORIZON_ECD 0
#define PITCH_MAX_ANGLE 25.0f
#define PITCH_MIN_ANGLE -30.0f
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)
#define GYRO2GIMBAL_DIR_YAW 1
#define GYRO2GIMBAL_DIR_PITCH 1
#define GYRO2GIMBAL_DIR_ROLL 1
#define ONE_BULLET_DELTA_ANGLE 36.0f
#define REDUCTION_RATIO_LOADER 90.0f
#define NUM_PER_CIRCLE 10
#define DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT 30
#define KEY_PRESS KEY_PRESS_NORMAL
#define CTRL_SPEED_COFF 1.0f

/* 本车达妙电机的输出/反馈成对方向。 */
typedef enum
{
    WHEEL_LEGGED_MOTOR_NORMAL = 0,
    WHEEL_LEGGED_MOTOR_REVERSE,
} WheelLeggedMotorDirection_e;

#define WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction)                                                                   \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL)
#define WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction)                                                                \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL)

/* J4310 关节电机配置模板。 */
#define WHEEL_LEGGED_J4310_CONFIG(can_handle_ptr, tx_identifier, rx_identifier, direction)                             \
    {                                                                                                                  \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                            \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .motor_reverse_flag = WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction),                                    \
                .feedback_reverse_flag = WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction),                              \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .angle_PID = {.Kp = 8.0f,                                                                              \
                              .Ki = 0.0f,                                                                              \
                              .Kd = 0.08f,                                                                             \
                              .MaxOut = 12.5f,                                                                         \
                              .DeadBand = 0.01f,                                                                       \
                              .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                              .IntegralLimit = 6.0f},                                                                  \
                .speed_PID = {.Kp = 0.65f,                                                                             \
                              .Ki = 0.1f,                                                                              \
                              .Kd = 0.007f,                                                                            \
                              .MaxOut = 7.0f,                                                                          \
                              .DeadBand = 0.05f,                                                                       \
                              .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                              .IntegralLimit = 6.0f},                                                                  \
            },                                                                                                         \
        .motor_type = J4310,                                                                                           \
        .can_init_config = {.can_handle = (can_handle_ptr), .tx_id = (tx_identifier), .rx_id = (rx_identifier)},       \
    }

/* H6215 轮毂电机配置模板。 */
#define WHEEL_LEGGED_H6215_CONFIG(can_handle_ptr, tx_identifier, rx_identifier, direction)                             \
    {                                                                                                                  \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .outer_loop_type = SPEED_LOOP,                                                                         \
                .close_loop_type = 0u,                                                                                 \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .motor_reverse_flag = WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction),                                    \
                .feedback_reverse_flag = WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction),                              \
            },                                                                                                         \
        .motor_type = H6215,                                                                                           \
        .can_init_config = {.can_handle = (can_handle_ptr), .tx_id = (tx_identifier), .rx_id = (rx_identifier)},       \
    }

/* ACE 显式 k 缩放同心五连杆配置。 */
static const ParallelLegGeometry_t g_leg_geometry_config = {
    .configured = 1u,
    .l0 = 0.0f,
    .real_first_link_ah = 0.105f,
    .real_second_link_hj = 0.125f,
    .virtual_second_link_ce = 0.0625f,
    .virtual_end_branch_sign = -1,
    .singular_epsilon = 1e-5f,
};

static WheelLeggedLegInitConfig_t g_left_leg_init_config = {
    .front_joint = {.motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan2, 0x0Bu, 0x0Au, WHEEL_LEGGED_MOTOR_NORMAL),
                    .chain_config = {.configured = 1u,
                                     .driving_sprocket_teeth = 12u,
                                     .driven_sprocket_teeth = 12u,
                                     .direction = 1.0f,
                                     .motor_zero_angle = -2.53547764f},
                    .kinematics_input = LEG_KINEMATICS_INPUT_PHI1},
    .rear_joint = {.motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan2, 0x09u, 0x08u, WHEEL_LEGGED_MOTOR_NORMAL),
                   .chain_config = {.configured = 1u,
                                    .driving_sprocket_teeth = 12u,
                                    .driven_sprocket_teeth = 12u,
                                    .direction = 1.0f,
                                    .motor_zero_angle = -4.74803543f},
                   .kinematics_input = LEG_KINEMATICS_INPUT_PHI2},
    .geometry_config = &g_leg_geometry_config,
    .length_control = {.force_pid_config = {.Kp = 500.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .MaxOut = 20.0f,
                                            .DeadBand = 0.0f,
                                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                            .IntegralLimit = 0.0f},
                       .velocity_damping_kd = 5.0f,
                       .length_reference = 0.170f,
                       .minimum_length = LEG_MIN_LENGTH,
                       .maximum_length = LEG_MAX_LENGTH},
};

static WheelLeggedLegInitConfig_t g_right_leg_init_config = {
    .front_joint = {.motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan1, 0x07u, 0x06u, WHEEL_LEGGED_MOTOR_REVERSE),
                    .chain_config = {.configured = 1u,
                                     .driving_sprocket_teeth = 12u,
                                     .driven_sprocket_teeth = 12u,
                                     .direction = 1.0f,
                                     .motor_zero_angle = 2.87193871f},
                    .kinematics_input = LEG_KINEMATICS_INPUT_PHI1},
    .rear_joint = {.motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan1, 0x05u, 0x04u, WHEEL_LEGGED_MOTOR_REVERSE),
                   .chain_config = {.configured = 1u,
                                    .driving_sprocket_teeth = 12u,
                                    .driven_sprocket_teeth = 12u,
                                    .direction = 1.0f,
                                    .motor_zero_angle = 0.337414742f},
                   .kinematics_input = LEG_KINEMATICS_INPUT_PHI2},
    .geometry_config = &g_leg_geometry_config,
    .length_control = {.force_pid_config = {.Kp = 500.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .MaxOut = 20.0f,
                                            .DeadBand = 0.0f,
                                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                            .IntegralLimit = 0.0f},
                       .velocity_damping_kd = 5.0f,
                       .length_reference = 0.170f,
                       .minimum_length = LEG_MIN_LENGTH,
                       .maximum_length = LEG_MAX_LENGTH},
};

static WheelLeggedWheelInitConfig_t g_left_wheel_init_config = {
    .motor_config = WHEEL_LEGGED_H6215_CONFIG(&hcan2, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_NORMAL),
    .wheel_radius = WHEEL_RADIUS,
    .reduction_ratio = WHEEL_REDUCTION_RATIO,
    .direction = 1.0f,
    .configured = 1u,
};

static WheelLeggedWheelInitConfig_t g_right_wheel_init_config = {
    .motor_config = WHEEL_LEGGED_H6215_CONFIG(&hcan1, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_REVERSE),
    .wheel_radius = WHEEL_RADIUS,
    .reduction_ratio = WHEEL_REDUCTION_RATIO,
    .direction = 1.0f,
    .configured = 1u,
};

static IMU_Init_Config_s g_chassis_imu_init_config = {
    .flag = 1u,
    .offset_flag = 1u,
    .scale = {1.0f, 1.0f, 1.0f},
    .Yaw = 0.0f,
    .Pitch = 0.0f,
    .Roll = 0.0f,
    .GyroOffset = {0.000319366809f, -0.00321031036f, -0.00178090471f},
};

static const WheelLeggedChassisStateConfig_t g_chassis_state_config = {
    .yaw_direction = 1.0f,
    .body_pitch_direction = -1.0f,
    .left_leg_relative_direction = 1.0f,
    .right_leg_relative_direction = 1.0f,
    .leg_world_body_pitch_gain = 1.0f,
    .left_leg_world_offset = -0.451405585f,
    .right_leg_world_offset = -0.542720139f,
};

static const WheelLeggedChassisLqrOutputConfig_t g_chassis_lqr_output_config = {
    .supported_body_mass = 1.683f,
    .pitch_torque_limit = 10.0f,
    .wheel_torque_limit = 0.5f,
    .wheel_torque_rate_limit = 2.0f,
    .minimum_support_projection = 0.10f,
    .prepare_length_tolerance = 0.015f,
    .prepare_length_rate_limit = 0.15f,
    .prepare_leg_angle_limit = 0.65f,
    .prepare_body_angle_limit = 0.25f,
    .prepare_stable_cycles = 100u,
};

static Chassis_Init_Config_s chassis_init_config = {
    .wheel_legged_init_config = {.left_leg_init_config = &g_left_leg_init_config,
                                 .right_leg_init_config = &g_right_leg_init_config,
                                 .left_wheel_init_config = &g_left_wheel_init_config,
                                 .right_wheel_init_config = &g_right_wheel_init_config,
                                 .imu_init_config = &g_chassis_imu_init_config,
                                 .state_config = &g_chassis_state_config,
                                 .lqr_output_config = &g_chassis_lqr_output_config},
    .model_config = {.configured = 0u,
                     .body_mass = 0.0f,
                     .leg_mass = 0.0f,
                     .wheel_mass = 0.0f,
                     .body_pitch_inertia = 0.0f,
                     .leg_inertia = 0.0f,
                     .wheel_inertia = 0.0f,
                     .yaw_inertia = 0.0f,
                     .leg_com_position = 0.0f,
                     .track_width = 0.0f,
                     .lqr_coefficients_configured = 0u},
    .initial_leg_length = 0.160f,
    .leg_min_length = LEG_MIN_LENGTH,
    .leg_max_length = LEG_MAX_LENGTH,
    .super_cap_config = {.can_config = {.can_handle = &hcan1, .tx_id = 0x210u, .rx_id = 0x211u}},
};

static Gimbal_Init_Config_s gimbal_init_config = {
    .yaw_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 0.4f,
                            .Ki = 0.0f,
                            .Kd = 0.02f,
                            .DeadBand = 0.01f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 22.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5000.0f,
                            .Ki = -100.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 25000.0f,
                        },

                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 6,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD,
        },
    .pitch_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 1.5f,
                            .Ki = 0.0f,
                            .Kd = 0.02f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5.0f,
                            .MaxOut = 25.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = -5000.0f,
                            .Ki = -200.0f,
                            .Kd = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 12000.0f,
                            .MaxOut = 28000.0f,
                        },
                },
            .motor_type = GM6020,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 2,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    .imu_init_config =
        {
            .flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = -90.0f,
            .Pitch = 0.0f,
            .Roll = 0.0f,
            .GyroOffset[0] = -0.0014910656f,
            .GyroOffset[1] = -0.00283604953f,
            .GyroOffset[2] = 0.00104337547f,
            .offset_flag = 1,
        },
    .pitch_feedforward_scale = 7000.0f};

#define FRICTION_MOTOR_CONFIG(handle, id, motor_direction, feedback_direction)                                         \
    ((Motor_Init_Config_s){                                                                                            \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 1.5f,                                                                                    \
                        .Ki = 0.2f,                                                                                    \
                        .Kd = 0.0f,                                                                                    \
                        .Improve = PID_Integral_Limit,                                                                 \
                        .IntegralLimit = 10000.0f,                                                                     \
                        .MaxOut = 15000.0f,                                                                            \
                    },                                                                                                 \
            },                                                                                                         \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .outer_loop_type = SPEED_LOOP,                                                                         \
                .close_loop_type = SPEED_LOOP,                                                                         \
                .motor_reverse_flag = motor_direction,                                                                 \
                .feedback_reverse_flag = feedback_direction,                                                           \
            },                                                                                                         \
        .motor_type = M3508,                                                                                           \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = handle,                                                                                  \
                .tx_id = id,                                                                                           \
            },                                                                                                         \
    })

static Shoot_Init_Config_s shoot_init_config = {
    .shoot_param =
        {
            .one_bullet_delta_angle = ONE_BULLET_DELTA_ANGLE, // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
            .reduction_ratio_loader = REDUCTION_RATIO_LOADER, // M2006拨盘电机的减速比
            .num_per_circle = NUM_PER_CIRCLE,                 // 拨盘一圈的装载量
            .loader_direction = 1,                            // 拨盘旋转方向,1为正向，-1为反向
            .friction_num = 2,                                // 摩擦轮数量
            .friction_speed = 37000.0f,                       // 摩擦轮速度
            .friction_coefficients = {1.0f, -1.0f},           // 摩擦轮速度比例系数
            .deadtime_burstfire = 50,
            .deadtime_onebullet = 350,
            .target_speed = 22.5f,
            .bullet_speed_adjustment = 200.0f,
            .feedforward = 200.0f,
            .one_barrel_heat_value = 10,        // 一发弹丸所需热量
            .shooter_barrel_cooling_value = 40, // 每秒冷却回复
            .shooter_barrel_heat_limit = 230,   // 热量上限
        },
    .friction_motor_config[0] = FRICTION_MOTOR_CONFIG(&hcan1, 4, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),
    .friction_motor_config[1] = FRICTION_MOTOR_CONFIG(&hcan1, 5, MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_NORMAL),

    .loader_motor_config =
        {
            .controller_param_init_config =
                {
                    .angle_PID =
                        {
                            .Kp = 60.0f,
                            .Ki = 0.0f,
                            .Kd = 0.5f,
                            .MaxOut = 40000.0f,
                        },
                    .speed_PID =
                        {
                            .Kp = 2.0f,
                            .Ki = 0.4f,
                            .Kd = 0.0f,
                            .Improve = PID_Integral_Limit | PID_ErrorHandle,
                            .IntegralLimit = 5000.0f,
                            .MaxOut = 10000.0f,
                        },
                },
            .motor_type = M2006, // 拨盘电机为M2006
            .can_init_config =
                {
                    .can_handle = &hcan2,
                    .tx_id = 3,
                },
            .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
            .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
            .controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP,
        },
};

// 云台 yaw 角度环参数:手瞄(默认)/自瞄两套,运行时按 gimbal_mode 切换
static PID_Init_Config_s yaw_angle_PID_manual_config = {
    .Kp = 0.4f,
    .Ki = 0.0f,
    .Kd = 0.02f,
    .DeadBand = 0.01f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 5.0f,
    .MaxOut = 22.0f,
};

static PID_Init_Config_s yaw_angle_PID_vision_config = {
    .Kp = 2.5f,
    .Ki = 0.0f,
    .Kd = 0.04f,
    .DeadBand = 0.01f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 5.0f,
    .MaxOut = 22.0f,
};

#if defined(GIMBAL_BOARD)
static CANComm_Init_Config_s gimbal_main_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x012,
            .rx_id = 0x011,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s *)0)->main),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->main),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};

static CANComm_Init_Config_s gimbal_motion_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x214,
            .rx_id = 0x213,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s *)0)->motion),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->motion),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};

static CANComm_Init_Config_s gimbal_gamestate_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan2,
            .tx_id = 0x216,
            .rx_id = 0x215,
        },
    .recv_data_len = sizeof(((Chassis_Upload_Data_s *)0)->gamestate),
    .send_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->gamestate),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};

#endif
#if defined(CHASSIS_BOARD)
static CANComm_Init_Config_s chassis_main_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x011,
            .rx_id = 0x012,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->main),
    .send_data_len = sizeof(((Chassis_Upload_Data_s *)0)->main),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};

static CANComm_Init_Config_s chassis_motion_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x213,
            .rx_id = 0x214,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->motion),
    .send_data_len = sizeof(((Chassis_Upload_Data_s *)0)->motion),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};

static CANComm_Init_Config_s chassis_gamestate_comm_conf = {
    .can_config =
        {
            .can_handle = &hcan3,
            .tx_id = 0x215,
            .rx_id = 0x216,
        },
    .recv_data_len = sizeof(((Chassis_Fetch_Data_s *)0)->gamestate),
    .send_data_len = sizeof(((Chassis_Upload_Data_s *)0)->gamestate),
    .daemon_count = DOUBLE_BOARD_COMM_LOST_DAEMON_COUNT,
};
#endif
