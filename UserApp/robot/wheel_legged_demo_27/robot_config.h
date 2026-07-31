/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   双闭环轮腿机器人硬件、机构和坐标标定配置
 ******************************************************************************
 */
#pragma once

/* Private includes ----------------------------------------------------------*/
#include "bsp_can.h"
#include "robot.h"

/* Private define ------------------------------------------------------------*/
/* 本车达妙电机的输出和反馈配对方向；同一台电机只能二选一。 */
typedef enum
{
    WHEEL_LEGGED_MOTOR_NORMAL = 0, /* 电机输出与反馈均为正向。 */
    WHEEL_LEGGED_MOTOR_REVERSE,    /* 电机输出与反馈均为反向。 */
} WheelLeggedMotorDirection_e;

/* 将本车的配对方向换算为底层达妙电机输出方向枚举。 */
#define WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction) \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL)

/* 将本车的配对方向换算为底层达妙电机反馈方向枚举。 */
#define WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction) \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL)

/* 根据关节 CAN 地址和配对方向构造一台 J4310 关节电机配置。 */
#define DOUBLE_CLOSED_LOOP_LEG_J4310_CONFIG(can_handle_ptr, tx_identifier, rx_identifier, direction)                  \
    {                                                                                                                    \
        .controller_setting_init_config =                                                                               \
            {                                                                                                           \
                .outer_loop_type = ANGLE_LOOP,                                                                          \
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                             \
                .angle_feedback_source = MOTOR_FEED,                                                                    \
                .speed_feedback_source = MOTOR_FEED,                                                                    \
                .motor_reverse_flag = WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction),                                    \
                .feedback_reverse_flag = WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction),                              \
            },                                                                                                          \
        .controller_param_init_config =                                                                                 \
            {                                                                                                           \
                .angle_PID =                                                                                            \
                    {                                                                                                   \
                        .Kp = 8.0f,                                                                                     \
                        .Ki = 0.0f,                                                                                     \
                        .Kd = 0.08f,                                                                                    \
                        .MaxOut = 12.5f,                                                                                \
                        .DeadBand = 0.01f,                                                                              \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,        \
                        .IntegralLimit = 6.0f,                                                                          \
                    },                                                                                                  \
                .speed_PID =                                                                                            \
                    {                                                                                                   \
                        .Kp = 0.65f,                                                                                    \
                        .Ki = 0.1f,                                                                                     \
                        .Kd = 0.007f,                                                                                   \
                        .MaxOut = 7.0f,                                                                                 \
                        .DeadBand = 0.05f,                                                                              \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,        \
                        .IntegralLimit = 6.0f,                                                                          \
                    },                                                                                                  \
            },                                                                                                          \
        .motor_type = J4310,                                                                                            \
        .can_init_config =                                                                                              \
            {                                                                                                           \
                .can_handle = (can_handle_ptr),                                                                         \
                .tx_id = (tx_identifier),                                                                               \
                .rx_id = (rx_identifier),                                                                               \
            },                                                                                                          \
    }

/* 根据轮毂 CAN 地址和配对方向构造一台 H6215 只读反馈配置。 */
#define DOUBLE_CLOSED_LOOP_LEG_H6215_CONFIG(can_handle_ptr, tx_identifier, rx_identifier, direction)                  \
    {                                                                                                                    \
        .controller_setting_init_config =                                                                               \
            {                                                                                                           \
                .outer_loop_type = SPEED_LOOP,                                                                          \
                .close_loop_type = 0u,                                                                                  \
                .angle_feedback_source = MOTOR_FEED,                                                                    \
                .speed_feedback_source = MOTOR_FEED,                                                                    \
                .motor_reverse_flag = WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction),                                    \
                .feedback_reverse_flag = WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction),                              \
            },                                                                                                          \
        .motor_type = H6215,                                                                                            \
        .can_init_config =                                                                                              \
            {                                                                                                           \
                .can_handle = (can_handle_ptr),                                                                         \
                .tx_id = (tx_identifier),                                                                               \
                .rx_id = (rx_identifier),                                                                               \
            },                                                                                                          \
    }

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* 左右腿共用的双闭环 CAD 几何和装配支路参数；所有长度单位均为 m。 */
/* TODO：以 CAD 或三坐标测量值复核全部杆长、装配支路和闭环一致性阈值后再冻结参数。 */
static const DoubleClosedLoopLegGeometry_t g_leg_geometry_config = {
    .configured = 1u,
    .oa_length = 0.0525f,         /* 暂测 OA=52.5 mm。 */
    .oc_length = 0.0525f,         /* 暂测 OC=52.5 mm。 */
    .ab_length = 0.0625f,         /* 暂测 AB=62.5 mm。 */
    .bc_length = 0.0625f,         /* 暂测 BC=62.5 mm。 */
    .bd_length = 0.10054f,        /* 暂测 BD=100.54 mm。 */
    .cd_length = 0.0400f,         /* 暂测 CD=40.0 mm。 */
    .of_length = 0.1050f,         /* 暂测 OF=105.0 mm。 */
    .cf_length = 0.0525f,         /* 暂测 CF=52.5 mm；O-C-F 共线且 C 位于中间。 */
    .de_length = 0.0525f,         /* 暂测 DE=52.5 mm。 */
    .ef_length = 0.0400f,         /* 暂测 EF=40.0 mm。 */
    .ep_length = 0.16294f,        /* 暂测 EP=162.94 mm。 */
    .fp_length = 0.1250f,         /* 暂测 FP=125.0 mm。 */
    .first_loop_branch_sign = -1, /* B 位于有向线 C->A 的视觉左侧，对应叉积为负。 */
    .bcd_branch_sign = 1,         /* D 位于有向线 C->B 的视觉左侧，对应叉积为正。 */
    .second_loop_branch_sign = 1, /* E 位于有向线 F->D 的视觉右侧，对应叉积为正。 */
    .efp_branch_sign = -1,        /* P 位于有向线 F->E 的视觉左侧，对应叉积为负。 */
    .geometry_consistency_epsilon = 0.0010f,
    .singular_epsilon = 1e-5f,
};

/* 左腿：前关节对应 phi2，即 x 轴到 O-C 的角；后关节对应 phi1，即 x 轴到 O-A 的角。 */
static WheelLeggedLegInitConfig_t g_left_leg_init_config = {
    .front_joint =
        {
            .motor_config = DOUBLE_CLOSED_LOOP_LEG_J4310_CONFIG(
                &hcan1, 0x05u, 0x04u, WHEEL_LEGGED_MOTOR_REVERSE),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* O-C 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = 0.0974674225f, /* phi2=0 时的左前电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
        },
    .rear_joint =
        {
            .motor_config = DOUBLE_CLOSED_LOOP_LEG_J4310_CONFIG(
                &hcan1, 0x07u, 0x06u, WHEEL_LEGGED_MOTOR_REVERSE),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* O-A 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = -3.34725761f, /* phi1=0 时的左后电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
        },
    .geometry_config = &g_leg_geometry_config,
};

/* 右腿：前关节对应 phi2，即 x 轴到 O-C 的角；后关节对应 phi1，即 x 轴到 O-A 的角。 */
static WheelLeggedLegInitConfig_t g_right_leg_init_config = {
    .front_joint =
        {
            .motor_config = DOUBLE_CLOSED_LOOP_LEG_J4310_CONFIG(
                &hcan2, 0x09u, 0x08u, WHEEL_LEGGED_MOTOR_NORMAL),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* O-C 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = 0.0703821182f, /* phi2=0 时的右前电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
        },
    .rear_joint =
        {
            .motor_config = DOUBLE_CLOSED_LOOP_LEG_J4310_CONFIG(
                &hcan2, 0x0Bu, 0x0Au, WHEEL_LEGGED_MOTOR_NORMAL),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* O-A 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = -3.39074516f, /* phi1=0 时的右后电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
        },
    .geometry_config = &g_leg_geometry_config,
};

/* 左轮：H6215，轮径 120 mm，减速比 1:1；direction 必须通过手推前进试验复核。 */
/* TODO：手推确认左轮前进时轮端角速度和 s_dot 均为正后，冻结 left_wheel.direction。 */
static WheelLeggedWheelInitConfig_t g_left_wheel_init_config = {
    .motor_config = DOUBLE_CLOSED_LOOP_LEG_H6215_CONFIG(
        &hcan1, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_NORMAL),
    .wheel_radius = 0.0600f, /* 轮半径 60 mm，单位 m。 */
    .reduction_ratio = 1.0f,
    .direction = 1.0f,       /* 暂定前进为正，手推后若 s_dot<0 则改为 -1。 */
    .configured = 1u,
};

/* 右轮：H6215，轮径 120 mm，减速比 1:1；direction 必须通过手推前进试验复核。 */
/* TODO：手推确认右轮前进时轮端角速度和 s_dot 均为正后，冻结 right_wheel.direction。 */
static WheelLeggedWheelInitConfig_t g_right_wheel_init_config = {
    .motor_config = DOUBLE_CLOSED_LOOP_LEG_H6215_CONFIG(
        &hcan2, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_NORMAL),
    .wheel_radius = 0.0600f, /* 轮半径 60 mm，单位 m。 */
    .reduction_ratio = 1.0f,
    .direction = 1.0f,       /* 暂定前进为正，手推后若 s_dot<0 则改为 -1。 */
    .configured = 1u,
};

/* 底盘 IMU 安装配置；INS 原始角度单位为 deg，由状态模块统一换算为 rad。 */
/* TODO：完成静止和单轴转动试验后，写入 IMU 安装角、标度、零偏及离线标定参数。 */
static IMU_Init_Config_s g_chassis_imu_init_config = {
    .flag = 1u,
    .offset_flag = 0u,
    .scale = {1.0f, 1.0f, 1.0f},
    .Yaw = 0.0f,
    .Pitch = 0.0f,
    .Roll = 0.0f,
    .GyroOffset = {0.0f, 0.0f, 0.0f},
};

/* 十维状态的坐标符号；世界系腿角关系需结合静态俯仰试验确认。 */
/* TODO：通过前进手推、单轴俯仰和左右腿摆动试验确认所有方向、机身俯仰增益与零位偏置。 */
static const WheelLeggedChassisStateConfig_t g_chassis_state_config = {
    .yaw_direction = 1.0f,
    .body_pitch_direction = -1.0f,
    .left_leg_relative_direction = 1.0f,
    .right_leg_relative_direction = 1.0f,
    .leg_world_body_pitch_gain = 1.0f,
    .left_leg_world_offset = 0.0f,
    .right_leg_world_offset = 0.0f,
};

/* 本车底盘根配置；robot.c 只将该对象交给 chassis component。 */
static WheelLeggedChassisInitConfig_t g_chassis_init_config = {
    .left_leg_init_config = &g_left_leg_init_config,
    .right_leg_init_config = &g_right_leg_init_config,
    .left_wheel_init_config = &g_left_wheel_init_config,
    .right_wheel_init_config = &g_right_wheel_init_config,
    .imu_init_config = &g_chassis_imu_init_config,
    .state_config = &g_chassis_state_config,
};
