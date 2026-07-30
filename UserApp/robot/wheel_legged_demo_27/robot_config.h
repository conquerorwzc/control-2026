/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   双闭环腿正运动学真机观测工程配置
 *
 * 当前初始化左右腿四个关节，均只做零力矩保活和正运动学观测。
 * 整车 CAN 分配记录如下：
 *   左轮毂：CAN1，tx_id=0x01，rx_id=0x00；右轮毂：CAN2，tx_id=0x01，rx_id=0x00。
 *   左前关节：CAN1，tx_id=0x05，rx_id=0x04；左后关节：CAN1，tx_id=0x07，rx_id=0x06。
 *   右前关节：CAN2，tx_id=0x09，rx_id=0x08；右后关节：CAN2，tx_id=0x0B，rx_id=0x0A。
 * 轮毂的 CAN 地址已记录，但电机型号和控制方式未确认，不能在这里提前初始化驱动。
 ******************************************************************************
 */
#pragma once

#include "bsp_can.h"
#include "robot.h"

/* 左前关节达妙电机的 CAN 发送标识符，对应 OC 主动轴 phi2。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_FRONT_JOINT_TX_ID 0x05u
/* 左前关节达妙电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_FRONT_JOINT_RX_ID 0x04u
/* 左后关节达妙电机的 CAN 发送标识符，对应 OA 主动轴 phi1。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_REAR_JOINT_TX_ID 0x07u
/* 左后关节达妙电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_REAR_JOINT_RX_ID 0x06u
/* 右前关节达妙电机的 CAN 发送标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_FRONT_JOINT_TX_ID 0x09u
/* 右前关节达妙电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_FRONT_JOINT_RX_ID 0x08u
/* 右后关节达妙电机的 CAN 发送标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_REAR_JOINT_TX_ID 0x0Bu
/* 右后关节达妙电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_REAR_JOINT_RX_ID 0x0Au
/* 左轮毂电机的 CAN 发送标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_WHEEL_TX_ID 0x01u
/* 左轮毂电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_LEFT_WHEEL_RX_ID 0x00u
/* 右轮毂电机的 CAN 发送标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_WHEEL_TX_ID 0x01u
/* 右轮毂电机的 CAN 接收标识符。 */
#define DOUBLE_CLOSED_LOOP_LEG_RIGHT_WHEEL_RX_ID 0x00u

/* CAD 杆长和装配支路尚未标定，禁止计算真实机构位姿。 */
#define DOUBLE_CLOSED_LOOP_LEG_GEOMETRY_CONFIGURED 0u
/*
 * 根据 CAN 外设、CAN 标识符和电机坐标方向构造一台零力矩保活用的达妙电机初始化配置。
 * 使用 MOTOR_FEED 自身反馈闭环时，电机输出方向与反馈方向应同向配置：均为 NORMAL 或均为 REVERSE。
 */
#define DOUBLE_CLOSED_LOOP_LEG_MOTOR_CONFIG(can_handle_ptr, tx_identifier, rx_identifier, motor_direction,             \
                                            feedback_direction)                                                        \
    {                                                                                                                  \
        .controller_setting_init_config =                                                                              \
            {                                                                                                          \
                .outer_loop_type = ANGLE_LOOP,                                                                         \
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP,                                                            \
                .angle_feedback_source = MOTOR_FEED,                                                                   \
                .speed_feedback_source = MOTOR_FEED,                                                                   \
                .motor_reverse_flag = (motor_direction),                                                               \
                .feedback_reverse_flag = (feedback_direction),                                                         \
            },                                                                                                         \
        .controller_param_init_config =                                                                                \
            {                                                                                                          \
                .angle_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 8.0f,                                                                                    \
                        .Ki = 0.0f,                                                                                    \
                        .Kd = 0.08f,                                                                                   \
                        .MaxOut = 12.5f,                                                                               \
                        .DeadBand = 0.01f,                                                                             \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,       \
                        .IntegralLimit = 6.0f,                                                                         \
                    },                                                                                                 \
                .speed_PID =                                                                                           \
                    {                                                                                                  \
                        .Kp = 0.65f,                                                                                   \
                        .Ki = 0.1f,                                                                                    \
                        .Kd = 0.007f,                                                                                  \
                        .MaxOut = 7.0f,                                                                                \
                        .DeadBand = 0.05f,                                                                             \
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,       \
                        .IntegralLimit = 6.0f,                                                                         \
                    },                                                                                                 \
            },                                                                                                         \
        .motor_type = J4310,                                                                                           \
        .can_init_config =                                                                                             \
            {                                                                                                          \
                .can_handle = (can_handle_ptr),                                                                        \
                .tx_id = (tx_identifier),                                                                              \
                .rx_id = (rx_identifier),                                                                              \
            },                                                                                                         \
    }

/*
 * 本头文件仅由本车的 robot.c 包含，故配置对象按本仓库既有风格定义为文件内静态对象。
 * 几何和传动参数未完成标定前必须保持 configured 为 0，不能用演示参数驱动真机 FK。
 */
/* 左前关节：对应 phi2（x 轴到 O-C 的角）；填写实际链轮齿数，传动比由程序自动计算。 */
static const WheelLeggedChainTransmissionConfig_t g_left_front_joint_chain_config = {
    .configured = 1u,
    .driving_sprocket_teeth = 12u,
    .driven_sprocket_teeth = 12u,
    .direction = 1.0f,
    .motor_zero_angle = 0.0974674225f, /* 机械 phi2=0 时，左前电机读数；对应 OC。 */
};

/* 左后关节：对应 phi1（x 轴到 O-A 的角）；填写实际链轮齿数，传动比由程序自动计算。 */
static const WheelLeggedChainTransmissionConfig_t g_left_rear_joint_chain_config = {
    .configured = 1u,
    .driving_sprocket_teeth = 12u,
    .driven_sprocket_teeth = 12u,
    .direction = 1.0f,
    .motor_zero_angle = -3.34725761f, /* 机械 phi1=0 时，左后电机读数；对应 OA。 */
};

/* 右前关节：填写电机侧主动链轮、O1 侧从动链轮的实际齿数；传动比由程序自动计算。 */
static const WheelLeggedChainTransmissionConfig_t g_right_front_joint_chain_config = {
    .configured = 1u,
    .driving_sprocket_teeth = 12u,
    .driven_sprocket_teeth = 12u,
    .direction = 1.0f,
    .motor_zero_angle = 0.0703821182f, /* 机械 phi2=0 时，右前电机读数；对应 OC。 */
};

/* 右后关节：填写电机侧主动链轮、O2 侧从动链轮的实际齿数；传动比由程序自动计算。 */
static const WheelLeggedChainTransmissionConfig_t g_right_rear_joint_chain_config = {
    .configured = 1u,
    .driving_sprocket_teeth = 12u,
    .driven_sprocket_teeth = 12u,
    .direction = 1.0f,
    .motor_zero_angle = -3.39074516f, /* 机械 phi1=0 时，右后电机读数；对应 OA。 */
};

/* 双闭环杆系的 CAD 几何和装配支路参数。所有长度均为转轴中心距，单位：m。 */
static const DoubleClosedLoopLegGeometry_t g_unconfigured_leg_geometry_config = {
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
    .first_loop_branch_sign = -1, /* B 位于有向线 C->A 的视觉左侧；对应叉积为负。 */
    .bcd_branch_sign = 1,         /* D 位于有向线 C->B 的视觉左侧；对应叉积为正。 */
    .second_loop_branch_sign = 1, /* E 位于有向线 F->D 的视觉右侧；对应叉积为正。 */
    .efp_branch_sign = -1,        /* P 位于有向线 F->E 的视觉左侧；对应叉积为负。 */
    .geometry_consistency_epsilon = 0.0010f,
    .singular_epsilon = 1e-5f,
};

/* 左腿关节初始化配置：已实测左前对应 phi2（OC），左后对应 phi1（OA）。 */
static WheelLeggedLegInitConfig_t g_left_leg_init_config = {
    .front_joint_motor_config = DOUBLE_CLOSED_LOOP_LEG_MOTOR_CONFIG(
        &hcan1, DOUBLE_CLOSED_LOOP_LEG_LEFT_FRONT_JOINT_TX_ID, DOUBLE_CLOSED_LOOP_LEG_LEFT_FRONT_JOINT_RX_ID,
        MOTOR_DIRECTION_REVERSE, FEEDBACK_DIRECTION_REVERSE),
    .rear_joint_motor_config = DOUBLE_CLOSED_LOOP_LEG_MOTOR_CONFIG(&hcan1, DOUBLE_CLOSED_LOOP_LEG_LEFT_REAR_JOINT_TX_ID,
                                                                   DOUBLE_CLOSED_LOOP_LEG_LEFT_REAR_JOINT_RX_ID,
                                                                   MOTOR_DIRECTION_REVERSE, FEEDBACK_DIRECTION_REVERSE),
    .front_joint_chain_config = &g_left_front_joint_chain_config,
    .rear_joint_chain_config = &g_left_rear_joint_chain_config,
    .front_joint_kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
    .rear_joint_kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
    .geometry_config = &g_unconfigured_leg_geometry_config,
};

/* 右腿关节初始化配置：已实测右前对应 phi2（OC），右后对应 phi1（OA）。 */
static WheelLeggedLegInitConfig_t g_right_leg_init_config = {
    .front_joint_motor_config = DOUBLE_CLOSED_LOOP_LEG_MOTOR_CONFIG(
        &hcan2, DOUBLE_CLOSED_LOOP_LEG_RIGHT_FRONT_JOINT_TX_ID, DOUBLE_CLOSED_LOOP_LEG_RIGHT_FRONT_JOINT_RX_ID,
        MOTOR_DIRECTION_NORMAL, FEEDBACK_DIRECTION_NORMAL),
    .rear_joint_motor_config = DOUBLE_CLOSED_LOOP_LEG_MOTOR_CONFIG(
        &hcan2, DOUBLE_CLOSED_LOOP_LEG_RIGHT_REAR_JOINT_TX_ID, DOUBLE_CLOSED_LOOP_LEG_RIGHT_REAR_JOINT_RX_ID,
        MOTOR_DIRECTION_NORMAL, FEEDBACK_DIRECTION_NORMAL),
    .front_joint_chain_config = &g_right_front_joint_chain_config,
    .rear_joint_chain_config = &g_right_rear_joint_chain_config,
    .front_joint_kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
    .rear_joint_kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
    .geometry_config = &g_unconfigured_leg_geometry_config,
};

/* 左轮毂接线记录；电机型号和控制方式未确认，当前只保存地址、不初始化驱动。 */
static const WheelLeggedWheelCanConfig_t g_left_wheel_can_config = {
    .can_handle = &hcan1,
    .tx_id = DOUBLE_CLOSED_LOOP_LEG_LEFT_WHEEL_TX_ID,
    .rx_id = DOUBLE_CLOSED_LOOP_LEG_LEFT_WHEEL_RX_ID,
};

/* 右轮毂接线记录；电机型号和控制方式未确认，当前只保存地址、不初始化驱动。 */
static const WheelLeggedWheelCanConfig_t g_right_wheel_can_config = {
    .can_handle = &hcan2,
    .tx_id = DOUBLE_CLOSED_LOOP_LEG_RIGHT_WHEEL_TX_ID,
    .rx_id = DOUBLE_CLOSED_LOOP_LEG_RIGHT_WHEEL_RX_ID,
};

/* 本车完整的底盘初始化配置；通用 chassis component 仅通过该对象获取具体硬件参数。 */
static WheelLeggedChassisInitConfig_t g_chassis_init_config = {
    .left_leg_init_config = &g_left_leg_init_config,
    .right_leg_init_config = &g_right_leg_init_config,
    .left_wheel_can_config = g_left_wheel_can_config,
    .right_wheel_can_config = g_right_wheel_can_config,
};
