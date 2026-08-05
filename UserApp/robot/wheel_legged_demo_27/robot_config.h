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
#define WHEEL_LEGGED_MOTOR_DIRECTION_FLAG(direction)                                                                   \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL)

/* 将本车的配对方向换算为底层达妙电机反馈方向枚举。 */
#define WHEEL_LEGGED_FEEDBACK_DIRECTION_FLAG(direction)                                                                \
    ((direction) == WHEEL_LEGGED_MOTOR_REVERSE ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL)

/* 根据关节 CAN 地址和配对方向构造一台 J4310 关节电机配置。 */
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
        .can_init_config = {                                                                                           \
            .can_handle = (can_handle_ptr),                                                                            \
            .tx_id = (tx_identifier),                                                                                  \
            .rx_id = (rx_identifier),                                                                                  \
        },                                                                                                             \
    }

/* 根据轮毂 CAN 地址和配对方向构造一台 H6215 只读反馈配置。 */
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
        .can_init_config = {                                                                                           \
            .can_handle = (can_handle_ptr),                                                                            \
            .tx_id = (tx_identifier),                                                                                  \
            .rx_id = (rx_identifier),                                                                                  \
        },                                                                                                             \
    }

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* 左右腿共用的 ACE 显式 k 缩放同心五连杆参数；所有长度单位均为 m。 */
/* TODO：填写真实 AH、HJ 和虚拟 CE，复核 AD/AH=CE/HJ 后再置 configured=1。 */
static const ParallelLegGeometry_t g_leg_geometry_config = {
    .configured = 1u,
    .l0 = 0.0f,                     /* 两个主动轴中心距，ACE 同心模型固定为 0 m。 */
    .real_first_link_ah = 0.105f,     /* AH：真实主动杆长度，待测量，单位 m。 */
    .real_second_link_hj = 0.125f,    /* HJ：真实末端杆长度，待测量，单位 m。 */
    .virtual_second_link_ce = 0.0625f, /* CE：虚拟五连杆被动杆长度，待测量，单位 m；k=CE/HJ。 */
    .virtual_end_branch_sign = -1,   /* C 相对有向线 D->E 的支路，待按实物填写 +1 或 -1。 */
    .singular_epsilon = 1e-5f,      /* 两圆重合或相切的判据，单位 m。 */
};

/* 左腿：前关节对应 phi1；后关节对应 phi2。 */
static WheelLeggedLegInitConfig_t g_left_leg_init_config = {
    .front_joint =
        {
            .motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan2, 0x0Bu, 0x0Au, WHEEL_LEGGED_MOTOR_NORMAL),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* phi1 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = -2.53547764f, /* TODO：重新标定 phi1=0 时的左前电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
        },
    .rear_joint =
        {
            .motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan2, 0x09u, 0x08u, WHEEL_LEGGED_MOTOR_NORMAL),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* phi2 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle =  -4.74803543f, /* TODO：重新标定 phi2=0 时的左后电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
        },
    .geometry_config = &g_leg_geometry_config,
    .length_control =
        {
            .force_pid_config =
                {
                    .Kp = 500.0f, /* 比例增益，单位 N/m。 */
                    .Ki = 0.0f,   /* 积分增益，单位 N/(m*s)。 */
                    .Kd = 0.0f,   /* 必须为 0；物理 D 项由下方 Jacobian 腿长速度计算。 */
                    .MaxOut = 20.0f, /* PI 与最终虚拟轴向力的唯一绝对限幅，单位 N。 */
                    .DeadBand = 0.0f, /* 初期不使用死区，避免通用 PID 在死区内清零输出。 */
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                    .IntegralLimit = 0.0f, /* Ki=0 时积分限幅为 0；启用 Ki 前需同步填写。 */
                },
            .velocity_damping_kd = 5.0f, /* Jacobian 腿长速度阻尼增益，单位 N*s/m。 */
            .length_reference = 0.160f,  /* 与当前固定工作点 LQR 一致的目标腿长，单位 m。 */
            .minimum_length = 0.07f,     /* 力控最小工作腿长，单位 m。 */
            .maximum_length = 0.19f,     /* 力控最大工作腿长，单位 m。 */
        },
};

/* 右腿：前关节对应 phi1；后关节对应 phi2。 */
static WheelLeggedLegInitConfig_t g_right_leg_init_config = {
    .front_joint =
        {
            .motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan1, 0x07u, 0x06u, WHEEL_LEGGED_MOTOR_REVERSE),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* phi1 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = 2.87193871f, /* TODO：重新标定 phi1=0 时的右前电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI1,
        },
    .rear_joint =
        {
            .motor_config = WHEEL_LEGGED_J4310_CONFIG(&hcan1, 0x05u, 0x04u, WHEEL_LEGGED_MOTOR_REVERSE),
            .chain_config =
                {
                    .configured = 1u,
                    .driving_sprocket_teeth = 12u, /* 电机侧主动链轮齿数。 */
                    .driven_sprocket_teeth = 12u,  /* phi2 主动轴侧从动链轮齿数。 */
                    .direction = 1.0f,
                    .motor_zero_angle = 0.337414742f, /* phi1=0 时的右后电机累计角，单位 rad。 */
                },
            .kinematics_input = LEG_KINEMATICS_INPUT_PHI2,
        },
    .geometry_config = &g_leg_geometry_config,
    .length_control =
        {
            .force_pid_config =
                {
                    .Kp = 500.0f, /* 比例增益，单位 N/m。 */
                    .Ki = 0.0f,   /* 积分增益，单位 N/(m*s)。 */
                    .Kd = 0.0f,   /* 必须为 0；物理 D 项由下方 Jacobian 腿长速度计算。 */
                    .MaxOut = 20.0f, /* PI 与最终虚拟轴向力的唯一绝对限幅，单位 N。 */
                    .DeadBand = 0.0f, /* 初期不使用死区，避免通用 PID 在死区内清零输出。 */
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                    .IntegralLimit = 0.0f, /* Ki=0 时积分限幅为 0；启用 Ki 前需同步填写。 */
                },
            .velocity_damping_kd = 5.0f, /* Jacobian 腿长速度阻尼增益，单位 N*s/m。 */
            .length_reference = 0.160f,  /* 与当前固定工作点 LQR 一致的目标腿长，单位 m。 */
            .minimum_length = 0.07f,     /* 力控最小工作腿长，单位 m。 */
            .maximum_length = 0.19f,     /* 力控最大工作腿长，单位 m。 */
        },
};

/* 左轮：H6215，轮径 120 mm，减速比 1:1；direction 必须通过手推前进试验复核。 */
/* TODO：手推确认左轮前进时轮端角速度和 s_dot 均为正后，冻结 left_wheel.direction。 */
static WheelLeggedWheelInitConfig_t g_left_wheel_init_config = {
    .motor_config = WHEEL_LEGGED_H6215_CONFIG(&hcan2, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_NORMAL),
    .wheel_radius = 0.0600f, /* 轮半径 60 mm，单位 m。 */
    .reduction_ratio = 1.0f,
    .direction = 1.0f, /* 暂定前进为正，手推后若 s_dot<0 则改为 -1。 */
    .configured = 1u,
};

/* 右轮：H6215，轮径 120 mm，减速比 1:1；direction 必须通过手推前进试验复核。 */
static WheelLeggedWheelInitConfig_t g_right_wheel_init_config = {
    .motor_config = WHEEL_LEGGED_H6215_CONFIG(&hcan1, 0x01u, 0x00u, WHEEL_LEGGED_MOTOR_REVERSE),
    .wheel_radius = 0.0600f, /* 轮半径 60 mm，单位 m。 */
    .reduction_ratio = 1.0f,
    .direction = 1.0f, /* 暂定前进为正，手推后若 s_dot<0 则改为 -1。 */
    .configured = 1u,
};

/* 底盘 IMU 安装配置；INS 原始角度单位为 deg，由状态模块统一换算为 rad。 */
static IMU_Init_Config_s g_chassis_imu_init_config = {
    .flag = 1u,
    .offset_flag = 1u,
    .scale = {1.0f, 1.0f, 1.0f},
    .Yaw = 0.0f,
    .Pitch = 0.0f,
    .Roll = 0.0f,
    .GyroOffset = {0.000319366809f, -0.00321031036f, -0.00178090471f},
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

/* 四输入 LQR 首次手扶接地试验的输出、安全和重力前馈配置。 */
static const WheelLeggedChassisLqrOutputConfig_t g_chassis_lqr_output_config = {
    .supported_body_mass = 1.683f,       /* 双腿支撑的机身质量，单位 kg；不含电池、双腿和双轮。 */
    .pitch_torque_limit = 0.10f,         /* 单腿虚拟 Tp 限幅，单位 N*m。 */
    .wheel_torque_limit = 0.20f,         /* 单个 H6215 轮端 Tw 限幅，单位 N*m。 */
    .wheel_torque_rate_limit = 1.0f,     /* H6215 轮端 Tw 变化率，单位 N*m/s。 */
    .minimum_support_projection = 0.20f, /* cos(theta_L)+cos(theta_R) 小于此值时禁止出力。 */
};

/* 本车底盘根配置；robot.c 只将该对象交给 chassis component。 */
static WheelLeggedChassisInitConfig_t g_chassis_init_config = {
    .left_leg_init_config = &g_left_leg_init_config,
    .right_leg_init_config = &g_right_leg_init_config,
    .left_wheel_init_config = &g_left_wheel_init_config,
    .right_wheel_init_config = &g_right_wheel_init_config,
    .imu_init_config = &g_chassis_imu_init_config,
    .state_config = &g_chassis_state_config,
    .lqr_output_config = &g_chassis_lqr_output_config,
};
