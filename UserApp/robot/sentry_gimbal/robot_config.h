#pragma once

#include "gimbal.h"
#include "shoot.h"
#include "general_def.h"

// 云台板统一控制四轴云台、J4310 拨弹盘和三只 M2006 摩擦轮。
#define GIMBAL_BOARD

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

// 四轴参数同步自 3e3c6aa8；大 Yaw 按联合运行约定保留 0x04/0x14。
#define ALIGN_DEG (-173.0f)  // deg，小 Yaw 机械对齐零位
/*
| 对象 | 角度反馈 | 速度反馈 | 速度环输出 |
| 大 Yaw：DM-J4310 | 电机累计角度，rad | 电机速度，rad/s | DM 力矩指令 |
| 大 Pitch：DM-J4310 | 电机累计角度，rad | 电机速度，rad/s | DM 力矩指令 |
| 小 Yaw：DM-J4310 | IMU 累计 Yaw，° | `Gyro[2]`，rad/s | DM 力矩指令，N*m |
| 小 Pitch：DM-J4310 | IMU Pitch，° | `Gyro[0]`，rad/s | DM 力矩指令 |
 *
 */
static Gimbal_Init_Config_s gimbal_init_config = {
    .big_pitch_motor_config =
        {
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x02,
                    .rx_id = 0x12,
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 0.7f,              // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 3.0f,              // N*m/rad，速度误差积分增益
                            .Kd = 0.0f,              // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 1.5f,          // N*m，速度环输出力矩限幅,4310额定3，峰值7
                            .IntegralLimit = 0.35f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 18.0f,            // (rad/s)/rad = 1/s，角度误差到目标速度的比例增益
                            .Ki = 0.1f,             // (rad/s)/(rad*s) = 1/s^2，角度误差积分增益
                            .Kd = 0.6f,             // (rad/s)/(rad/s)，无量纲，角度误差微分增益；启用微分先行
                            .DeadBand = 0.005f,      // rad，约 0.573 deg，角度误差死区
                            .MaxOut = 4.0f,         // rad/s，目标速度限幅；可减小以抑制目标切换时的速度尖峰
                            //todo：lsy要求速度还要再快一点，思考一下怎么调参，因为两个角度直接切换目标突变太大导致会有一个尖峰，建议对速度规划/加一个斜坡去优化
                            .IntegralLimit = 0.5f,  // rad/s，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                        },
                },
        },
    .small_pitch_motor_config =
        {
            .controller_setting_init_config =
                {.motor_reverse_flag = 1},
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x07,
                    .rx_id = 0x06,
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 0.5f,              // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 1.5f,              // N*m/rad，速度误差积分增益
                            .Kd = 0.0f,              // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 1.5f,          // N*m，速度环输出力矩限幅
                            .IntegralLimit = 0.25f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 0.50f,        // (rad/s)/deg，IMU 角度误差到目标速度的比例增益
                            .Ki = 0.0f,         // (rad/s)/(deg*s)，角度误差积分增益
                            .Kd = 0.0f,         // (rad/s)/(deg/s)，角度误差微分增益
                            .DeadBand = 0.01f,  // deg，IMU 角度误差死区
                            .MaxOut = 3.2f,     // rad/s，目标速度限幅；0.05 rad、2.5 Hz 的理想速度峰值为 0.785 rad/s
                            .IntegralLimit = 0.0f,        // rad/s，积分输出限幅
                            .Improve = PID_IMPROVE_NONE,  // 无量纲，优化功能位标志
                        },
                },
        },
    .big_yaw_motor_config =
        {

            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0x04, // 联合运行保留原硬件 CAN ID，避免反馈与拨弹盘重叠。
                    .rx_id = 0x14,
                },
            .controller_param_init_config =
                {

                    .speed_PID =
                        {
                            .Kp = 0.81f,             // N*m/(rad/s)，速度误差到力矩的比例增益
                            .Ki = 4.63f,             // N*m/rad，速度误差积分增益
                            .Kd = 0.00f,             // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,        // rad/s，速度误差死区
                            .MaxOut = 1.5f,          // N*m，速度环输出力矩限幅
                            .IntegralLimit = 0.35f,  // N*m，积分输出限幅
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,  // 无量纲，优化功能位标志
                        },
                    .angle_PID =
                        {
                            .Kp = 20.0f,                    // (rad/s)/rad = 1/s，角度误差到目标速度的比例增益
                            .Kd = 0.0f,                     // (rad/s)/(rad/s)，无量纲，角度误差微分增益
                            .Ki = 0.0f,                     // (rad/s)/(rad*s) = 1/s^2，角度误差积分增益
                            .DeadBand = 0.0f,              // rad，角度误差死区
                            .MaxOut = 25.0f,                // rad/s，目标速度限幅
                            .IntegralLimit = 0.05f,         // rad/s，积分输出限幅
                            .Improve = PID_Integral_Limit,  // 无量纲，优化功能位标志
                        },
                },
        },
    .small_yaw_motor_config =
        {
            .controller_setting_init_config =
                {.motor_reverse_flag = 1},
            .motor_type = J4310,
            .can_init_config =
                {
                    .can_handle = &hcan1,
                    .tx_id = 0X0B,
                    .rx_id = 0X0A
                },
            .controller_param_init_config =
                {
                    .speed_PID =
                        {
                            .Kp = 0.7f,                // N*m/(rad/s)，数值复用大 Pitch 速度环
                            .Ki = 3.0f,                // N*m/rad，数值复用大 Pitch 速度环
                            .Kd = 0.0f,                // N*m/(rad/s^2)，速度误差微分增益
                            .DeadBand = 0.0f,          // rad/s，速度误差死区
                            .MaxOut = 1.5f,           // N*m，数值复用大 Pitch 速度环
                            .IntegralLimit = 0.35f,   // N*m，数值复用大 Pitch 速度环
                            .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                        },
                    .angle_PID =
                        {
                            .Kp = 0.1f,                   // (rad/s)/deg，保留原小 Yaw IMU 角度环参数
                            .Ki = 0.0f,                   // (rad/s)/(deg*s)
                            .Kd = 0.0f,                   // (rad/s)/(deg/s)
                            .DeadBand = 0.15f,            // deg，IMU 角度误差死区
                            .MaxOut = 6.0f,               // rad/s，目标速度限幅
                            .IntegralLimit = 0.0f,        // rad/s，积分输出限幅
                            .Improve = PID_IMPROVE_NONE,
                        },
                },
        },
    .gravity_config =
        {
            .big_pitch_scale_nm = 0.0f,    // N*m，大轴近端重力矩系数，不含小轴末端偏心项
            .small_pitch_scale_nm = 0.0f,  // N*m，末端偏心重力矩系数，同时计入大 Pitch
            .big_pitch_zero_rad = 3.5f,     // rad，两轴连线水平时的大轴反馈角
            .small_pitch_zero_rad = 2.5f,   // rad，大轴处于上述零位时末端重心方向水平的小轴反馈角
            .big_pitch_direction = -1.0f,   // 无量纲，正向直连为 1，反向直连为 -1
            .small_pitch_direction = 1.0f,  // 无量纲，正向直连为 1，反向直连为 -1
        },
    .imu_init_config =
        {

            .flag = 1,
            .offset_flag = 1,
            .scale = {1.0f, 1.0f, 1.0f},
            .Yaw = -90.0f,
            .Pitch = 0.0f,
            .Roll = 0.0f,
            .GyroOffset = {0.00382259092, -0.00479286071, -0.00198436622},
        },
    // Follow 输出直接累加到大 Yaw 目标，单位 deg/次，当前实现未在累加处乘 dt。
    .Yaw_Follow_PID =
        {
            .Kp = 0.0035f,          // (deg/次)/deg，偏角到每次目标角度增量的比例增益
            .Ki = 0.000f,           // (deg/次)/(deg*s)，偏角积分增益
            .Kd = 0.000f,           // (deg/次)/(deg/s)，偏角微分增益；启用微分先行
            .DeadBand = 0.3f,     // deg，小 Yaw 相对偏角死区
            //todo：陀螺仪零飘有点大没擦好导致follow环死区不开大大yaw会1hz左右来回0.几度动，建议重新擦一下零飘或者在线擦？
            .MaxOut = 10.0f,        // deg/次，每次目标角度增量限幅
            .IntegralLimit = 1.0f,  // deg/次，积分输出限幅
            .Improve =
                PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,  // 无量纲，优化功能位标志
        },
};
