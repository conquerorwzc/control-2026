#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
// 大 Pitch 电机反馈坐标中的标定目标，单位 rad；须确认安装零位、方向和传动比。
#ifndef PI
#define PI 3.14159265358979f
#endif
#define UP_ANGLE 1.83f
#define DOWN_ANGLE (0.36f)
typedef enum {
    GIMBAL_POWER_OFF = 0,  // 电机零输出，不代表承重机构保持当前位置
    GIMBAL_ON,
    GIMBAL_VISION
} Gimbal_Mode_e;

// 大 Pitch 用于改变枪口高度，不参与小 Pitch 的瞄准回中。
// 姿态相对地面定义，不能直接作为电机编码器目标值。
typedef enum {
    GIMBAL_BIG_PITCH_HORIZONTAL = 0,  // 相对地面 0°
    GIMBAL_BIG_PITCH_VERTICAL,        // 相对地面 90°
} Gimbal_Big_Pitch_Pose_e;

typedef struct {
    float yaw;                // 枪口绝对 Yaw 目标，单位 deg，多圈连续角
    float pitch;              // 枪口绝对 Pitch 目标，单位 deg
    float big_yaw;            // 大 Yaw 电机反馈坐标中的目标，单位 deg，多圈连续角；上游处理安装零偏
    float chassis_rotate_wz;  // 底盘旋转角速度前馈，使用前须与速度环反馈统一单位和方向
    Gimbal_Big_Pitch_Pose_e big_pitch_pose;  // 大 Pitch 目标姿态，不表示已经到位
    Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;
/* 重力补偿极性定义
 *      o--------o---------Δ
 *     大pitch  小pitch    负载
 *     逆时针旋转 angle增加极性为正
 */
typedef struct {
    float big_pitch_scale_nm;    // 大轴近端重力矩系数，单位 N*m；不含小轴末端偏心项
    float small_pitch_scale_nm;  // 小轴末端重力矩系数，单位 N*m；同时作用于大轴

    float big_pitch_zero_rad;    // 两轴连线水平时的大轴电机反馈角，单位 rad
    float small_pitch_zero_rad;  // 大轴处于上述零位时，末端重心方向水平的小轴反馈角

    // 关节角到电机控制坐标的方向映射：正向直连为 1，反向直连为 -1。
    float big_pitch_direction;
    float small_pitch_direction;
} GravityFeedforward_Init_Config_s;

typedef struct {
    GravityFeedforward_Init_Config_s config;

    float big_pitch_feedforward;    // 最终大轴补偿力矩，单位 N*m，在速度 PID 后独立叠加
    float small_pitch_feedforward;  // 最终小轴补偿力矩，单位 N*m
} GravityFeedforwardInstance;

typedef struct {
    Motor_Init_Config_s big_yaw_motor_config;      // DM-J4310
    Motor_Init_Config_s small_yaw_motor_config;    // GM6020
    Motor_Init_Config_s big_pitch_motor_config;    // DM-J4310
    Motor_Init_Config_s small_pitch_motor_config;  // DM-J4310
    PID_Init_Config_s Yaw_Follow_PID;
    IMU_Init_Config_s imu_init_config;  // IMU 安装在枪口/小 Pitch 上，用于绝对姿态反馈
    GravityFeedforward_Init_Config_s gravity_config;  // 串联双 Pitch 重力补偿配置
} Gimbal_Init_Config_s;

typedef struct {
    Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
    DMMotorInstance* big_yaw_motor;
    DJIMotorInstance* small_yaw_motor;
    DMMotorInstance* big_pitch_motor;
    DMMotorInstance* small_pitch_motor;
    PIDInstance yaw_follow;
    INS_t* gimbal_IMU_data;  // 枪口 IMU 数据；电机编码器用于关节相对角度和机械限位
    GravityFeedforwardInstance gravity_feedforward;
} GimbalInstance;
/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config);

/**
 * @brief 云台任务
 *
 */
void GimbalTask(void);
