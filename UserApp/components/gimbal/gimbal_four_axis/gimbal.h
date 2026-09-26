#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
// 大 Pitch 电机反馈坐标中的标定目标，单位 rad；须确认安装零位、方向和传动比。
#ifndef PI
#define PI 3.14159265358979f
#endif
#define UP_ANGLE 0.83f
#define DOWN_ANGLE (-0.54f)
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

typedef struct {
    Motor_Init_Config_s big_yaw_motor_config;      // DM-J4310
    Motor_Init_Config_s small_yaw_motor_config;    // GM6020
    Motor_Init_Config_s big_pitch_motor_config;    // DM-J4310
    Motor_Init_Config_s small_pitch_motor_config;  // DM-J4310
    PID_Init_Config_s Yaw_Follow_PID;
    IMU_Init_Config_s imu_init_config;  // IMU 安装在枪口/小 Pitch 上，用于绝对姿态反馈
    float big_pitch_feedforward_scale;    // 大 Pitch 重力补偿系数，使用 DM 力矩输出单位
    float small_pitch_feedforward_scale;  // 小 Pitch 重力补偿系数，使用 DM 力矩输出单位
} Gimbal_Init_Config_s;

typedef struct {
    Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
    DMMotorInstance* big_yaw_motor;
    DJIMotorInstance* small_yaw_motor;
    DMMotorInstance* big_pitch_motor;
    DMMotorInstance* small_pitch_motor;
    PIDInstance yaw_follow;
    INS_t* gimbal_IMU_data;  // 枪口 IMU 数据；电机编码器用于关节相对角度和机械限位
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
