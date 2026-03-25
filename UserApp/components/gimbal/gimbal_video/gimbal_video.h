/**
* @file    gimbal_video.h
 * @brief   图传云台控制逻辑头文件
 */

#pragma once

#include "dji_motor.h"
#include "daemon.h"

/* ----------- 对外依赖：断电状态来自 robot 层 ----------- */
typedef enum {
    VIDEO_POWER_OFF = 0,
    VIDEO_POWER_ON,
} VideoGimbal_Power_e;

/* ----------- 图传标定状态机 ----------- */
typedef enum {
    VIDEO_CALI_START = 0,     // 上电/重连：初始化零点
    VIDEO_CALI_WAIT_BTN,      // 等待操作员触发 Pitch 标定
    VIDEO_CALI_START_PITCH,   // 准备开始 Pitch 双向寻零
    VIDEO_CALI_FIND_MAX,      // 向上寻找 Pitch 最高限位
    VIDEO_CALI_FIND_MIN,      // 向下寻找 Pitch 最低限位
    VIDEO_CALI_DONE,          // 标定完成
    VIDEO_CALI_ERROR          // 标定超时/异常
} VideoCaliState_e;

/* ----------- 上层控制指令 ----------- */
typedef struct {
    float video_yaw;          // Yaw 控制增量（每帧）
    float video_pitch;        // Pitch 控制绝对量（0.0~1.0）

    uint8_t video_cali;       // 触发 Pitch 双向堵转标定

    VideoGimbal_Power_e power; // 断电联动
} VideoGimbal_Ctrl_Cmd_s;

/* ----------- 初始化配置 ----------- */
typedef struct {
    Motor_Init_Config_s yaw_motor_config;   // GM6020 Yaw
    Motor_Init_Config_s pitch_motor_config; // M3508  Pitch

    // GM6020 Yaw 固定物理零点（total_angle 单位，度）
    float yaw_zero_angle;
} VideoGimbal_Init_Config_s;

/* ----------- 组件实例 ----------- */
typedef struct {
    VideoGimbal_Ctrl_Cmd_s ctrl_cmd;

    DJIMotorInstance *yaw_motor;    // GM6020
    DJIMotorInstance *pitch_motor;  // M3508

    /* 控制量 */
    float Video_yaw;    // 累积 yaw 控制量（相对零点）
    float Video_pitch;  // 0.0~1.0 百分比

    /* 底层 PID 目标 */
    float Y_target;
    float P_target;

    /* Pitch 标定范围 */
    float video_max_p;
    float video_min_p;

    /* 标定状态机 */
    VideoCaliState_e video_cali_state;

    /* GM6020 Yaw 绝对零点标定 */
    float   yaw_zero_angle;       // 记录的物理零点（total_angle，度）
    uint8_t yaw_zero_cali_done;   // 0=未标定，1=已标定
} VideoGimbalInstance;

/* ----------- 对外接口 ----------- */
VideoGimbalInstance *VideoGimbalInit(VideoGimbal_Init_Config_s *config);
void VideoGimbalTask(void);