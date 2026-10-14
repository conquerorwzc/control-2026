/**
******************************************************************************
* @file    chassis.h
* @author  NeoZeng
* @author  Annotation and Modification By SRM-Control 2026
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   Omni Chassis Module
******************************************************************************
* @attention
* Omni Chassis Motor Layout (body frame: +x forward, +y left, +z up):
*
*          motor[0]     motor[1]
*        Left Front   Right Front
*             ╭─────────────╮
*             │      ↑      │
*             │    Front    │
*             ╰─────────────╯
*          motor[2]     motor[3]
*        Left Rear    Right Rear
*
* @note    Motor Index:
*          motor[0] - Left Front Wheel
*          motor[1] - Right Front Wheel
*          motor[2] - Left Rear Wheel
*          motor[3] - Right Rear Wheel
******************************************************************************
*/
#pragma once

#include "dji_motor.h"
#include "ins_task.h"
#include "speed_observer/speed_observer.h"

typedef enum
{
    CHASSIS_POWER_OFF = 0, // 电流零输入
    CHASSIS_ROTATE,        // 小陀螺模式
    CHASSIS_FOLLOW,        // 跟随模式，底盘叠加角度环控制
    CHASSIS_HOLD,          // 航向保持模式
} Chassis_Mode_e;

/* 控制量 vx/vy 所在的坐标系 */
typedef enum
{
    CHASSIS_CMD_GIMBAL_FRAME = 0, // 云台系: 以云台当前指向为 +x, 按 offset_angle 旋转到底盘系(默认, 遥控器指令)
    CHASSIS_CMD_CHASSIS_FRAME,    // 底盘系: 以底盘正前方为 +x, 直接使用, 不随云台旋转(导航/上位机指令)
} Chassis_Cmd_Frame_e;

typedef struct
{
    // 控制部分
    // SI 速度目标，与导航同轴；原点为底盘几何中心。
    float vx; // m/s，+x 朝电池安装方向
    float vy; // m/s，+y 向左
    float wz; // rad/s，+wz 俯视逆时针；目标仍由下位机生成
    Chassis_Mode_e chassis_mode;
    Chassis_Cmd_Frame_e cmd_frame; // vx/vy 所在的坐标系, 默认云台系
    float offset_angle;            // 云台相对底盘的偏角，逆时针为正，单位度
    float yaw_hold_ref;            // 航向保持目标角度
    int chassis_speed_buff;
    uint16_t max_power; // 最大功率限制
    // UI部分
    //  ...
    uint8_t SuperCapBoost;
} Chassis_Ctrl_Cmd_s;

#pragma pack()
// 超级电容策略结构体

typedef enum
{
    SAFETY_MODE = 0,      // 安全模式，超电电压低于8伏时进入，大于18伏退出，底盘限制30W
    PASSIVE_MODE,         // 被动模式，超电电压正常时的工作模式
    ACTIVE_MODE,          // ，主动模式，主动使用超电能量
    CHARGING_MODE,        // 充电模式，衰减底盘功率，保障电容电压健康
    FORCED_CHARGING_MODE, // 强制充电模式，更极端的功率衰减，强制超电快速充电
} SuperCapMode;

typedef struct
{
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
} Power_Param_3508_s;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct
{
    float wheel_base;               // 纵向轴距(前进后退方向)
    float track_width;              // 横向轮距(左右平移方向)
    float center_gimbal_offset_x;   // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
    float center_gimbal_offset_y;   // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
    float wheel_radius;             // 轮子半径
    float wheel_reduction_ratio;    // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
    Power_Param_3508_s power_param; // 3508功率模型参数，采用中科大的模型
} Chassis_Param_s;

typedef struct
{
    Chassis_Param_s chassis_param;
    Motor_Init_Config_s wheel_motor_config[4];
    PID_Init_Config_s follow_pid;
    PID_Init_Config_s yaw_hold_pid; // 角度误差单位度，输出为 rad/s；由 enable_yaw_hold 配置初始化
    PID_Init_Config_s vx_hold_pid;  // 整车速度外环：输入/修正输出均为 m/s
    PID_Init_Config_s vy_hold_pid;  // 整车速度外环：输入/修正输出均为 m/s
    PID_Init_Config_s wz_hold_pid;  // 整车速度外环：输入/修正输出均为 rad/s
    uint8_t enable_speed_hold;     // 三轴速度外环开关，适用于所有底盘使能模式
    IMU_Init_Config_s imu_init_config;
    uint8_t enable_imu_sampling; // 独立开启 INS 采集，用于安装方向核对，不自动启用速度融合
    uint8_t enable_yaw_hold;
    SpeedObserverConfig_s observer_config;
    uint8_t observer_imu_calibrated; // 仅在 INS 输出轴向和陀螺仪零偏核对后置 1
    uint8_t observer_accel_enabled; // 还需确认重力补偿与安装偏移后置 1
    float observer_imu_offset[2];   // 底盘中心 -> IMU，底盘系 m；INS CenterOffset 必须保持 0
    float observer_sensor_timeout;  // 观测器采样新鲜度限制，s；不修改底盘急停策略
} Chassis_Init_Config_s;


typedef struct
{
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
    DJIMotorInstance *wheel_motor[4]; // LF/ID1, RF/ID4, LB/ID2, RB/ID3
    SpeedObserver_s speed_observer; // wheel 为轮速正解，fused 为卡尔曼融合速度
    PIDInstance vx_hold_pid;
    PIDInstance vy_hold_pid;
    PIDInstance wz_hold_pid;
    uint8_t enable_speed_hold;
    uint8_t speed_hold_active;       // 三轴 PID 已预置历史并处于闭环状态
    SuperCapMode super_cap_mode;
    INS_t *chassis_IMU;
    PIDInstance yaw_hold_pid;
} ChassisInstance;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 *
 */
void ChassisTask();
