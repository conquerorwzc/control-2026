/**
******************************************************************************
* @file    chassis.h
* @author  NeoZeng
* @author  Annotation and Modification By SRM-Control 2026
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   Mecanum Chassis Module
******************************************************************************
* @attention
* Mecanum Chassis Motor Layout:
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
#include "trajectory_planner.h"

typedef enum
{
    CHASSIS_POWER_OFF,
    CHASSIS_FOLLOW,
    CHASSIS_CALIBRATING,
    CHASSIS_CLIMB_IDLE,
    CHASSIS_CLIMB_ALL_RETRACT,
    CHASSIS_CLIMB_BOTH_EXTEND,
    CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF,
    CHASSIS_CLIMB_FRONT_RETRACT
} Chassis_Mode_e;

typedef enum
{
    ROBOT_POWER_OFF = 0,
    ROBOT_POWER_ON,          // 正常行车模式
    ROBOT_EXCHANGE_MODE,     // 兑换模式
    ROBOT_CLIMB_MODE,        // 上台阶模式
    ROBOT_DOWN_STAIRS_MODE,  // 下台阶模式 
    ROBOT_EMERGENCY_STOP
} Robot_Mode_e;


typedef enum
{
    CLIMB_STAGE_IDLE = 0,      // 平地/复位状态 (全收)
    CLIMB_STAGE_BOTH_EXTEND,   // 阶段1: 全伸 (准备上台阶)
    CLIMB_STAGE_FRONT_RETRACT, // 阶段2: 只收前 (前轮已上，屁股还抬着)
    CLIMB_STAGE_ALL_RETRACT    // 阶段3: 全收 (上完台阶/复位)
} ClimbState_e;

typedef struct {
    uint8_t all_cali_done;     // 零点全部完成标志 (最重要的护盾判断条件)
    uint8_t has_calibrated_once; // 记录是否完成过首次标定
    uint8_t cali_done[4];      // 各腿独立零点标志 [0后左, 1后右, 2前左, 3前右]
    float   init_angle[4];     // 零点真实物理坐标

    uint8_t is_max_calibrated; // 最大行程全部完成标志
    uint8_t max_cali_done[4];  // 各腿独立最大行程标志
    float   max_angle[4];      // 最大极限物理坐标
} Chassis_Cali_State_s;

typedef struct
{
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度
    Chassis_Mode_e chassis_mode;
    ClimbState_e climb_state;
    uint8_t robot_mode;
    float lift_ratio;
    float offset_angle; // 底盘和归中位置的夹角
    int chassis_speed_buff;
    uint16_t max_power;  // 最大功率限制
    int16_t lift_height; // 抬升高度
    float forward_lift_in;          // 导杆收回位置
    float forward_lift_out;         // 导杆伸出位置
    float backward_lift_in;         // 腿抬升收回位置
    float backward_lift_out;       // 腿抬升伸出位置
                         // UI部分
                         //  ...

} Chassis_Ctrl_Cmd_s;

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
    float forward_lift_in;          // 导杆收回位置
    float forward_lift_out;         // 导杆伸出位置
    float backward_lift_in;         // 腿抬升收回位置
    float backward_lift_out;       // 腿抬升伸出位置
    float climb_tilt_ratio;  // 烂路模式 (前收后伸) 时的“撅屁股”程度比例 (0.0 ~ 1.0)
} Chassis_Param_s;

typedef struct
{
    Chassis_Param_s chassis_param;
    Motor_Init_Config_s wheel_motor_config[4];
    Motor_Init_Config_s lift_forward_motor_config[2];
    Motor_Init_Config_s lift_backward_motor_config[2];
    PID_Init_Config_s follow_pid;
} Chassis_Init_Config_s;

typedef struct {
    DJIMotorInstance *motor;      // 绑定的电机硬件躯干
    TrapezoidalPlanner_t planner; // 专属数学大脑
    float *ff_channel;            // 前馈输出通道的挂载点

    uint8_t use_curve;            // 功能开关：1用曲线(前腿)，0直接发(后腿)
    float moving_max_out;         // 运动时的最大电流爆发值
    float stop_max_out;           // 驻车时的安全电流维持值

    float target_pos;             // 外界的最终绝对目标位置
} LiftLeg_t;

typedef struct
{
    Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
    Chassis_Cali_State_s cali_state;
    DJIMotorInstance *wheel_motor[4];          // left right forward back
    LiftLeg_t front_legs[2];
    LiftLeg_t rear_legs[2];
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