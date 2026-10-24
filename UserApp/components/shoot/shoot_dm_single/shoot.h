#pragma once

#include "dji_motor.h"
#include "dmmotor.h"

typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON
} Shoot_Mode_e;
typedef enum
{
    FRICTION_OFF = 0,
    FRICTION_ON
} Friction_Mode_e;
typedef enum
{
    LOAD_STOP = 0,
    LOAD_1_BULLET,
    LOAD_BURSTFIRE
} Load_Mode_e;

// 机器人层只填写命令；摩擦轮为转子侧 °/s，拨弹盘为 rad。
typedef struct
{
    Shoot_Mode_e shoot_mode;
    Friction_Mode_e friction_mode;
    float friction_speed;
    Load_Mode_e load_mode;
    uint32_t load_period_ms; // 单发逐颗调度周期；连发换算为速度；拨弹时须为正值，单位 ms。
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    Motor_Init_Config_s friction_motor_config[FRICTION_NUM];
    Motor_Init_Config_s loader_motor_config;
    float friction_coefficients[FRICTION_NUM];
    float friction_speed_min;
    float friction_speed_max;
    float loader_step_angle;      // 一发对应的 J4310 电机角度，rad，正值。
    float loader_angle_tolerance; // 到位误差，rad。
    float loader_contact_angle; // 弹丸刚抵到单发限位时的电机累计角度，rad。
    PID_Init_Config_s loader_burst_pid;
} Shoot_Init_Config_s;

// 电机累计角度目标使用 loader_motor->motor_controller.pid_ref；
// 连发斜坡速度参考使用 loader_motor->motor_controller.speed_PID.Ref。
typedef struct
{
    Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
    DJIMotorInstance *friction_motor[FRICTION_NUM];
    DMMotorInstance *loader_motor;
    const Shoot_Init_Config_s *config; // 引用长期有效的初始化配置，不重复复制固定参数。
    float friction_set;
    float loader_torque_feedforward;
    uint32_t last_load_tick;
    Load_Mode_e last_load_mode;
} ShootInstance;

/** @brief 注册三只 DJI 摩擦轮和一只 J4310；配置须在实例使用期间持续有效，当前仅初始化一个实例。 */
ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config);

/** @brief 单发以位置双环按周期逐颗拨弹，连发使用速度环；两种模式共用自动退弹状态机。 */
void ShootTask(void);
