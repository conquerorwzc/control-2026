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
    LOAD_BURSTFIRE,
    LOAD_REVERSE
} Load_Mode_e;

// 机器人层只填写命令；摩擦轮为转子侧 °/s，拨弹盘为 rad。
typedef struct
{
    Shoot_Mode_e shoot_mode;
    Friction_Mode_e friction_mode;
    float friction_speed;
    Load_Mode_e load_mode;
    uint32_t load_period_ms; // 连发/反转步进间隔，0 使用初始化配置。
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
    uint32_t loader_period_ms;
} Shoot_Init_Config_s;

typedef struct
{
    Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
    DJIMotorInstance *friction_motor[FRICTION_NUM];
    DMMotorInstance *loader_motor;
    float friction_coefficients[FRICTION_NUM];
    float friction_speed_min;
    float friction_speed_max;
    float friction_set;
    float loader_step_angle;
    float loader_angle_tolerance;
    uint32_t loader_period_ms;
    float loader_target;
    float loader_speed_feedforward;
    uint32_t last_load_tick;
    Load_Mode_e last_load_mode;
} ShootInstance;

/** @brief 注册三只 DJI 摩擦轮和一只 J4310；当前组件只初始化一个实例。 */
ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config);

/** @brief 周期执行；单发在进入 LOAD_1_BULLET 时触发一次，连发/反转按间隔步进。 */
void ShootTask(void);
