//
// Created by Dell on 2026/10/19.
//

#pragma once

#include "dji_motor.h"

#define SHOOT_CNT_MAX 1

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} Shoot_Mode_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} Friction_Mode_e;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    Shoot_Mode_e shoot_mode;
    Friction_Mode_e friction_mode;
    float friction_speed; // 摩擦轮转速
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    Motor_Init_Config_s friction_motor_config[FRICTION_NUM];
    float friction_coefficients[FRICTION_NUM];
    float friction_speed_min;
    float friction_speed_max;
} Shoot_Init_Config_s;

typedef struct
{
    Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
    DJIMotorInstance *friction_motor[FRICTION_NUM];
    float friction_coefficients[FRICTION_NUM];
    float friction_set;
    float friction_speed_min;
    float friction_speed_max;
} ShootInstance;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config);
/**
 * @brief 云台任务
 *
 */
void ShootTask();