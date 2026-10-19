//
// Created by Dell on 2026/10/19.
//

#include "shoot.h"
#include "dji_motor.h"
#include "user_lib.h"

static ShootInstance *shoot;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
float FrictionFeedforward(int index)
{
    return 0.0f;
}
float LimitFrictionSpeedAbs(float speed, float friction_speed_min, float friction_speed_max)
{
    float sign = speed >= 0.0f ? 1.0f : -1.0f;
    float abs_speed = speed >= 0.0f ? speed : -speed;

    if (abs_speed < friction_speed_min)
    {
        abs_speed = friction_speed_min;
    }
    else if (abs_speed > friction_speed_max)
    {
        abs_speed = friction_speed_max;
    }

    return sign * abs_speed;
}

ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config)
{
    ShootInstance *shoot_instance = (ShootInstance *)zmalloc(sizeof(ShootInstance));
    for (int j = 0; j < FRICTION_NUM; j++)
        shoot_instance->friction_coefficients[j] = shoot_init_config->friction_coefficients[j];
    for (int i = 0; i < FRICTION_NUM; i++)
    {
        shoot_instance->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
    }
    shoot_instance->friction_speed_max = shoot_init_config->friction_speed_max;
    shoot_instance->friction_speed_min = shoot_init_config->friction_speed_min;
    shoot = shoot_instance;
    shoot_ctrl_cmd = &shoot_instance->shoot_ctrl_cmd; // 在运行时初始化指针
    return shoot_instance;
}

void ShootTask()
{
    if (shoot_ctrl_cmd->friction_mode == FRICTION_ON)
    {
        shoot->friction_set = LimitFrictionSpeedAbs(shoot->shoot_ctrl_cmd.friction_speed, shoot->friction_speed_min,
                                                    shoot->friction_speed_max);
    }
    else // 关闭摩擦轮
    {
        shoot->friction_set = 0;
    }
    if (shoot_ctrl_cmd->shoot_mode == SHOOT_OFF)
    {
        for (int j = 0; j < FRICTION_NUM; j++)
            DJIMotorSetPIDRef(shoot->friction_motor[j], 0);
    }
    else // 恢复运行
    {
        for (int j = 0; j < FRICTION_NUM; j++)
            DJIMotorEnable(shoot->friction_motor[j]);

        // 使用根据机器人类型设置的摩擦轮系数
        for (int i = 0; i < FRICTION_NUM; i++)
        {
            DJIMotorSetPIDRef(shoot->friction_motor[i], shoot->friction_coefficients[i] * shoot->friction_set);
            float feed_output = FrictionFeedforward(i);
            shoot->friction_motor[i]->motor_controller.final_output += feed_output;
        }
    }
}