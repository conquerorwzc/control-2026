//
// Created by Dell on 2026/10/19.
//

#include "shoot.h"
#include "dji_motor.h"
#include "user_lib.h"

static uint8_t idx = 0;  // register idx,是该文件的全局索引,在注册时使用
static ShootInstance* shoot;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;

ShootInstance* ShootInit(Shoot_Init_Config_s* shoot_init_config)
{
    ShootInstance* shoot_instance = (ShootInstance*)zmalloc(sizeof(ShootInstance));

    for (int i = 0; i < FRICTION_NUM; i++) {
        shoot_instance->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
    }

    shoot = shoot_instance;
    shoot_ctrl_cmd = &shoot_instance->shoot_ctrl_cmd;  // 在运行时初始化指针
    idx++;
    return shoot_instance;
}

void ShootTask()
{
    if (shoot_ctrl_cmd->shoot_mode == SHOOT_OFF) {
        for (int j = 0; j < FRICTION_NUM; j++) DJIMotorSetPIDRef(shoot->friction_motor[j], 0);
    } else  // 恢复运行
    {
        for (int j = 0; j < FRICTION_NUM; j++) DJIMotorEnable(shoot->friction_motor[j]);

        // 使用根据机器人类型设置的摩擦轮系数
        for (int i = 0; i < FRICTION_NUM; i++) {
            DJIMotorSetPIDRef(shoot->friction_motor[i], friction_coefficients[i] * friction_set);
            float feed_output = GetFrictionFeedforward(i);
            shoot->friction_motor[i]->motor_controller.final_output += feed_output;
        }
    }
}