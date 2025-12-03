//
// Created by ROG on 2025/11/17.
//

#include "grab.h"

#include "user_lib.h"

static GrabInstance* grab;
static Grab_Ctrl_Cmd_s* grab_ctrl_cmd;

GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config);
void GrabTask();
void MotorTask();

int a[5] = {0, 0, 0, 0, 0};

/**
 * @brief 初始化机械臂
 */
GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config) {
  GrabInstance* grab_instance = (GrabInstance*)zmalloc(sizeof(GrabInstance));
  // for (int i = 2; i >=0 ; i--)
  // {
  //   grab_instance->Grab_dmmotor[i]  = DMMotorInit(&Grab_init_config->Grab_motor_config[i]);
  // }
  grab_instance->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
  grab_instance->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
  grab_instance->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);  // v3
  grab_instance->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);  // v3
  grab_instance->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);  // v2
  grab = grab_instance;
  grab_ctrl_cmd = &grab->grab_ctrl_cmd;
  return grab_instance;
}

/**
 * @brief 机械臂任务函数
 */
void GrabTask() {
  grab_ctrl_cmd = &grab->grab_ctrl_cmd;
  MotorTask();

}

void MotorTask() {
  if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF) {
    DMMotorStop(grab->grab_dmmotor[0]);
    DMMotorStop(grab->grab_dmmotor[1]);
    DMMotorStop(grab->grab_dmmotor[2]);
    DJIMotorStop(grab->grab_djimotor[0]);
    DJIMotorStop(grab->grab_djimotor[1]);
  } else {
    DMMotorEnable(grab->grab_dmmotor[0]);
    DMMotorEnable(grab->grab_dmmotor[1]);
    DMMotorEnable(grab->grab_dmmotor[2]);
    DJIMotorEnable(grab->grab_djimotor[0]);
    DJIMotorEnable(grab->grab_djimotor[1]);
    DMMotorPIDCal(grab->grab_dmmotor[0], a[0]);
    DMMotorPIDCal(grab->grab_dmmotor[1], a[1]);
    DMMotorPIDCal(grab->grab_dmmotor[2], a[2]);
    DJIMotorSetPIDRef(grab->grab_djimotor[0], a[3]);
    DJIMotorSetPIDRef(grab->grab_djimotor[1], a[3]);
  }
}