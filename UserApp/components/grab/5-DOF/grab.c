//
// Created by ROG on 2025/11/17.
//

#include "grab.h"
#include "user_lib.h"

static GrabInstance* Grab;
static Grab_Ctrl_Cmd_s* Grab_ctrl_cmd;

int a[5]={0,0,0,0,0};
GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config)
{
  GrabInstance* grab_instance = (GrabInstance*)zmalloc(sizeof(GrabInstance));
  // for (int i = 2; i >=0 ; i--)
  // {
  //   grab_instance->Grab_dmmotor[i]  = DMMotorInit(&Grab_init_config->Grab_motor_config[i]);
  // }
  grab_instance->Grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
  grab_instance->Grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
  grab_instance->Grab_dmmotor[0]  = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);//v3
  grab_instance->Grab_dmmotor[1]  = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);//v3
  grab_instance->Grab_dmmotor[2]  = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);//v2
  Grab = grab_instance;
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  return grab_instance;
}


void GrabTask()
{
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  if (Grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {DMMotorStop(Grab->Grab_dmmotor[0]);
     DMMotorStop(Grab->Grab_dmmotor[1]);
     DMMotorStop(Grab->Grab_dmmotor[2]);
     DJIMotorStop(Grab->Grab_djimotor[0]);
     DJIMotorStop(Grab->Grab_djimotor[1]);
    }
   else
   {
    DMMotorEnable(Grab->Grab_dmmotor[0]);
    DMMotorEnable(Grab->Grab_dmmotor[1]);
    DMMotorEnable(Grab->Grab_dmmotor[2]);
    DJIMotorEnable(Grab->Grab_djimotor[0]);
    DJIMotorEnable(Grab->Grab_djimotor[1]);
    DMMotorPIDCal(Grab->Grab_dmmotor[0], a[0]);
    // DMMotorPIDCal(Grab->Grab_dmmotor[1], a[1]);
    // DMMotorPIDCal(Grab->Grab_dmmotor[2], a[2]);
    DJIMotorSetPIDRef(Grab->Grab_djimotor[0], a[3]);
    DJIMotorSetPIDRef(Grab->Grab_djimotor[1], a[3]);
   }
}