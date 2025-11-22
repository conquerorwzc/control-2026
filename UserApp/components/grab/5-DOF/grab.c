//
// Created by ROG on 2025/11/17.
//

#include "grab.h"
#include "user_lib.h"
static GrabInstance* Grab;
static Grab_Ctrl_Cmd_s* Grab_ctrl_cmd;
int i;
GrabInstance* GrabInit(Grab_Init_Config_s* Grab_init_config)
{
  GrabInstance* grab_instance = (GrabInstance*)zmalloc(sizeof(GrabInstance));
  for (int i = 2; i >=0 ; i--)
  {
    grab_instance->Grab_motor[i]  = DMMotorInit(&Grab_init_config->Grab_motor_config[i]);
  }
  Grab = grab_instance;
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  return grab_instance;
}


void GrabTask()
{
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  for (int i = 0; i < 3; i++)
  {
    // DMMotorPIDCal(Grab->Grab_motor[0], Grab_ctrl_cmd->r1);
    // DMMotorPIDCal(Grab->Grab_motor[1], Grab_ctrl_cmd->r2);
    // DMMotorPIDCal(Grab->Grab_motor[2], Grab_ctrl_cmd->r3);
  }
}