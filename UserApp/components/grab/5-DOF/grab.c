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
  grab_instance->Grab_motor = DMMotorInit(&Grab_init_config->Grab_motor_config);
  Grab = grab_instance;
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  return grab_instance;
}


void GrabTask()
{
  Grab_ctrl_cmd = &Grab->Grab_ctrl_cmd;
  DMMotorPIDCal(Grab->Grab_motor, 200);
}