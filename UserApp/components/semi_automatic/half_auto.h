#ifndef HALF_AUTO_H
#define HALF_AUTO_H

#include "robot.h"
#include "remote_control.h"
#include "general_def.h"
#include "stdlib.h"
#include "grab.h"
#include "chassis.h"

typedef enum
{
    Store_First_Energy_Unit =0,
    Store_Second_Energy_Unit,
    Grab_Second_Energy_Unit,
    Grab_First_Energy_Unit,
    Climb_Step_Prep,           // 👈 新增：上台阶预备姿态
} Half_Control_List;

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd,
                      uint8_t press_l, uint8_t press_l_last, uint8_t press_r, uint8_t press_r_last);
void Half_auto_reset(Grab_Ctrl_Cmd_s *cmd);
void store_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
void store_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
void grab_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step);
void grab_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step);
void climb_step_prep(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
#endif