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
    Store_First_Energy_Unit = 0,  // 模式 0：存第一个
    Store_Second_Energy_Unit,     // 模式 1：存第二个
    Grab_Six_Oclock_Energy_Unit,  // 模式 2：取六点钟
    Grab_Four_Oclock_Energy_Unit  // 💥 模式 3：取四点钟 (新增)
} Half_Control_List;

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd,
                      uint8_t press_l, uint8_t press_l_last, uint8_t press_r, uint8_t press_r_last);

void store_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
void store_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
void grab_six_oclock_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step);

// 💥 新增：取四点钟函数声明
void grab_four_oclock_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step);

#endif