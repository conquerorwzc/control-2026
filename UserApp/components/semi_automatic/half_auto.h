#ifndef HALF_AUTO_H
#define HALF_AUTO_H

#include "robot.h"
#include "remote_control.h"
#include "general_def.h"
#include "stdlib.h"
#include "grab.h"

typedef enum
{
    Store_First_Energy_Unit = 0, // 模式 0：存第一个能量单元
    Store_Second_Energy_Unit     // 模式 1：存第二个能量单元
} Half_Control_List;

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t press_l, uint8_t press_l_last, uint8_t press_r,uint8_t press_r_last);

// 只保留我们刚写好的两个动作函数
void store_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);
void store_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step);

#endif