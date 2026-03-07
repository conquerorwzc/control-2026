#include "robot.h"
#include "remote_control.h"
#include "general_def.h"
#include "stdlib.h"
#include "grab.h"

typedef enum
{
    Get_the_Kernel = 0,
    Store_Kernel,
    Move_to_target,
} Half_Control_List;

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t press_l, uint8_t press_l_last, uint8_t press_r,uint8_t press_r_last);
void get_kernel(Grab_Ctrl_Cmd_s *grab_ctrl_cmd,uint8_t step);
void store_kernel(Grab_Ctrl_Cmd_s *grab_ctrl_cmd,uint8_t step);
void move_to_target(Grab_Ctrl_Cmd_s *grab_ctrl_cmd,uint8_t step);