#include "half_auto.h"
void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t press_l, uint8_t press_l_last, uint8_t press_r,
                      uint8_t press_r_last)
{
    static int step = 0;
    static Half_Control_List half_control_list = Get_the_Kernel;
    if (press_r && !press_r_last)
    {
        half_control_list = (half_control_list + 1) % 3; // 每次按下右键切换半自动控制步骤
        step = 0;                                        // 切换步骤时重置当前步骤计数
    }
    if (press_l && !press_l_last) // 按下左键时加载下一步动作，具体动作根据half_control_list的值来执行
    {
        step++;
    }
    switch (half_control_list)
    {
    case Get_the_Kernel:
        get_kernel(grab_ctrl_cmd, step);
        break;
    case Store_Kernel:
        store_kernel(grab_ctrl_cmd, step);
        break;
    case Move_to_target:
        move_to_target(grab_ctrl_cmd, step);
        break;
    default:
        break;
    }

}

 void get_kernel(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 1:
        grab_ctrl_cmd->base_joint = 10.0f;
        break;
    case 2:
        grab_ctrl_cmd->elbow_pitch = 20.0f;
        break;
    case 3:
        grab_ctrl_cmd->elbow_roll = 10.0f;
        break;
    case 4:
        grab_ctrl_cmd->wrist_pitch = 15.0f;
        break;
    case 5:
        grab_ctrl_cmd->wrist_roll = 5.0f;
        break;
    case 6:
        grab_ctrl_cmd->torque = -5.0f;
        break;
    default:
        break;
    }
}

void store_kernel(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 1:
        grab_ctrl_cmd->elbow_pitch = 45.0f;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 90.0f;
        break;
    case 3:
        grab_ctrl_cmd->elbow_pitch = 30.0f;
        break;
    case 4:
        grab_ctrl_cmd->wrist_pitch = 0.0f;
        break;
    case 5:
        grab_ctrl_cmd->torque = 2.0f;
        break;
    default:
        break;
    }
}

void move_to_target(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 1:
        grab_ctrl_cmd->base_joint = 180.0f;
        break;
    case 2:
        grab_ctrl_cmd->elbow_pitch = 50.0f;
        break;
    case 3:
        grab_ctrl_cmd->elbow_roll = 0.0f;
        break;
    case 4:
        grab_ctrl_cmd->wrist_pitch = 30.0f;
        break;
    case 5:
        grab_ctrl_cmd->wrist_roll = 0.0f;
        break;
    default:
        break;
    }
}