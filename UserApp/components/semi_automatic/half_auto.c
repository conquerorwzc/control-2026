#include "half_auto.h"

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t press_l, uint8_t press_l_last, uint8_t press_r,
                      uint8_t press_r_last)
{
    static int step = 0;
    // 默认初始状态为存第一个能量单元
    static Half_Control_List half_control_list = Store_First_Energy_Unit;

    if (press_r && !press_r_last)
    {
        // 只有 2 个模式了，对 2 取余
        half_control_list = (half_control_list + 1) % 2;
        step = 0;
    }

    if (press_l && !press_l_last)
    {
        step++;
    }

    // 状态路由
    switch (half_control_list)
    {
    case Store_First_Energy_Unit:
        store_first_energy_unit(grab_ctrl_cmd, step);
        break;
    case Store_Second_Energy_Unit:
        store_second_energy_unit(grab_ctrl_cmd, step);
        break;
    default:
        break;
    }
}

// ========================================================
// 模式 0：存第一个能量单元 (共 7 步)
// ========================================================
void store_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 1:
        grab_ctrl_cmd->base_joint  = 1.75f;
        grab_ctrl_cmd->elbow_roll  = 6.54f;
        grab_ctrl_cmd->elbow_pitch = 2.78f;
        grab_ctrl_cmd->wrist_pitch = 73.21f;
        grab_ctrl_cmd->wrist_roll  = 0.08f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 2:
        grab_ctrl_cmd->base_joint  = 11.99f;
        grab_ctrl_cmd->elbow_roll  = 54.17f;
        grab_ctrl_cmd->elbow_pitch = 32.20f;
        grab_ctrl_cmd->wrist_pitch = 60.33f;
        grab_ctrl_cmd->wrist_roll  = -2.85f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 3:
        grab_ctrl_cmd->base_joint  = 5.31f;
        grab_ctrl_cmd->elbow_roll  = 78.21f;
        grab_ctrl_cmd->elbow_pitch = 37.91f;
        grab_ctrl_cmd->wrist_pitch = 52.82f;
        grab_ctrl_cmd->wrist_roll  = -17.27f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 4:
        grab_ctrl_cmd->base_joint  = 8.48f;
        grab_ctrl_cmd->elbow_roll  = 82.67f;
        grab_ctrl_cmd->elbow_pitch = 45.05f;
        grab_ctrl_cmd->wrist_pitch = 55.94f;
        grab_ctrl_cmd->wrist_roll  = -40.29f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 5:
        grab_ctrl_cmd->base_joint  = 11.07f;
        grab_ctrl_cmd->elbow_roll  = 86.80f;
        grab_ctrl_cmd->elbow_pitch = 58.30f;
        grab_ctrl_cmd->wrist_pitch = 61.12f;
        grab_ctrl_cmd->wrist_roll  = -55.19f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 6: // 💥 关键节点：释放能量单元
        grab_ctrl_cmd->base_joint  = 9.14f;
        grab_ctrl_cmd->elbow_roll  = 93.16f;
        grab_ctrl_cmd->elbow_pitch = 56.77f;
        grab_ctrl_cmd->wrist_pitch = 54.97f;
        grab_ctrl_cmd->wrist_roll  = -33.26f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 7: // 安全收回
        grab_ctrl_cmd->base_joint  = 0.04f;
        grab_ctrl_cmd->elbow_roll  = 7.42f;
        grab_ctrl_cmd->elbow_pitch = 13.71f;
        grab_ctrl_cmd->wrist_pitch = 5.75f;
        grab_ctrl_cmd->wrist_roll  = -0.30f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    default:
        break;
    }
}

// ========================================================
// 模式 1：存第二个能量单元 (共 15 步)
// ========================================================
void store_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 1:
        grab_ctrl_cmd->base_joint  = -6.02f;
        grab_ctrl_cmd->elbow_roll  = -1.32f;
        grab_ctrl_cmd->elbow_pitch = 6.19f;
        grab_ctrl_cmd->wrist_pitch = 59.45f;
        grab_ctrl_cmd->wrist_roll  = -1.31f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 2:
        grab_ctrl_cmd->base_joint  = 0.61f;
        grab_ctrl_cmd->elbow_roll  = 16.68f;
        grab_ctrl_cmd->elbow_pitch = 6.61f;
        grab_ctrl_cmd->wrist_pitch = 55.45f;
        grab_ctrl_cmd->wrist_roll  = -26.93f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 3:
        grab_ctrl_cmd->base_joint  = -0.08f;
        grab_ctrl_cmd->elbow_roll  = 42.85f;
        grab_ctrl_cmd->elbow_pitch = 9.08f;
        grab_ctrl_cmd->wrist_pitch = 68.90f;
        grab_ctrl_cmd->wrist_roll  = -21.92f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 4:
        grab_ctrl_cmd->base_joint  = 10.19f;
        grab_ctrl_cmd->elbow_roll  = 56.57f;
        grab_ctrl_cmd->elbow_pitch = 22.45f;
        grab_ctrl_cmd->wrist_pitch = 74.61f;
        grab_ctrl_cmd->wrist_roll  = -33.70f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 5:
        grab_ctrl_cmd->base_joint  = 14.67f;
        grab_ctrl_cmd->elbow_roll  = 68.53f;
        grab_ctrl_cmd->elbow_pitch = 33.21f;
        grab_ctrl_cmd->wrist_pitch = 72.02f;
        grab_ctrl_cmd->wrist_roll  = -36.95f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 6:
        grab_ctrl_cmd->base_joint  = 16.03f;
        grab_ctrl_cmd->elbow_roll  = 76.26f;
        grab_ctrl_cmd->elbow_pitch = 41.16f;
        grab_ctrl_cmd->wrist_pitch = 74.48f;
        grab_ctrl_cmd->wrist_roll  = -46.18f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 7:
        grab_ctrl_cmd->base_joint  = 26.01f;
        grab_ctrl_cmd->elbow_roll  = 82.23f;
        grab_ctrl_cmd->elbow_pitch = 65.84f;
        grab_ctrl_cmd->wrist_pitch = 49.04f;
        grab_ctrl_cmd->wrist_roll  = -42.31f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 8:
        grab_ctrl_cmd->base_joint  = 26.45f;
        grab_ctrl_cmd->elbow_roll  = 80.48f;
        grab_ctrl_cmd->elbow_pitch = 73.73f;
        grab_ctrl_cmd->wrist_pitch = 48.38f;
        grab_ctrl_cmd->wrist_roll  = -38.93f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 9:
        grab_ctrl_cmd->base_joint  = 30.14f;
        grab_ctrl_cmd->elbow_roll  = 87.17f;
        grab_ctrl_cmd->elbow_pitch = 76.96f;
        grab_ctrl_cmd->wrist_pitch = 48.95f;
        grab_ctrl_cmd->wrist_roll  = -40.91f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 10: // 释放前的最后对齐
        grab_ctrl_cmd->base_joint  = 31.42f;
        grab_ctrl_cmd->elbow_roll  = 88.66f;
        grab_ctrl_cmd->elbow_pitch = 81.07f;
        grab_ctrl_cmd->wrist_pitch = 48.95f;
        grab_ctrl_cmd->wrist_roll  = -40.95f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 11: // 💥 关键点：松开夹爪释放能量单元！
        grab_ctrl_cmd->base_joint  = 31.64f;
        grab_ctrl_cmd->elbow_roll  = 94.19f;
        grab_ctrl_cmd->elbow_pitch = 78.43f;
        grab_ctrl_cmd->wrist_pitch = 48.16f;
        grab_ctrl_cmd->wrist_roll  = -42.14f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 12: // 开始后撤
        grab_ctrl_cmd->base_joint  = 27.24f;
        grab_ctrl_cmd->elbow_roll  = 94.08f;
        grab_ctrl_cmd->elbow_pitch = 103.78f;
        grab_ctrl_cmd->wrist_pitch = 33.26f;
        grab_ctrl_cmd->wrist_roll  = -20.61f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 13:
        grab_ctrl_cmd->base_joint  = 50.97f;
        grab_ctrl_cmd->elbow_roll  = 92.72f;
        grab_ctrl_cmd->elbow_pitch = 91.65f;
        grab_ctrl_cmd->wrist_pitch = 44.99f;
        grab_ctrl_cmd->wrist_roll  = -16.69f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 14:
        grab_ctrl_cmd->base_joint  = 47.72f;
        grab_ctrl_cmd->elbow_roll  = 91.39f;
        grab_ctrl_cmd->elbow_pitch = 77.73f;
        grab_ctrl_cmd->wrist_pitch = 44.64f;
        grab_ctrl_cmd->wrist_roll  = -10.32f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 15: // 安全停稳
        grab_ctrl_cmd->base_joint  = 37.61f;
        grab_ctrl_cmd->elbow_roll  = 65.49f;
        grab_ctrl_cmd->elbow_pitch = 57.18f;
        grab_ctrl_cmd->wrist_pitch = 33.74f;
        grab_ctrl_cmd->wrist_roll  = 7.86f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    default:
        // 后续不再动作，保持最后安全姿态
        break;
    }
}