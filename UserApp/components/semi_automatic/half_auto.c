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
    case 0: // custom_trajectory[0]
        grab_ctrl_cmd->base_joint  = -1.80f;
        grab_ctrl_cmd->elbow_roll  = 1.69f;
        grab_ctrl_cmd->elbow_pitch = -13.93f;
        grab_ctrl_cmd->wrist_pitch = 82.26f;
        grab_ctrl_cmd->wrist_roll  = -5.27f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 1: // custom_trajectory[0]
        grab_ctrl_cmd->base_joint  = -1.80f;
        grab_ctrl_cmd->elbow_roll  = 1.69f;
        grab_ctrl_cmd->elbow_pitch = -13.93f;
        grab_ctrl_cmd->wrist_pitch = 82.26f;
        grab_ctrl_cmd->wrist_roll  = -5.27f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 2: // custom_trajectory[1]
        grab_ctrl_cmd->base_joint  = -3.51f;
        grab_ctrl_cmd->elbow_roll  = 54.19f;
        grab_ctrl_cmd->elbow_pitch = 8.90f;
        grab_ctrl_cmd->wrist_pitch = 80.72f;
        grab_ctrl_cmd->wrist_roll  = -32.25f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 3: // custom_trajectory[2]
        grab_ctrl_cmd->base_joint  = -8.34f;
        grab_ctrl_cmd->elbow_roll  = 63.17f;
        grab_ctrl_cmd->elbow_pitch = 11.39f;
        grab_ctrl_cmd->wrist_pitch = 86.13f;
        grab_ctrl_cmd->wrist_roll  = -55.81f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 4: // custom_trajectory[3]
        grab_ctrl_cmd->base_joint  = -4.87f;
        grab_ctrl_cmd->elbow_roll  = 72.42f;
        grab_ctrl_cmd->elbow_pitch = 22.93f;
        grab_ctrl_cmd->wrist_pitch = 87.45f;
        grab_ctrl_cmd->wrist_roll  = -59.85f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 5: // custom_trajectory[4]
        grab_ctrl_cmd->base_joint  = -4.17f;
        grab_ctrl_cmd->elbow_roll  = 76.02f;
        grab_ctrl_cmd->elbow_pitch = 27.24f;
        grab_ctrl_cmd->wrist_pitch = 71.01f;
        grab_ctrl_cmd->wrist_roll  = -56.02f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 6: // custom_trajectory[5]
        grab_ctrl_cmd->base_joint  = -6.63f;
        grab_ctrl_cmd->elbow_roll  = 79.65f;
        grab_ctrl_cmd->elbow_pitch = 27.94f;
        grab_ctrl_cmd->wrist_pitch = 75.14f;
        grab_ctrl_cmd->wrist_roll  = -53.56f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 7: // custom_trajectory[6]
        grab_ctrl_cmd->base_joint  = 3.42f;
        grab_ctrl_cmd->elbow_roll  = 86.84f;
        grab_ctrl_cmd->elbow_pitch = 47.33f;
        grab_ctrl_cmd->wrist_pitch = 67.36f;
        grab_ctrl_cmd->wrist_roll  = -62.09f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 8: // custom_trajectory[7]
        grab_ctrl_cmd->base_joint  = 6.59f;
        grab_ctrl_cmd->elbow_roll  = 90.08f;
        grab_ctrl_cmd->elbow_pitch = 48.94f;
        grab_ctrl_cmd->wrist_pitch = 71.01f;
        grab_ctrl_cmd->wrist_roll  = -61.74f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 9: // custom_trajectory[7]
        grab_ctrl_cmd->base_joint  = 6.59f;
        grab_ctrl_cmd->elbow_roll  = 90.08f;
        grab_ctrl_cmd->elbow_pitch = 48.94f;
        grab_ctrl_cmd->wrist_pitch = 71.01f;
        grab_ctrl_cmd->wrist_roll  = -61.74f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 10: // custom_trajectory[8]
        grab_ctrl_cmd->base_joint  = 14.41f;
        grab_ctrl_cmd->elbow_roll  = 90.19f;
        grab_ctrl_cmd->elbow_pitch = 51.04f;
        grab_ctrl_cmd->wrist_pitch = 72.06f;
        grab_ctrl_cmd->wrist_roll  = -62.05f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 11: // custom_trajectory[9]
        grab_ctrl_cmd->base_joint  = 18.98f;
        grab_ctrl_cmd->elbow_roll  = 89.90f;
        grab_ctrl_cmd->elbow_pitch = 48.24f;
        grab_ctrl_cmd->wrist_pitch = 68.94f;
        grab_ctrl_cmd->wrist_roll  = -61.17f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 12: // custom_trajectory[10]
        grab_ctrl_cmd->base_joint  = 19.37f;
        grab_ctrl_cmd->elbow_roll  = 87.56f;
        grab_ctrl_cmd->elbow_pitch = 39.13f;
        grab_ctrl_cmd->wrist_pitch = 62.66f;
        grab_ctrl_cmd->wrist_roll  = -60.24f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 13: // custom_trajectory[11]
        grab_ctrl_cmd->base_joint  = 12.34f;
        grab_ctrl_cmd->elbow_roll  = 76.09f;
        grab_ctrl_cmd->elbow_pitch = 26.93f;
        grab_ctrl_cmd->wrist_pitch = 25.70f;
        grab_ctrl_cmd->wrist_roll  = -35.24f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 14: // custom_trajectory[12]
        grab_ctrl_cmd->base_joint  = 2.72f;
        grab_ctrl_cmd->elbow_roll  = 16.33f;
        grab_ctrl_cmd->elbow_pitch = -8.53f;
        grab_ctrl_cmd->wrist_pitch = 9.97f;
        grab_ctrl_cmd->wrist_roll  = -20.25f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    default:
        // 安全停止或保持最后状态
        grab_ctrl_cmd->torque = -0.6f;
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
    case 0:
        grab_ctrl_cmd->base_joint  = -1.58f;
        grab_ctrl_cmd->elbow_roll  = -3.31f;
        grab_ctrl_cmd->elbow_pitch = 12.60f;
        grab_ctrl_cmd->wrist_pitch = 72.42f;
        grab_ctrl_cmd->wrist_roll  = 3.16f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 1:
        grab_ctrl_cmd->base_joint  = -1.58f;
        grab_ctrl_cmd->elbow_roll  = -3.31f;
        grab_ctrl_cmd->elbow_pitch = 12.60f;
        grab_ctrl_cmd->wrist_pitch = 72.42f;
        grab_ctrl_cmd->wrist_roll  = 3.16f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 2:
        grab_ctrl_cmd->base_joint  = -0.17f;
        grab_ctrl_cmd->elbow_roll  = 58.69f;
        grab_ctrl_cmd->elbow_pitch = -0.01f;
        grab_ctrl_cmd->wrist_pitch = 79.01f;
        grab_ctrl_cmd->wrist_roll  = -18.32f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 3:
        grab_ctrl_cmd->base_joint  = 12.87f;
        grab_ctrl_cmd->elbow_roll  = 76.70f;
        grab_ctrl_cmd->elbow_pitch = 7.52f;
        grab_ctrl_cmd->wrist_pitch = 99.40f;
        grab_ctrl_cmd->wrist_roll  = -33.84f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 4:
        grab_ctrl_cmd->base_joint  = 15.29f;
        grab_ctrl_cmd->elbow_roll  = 84.16f;
        grab_ctrl_cmd->elbow_pitch = 28.73f;
        grab_ctrl_cmd->wrist_pitch = 74.09f;
        grab_ctrl_cmd->wrist_roll  = -30.93f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 5:
        grab_ctrl_cmd->base_joint  = 25.48f;
        grab_ctrl_cmd->elbow_roll  = 87.98f;
        grab_ctrl_cmd->elbow_pitch = 58.67f;
        grab_ctrl_cmd->wrist_pitch = 46.88f;
        grab_ctrl_cmd->wrist_roll  = -32.30f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 6:
        grab_ctrl_cmd->base_joint  = 28.34f;
        grab_ctrl_cmd->elbow_roll  = 88.00f;
        grab_ctrl_cmd->elbow_pitch = 67.76f;
        grab_ctrl_cmd->wrist_pitch = 44.82f;
        grab_ctrl_cmd->wrist_roll  = -34.49f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 7:
        grab_ctrl_cmd->base_joint  = 28.74f;
        grab_ctrl_cmd->elbow_roll  = 88.66f;
        grab_ctrl_cmd->elbow_pitch = 74.54f;
        grab_ctrl_cmd->wrist_pitch = 37.96f;
        grab_ctrl_cmd->wrist_roll  = -39.20f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 8:
        grab_ctrl_cmd->base_joint  = 31.64f;
        grab_ctrl_cmd->elbow_roll  = 89.73f;
        grab_ctrl_cmd->elbow_pitch = 79.70f;
        grab_ctrl_cmd->wrist_pitch = 37.88f;
        grab_ctrl_cmd->wrist_roll  = -40.51f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 9:
        grab_ctrl_cmd->base_joint  = 35.06f;
        grab_ctrl_cmd->elbow_roll  = 91.70f;
        grab_ctrl_cmd->elbow_pitch = 84.35f;
        grab_ctrl_cmd->wrist_pitch = 42.89f;
        grab_ctrl_cmd->wrist_roll  = -38.76f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 10:
        grab_ctrl_cmd->base_joint  = 33.66f;
        grab_ctrl_cmd->elbow_roll  = 93.47f;
        grab_ctrl_cmd->elbow_pitch = 84.94f;
        grab_ctrl_cmd->wrist_pitch = 50.40f;
        grab_ctrl_cmd->wrist_roll  = -36.25f;
        grab_ctrl_cmd->torque      = 2.0f;
        break;
    case 11:
        grab_ctrl_cmd->base_joint  = 33.74f;
        grab_ctrl_cmd->elbow_roll  = 94.27f;
        grab_ctrl_cmd->elbow_pitch = 84.44f;
        grab_ctrl_cmd->wrist_pitch = 50.84f;
        grab_ctrl_cmd->wrist_roll  = -38.36f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 12:
        grab_ctrl_cmd->base_joint  = 46.49f;
        grab_ctrl_cmd->elbow_roll  = 94.25f;
        grab_ctrl_cmd->elbow_pitch = 78.17f;
        grab_ctrl_cmd->wrist_pitch = 52.95f;
        grab_ctrl_cmd->wrist_roll  = -37.97f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 13:
        grab_ctrl_cmd->base_joint  = 45.35f;
        grab_ctrl_cmd->elbow_roll  = 94.27f;
        grab_ctrl_cmd->elbow_pitch = 68.70f;
        grab_ctrl_cmd->wrist_pitch = 56.90f;
        grab_ctrl_cmd->wrist_roll  = -37.79f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 14:
        grab_ctrl_cmd->base_joint  = 42.36f;
        grab_ctrl_cmd->elbow_roll  = 92.66f;
        grab_ctrl_cmd->elbow_pitch = 49.77f;
        grab_ctrl_cmd->wrist_pitch = 39.98f;
        grab_ctrl_cmd->wrist_roll  = -33.04f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 15:
        grab_ctrl_cmd->base_joint  = 27.99f;
        grab_ctrl_cmd->elbow_roll  = 61.56f;
        grab_ctrl_cmd->elbow_pitch = 15.44f;
        grab_ctrl_cmd->wrist_pitch = 35.33f;
        grab_ctrl_cmd->wrist_roll  = -26.67f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 16:
        grab_ctrl_cmd->base_joint  = 24.91f;
        grab_ctrl_cmd->elbow_roll  = 46.10f;
        grab_ctrl_cmd->elbow_pitch = -2.30f;
        grab_ctrl_cmd->wrist_pitch = 36.69f;
        grab_ctrl_cmd->wrist_roll  = -26.50f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 17:
        grab_ctrl_cmd->base_joint  = 12.78f;
        grab_ctrl_cmd->elbow_roll  = -5.03f;
        grab_ctrl_cmd->elbow_pitch = -3.61f;
        grab_ctrl_cmd->wrist_pitch = 18.85f;
        grab_ctrl_cmd->wrist_roll  = -25.71f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 18:
        grab_ctrl_cmd->base_joint  = -9.18f;
        grab_ctrl_cmd->elbow_roll  = -15.52f;
        grab_ctrl_cmd->elbow_pitch = -7.28f;
        grab_ctrl_cmd->wrist_pitch = 16.34f;
        grab_ctrl_cmd->wrist_roll  = -20.52f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 19:
        grab_ctrl_cmd->base_joint  = -8.08f;
        grab_ctrl_cmd->elbow_roll  = -15.48f;
        grab_ctrl_cmd->elbow_pitch = 0.57f;
        grab_ctrl_cmd->wrist_pitch = 30.71f;
        grab_ctrl_cmd->wrist_roll  = -2.41f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    case 20:
        grab_ctrl_cmd->base_joint  = -7.16f;
        grab_ctrl_cmd->elbow_roll  = -15.46f;
        grab_ctrl_cmd->elbow_pitch = 1.89f;
        grab_ctrl_cmd->wrist_pitch = 28.03f;
        grab_ctrl_cmd->wrist_roll  = -2.24f;
        grab_ctrl_cmd->torque      = -0.6f;
        break;
    default:
        grab_ctrl_cmd->torque = -0.6f;
        break;
    }
}