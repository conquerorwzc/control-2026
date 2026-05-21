#include "half_auto.h"
#include "trajectory_planner.h"


static uint8_t normal_step = 0; // 专属平地取/存矿的步数 (最大20)
static uint8_t climb_step = 0;  // 专属上台阶重心的步数 (最大13)
static Half_Control_List half_control_list = Store_First_Energy_Unit;

// ========================================================
// S曲线规划器实例 (每个关节独立，由 Half_Auto_Smooth 统一管理)
// ========================================================
static SCurvePlanner_t sc_base;
static SCurvePlanner_t sc_elbow_roll;
static SCurvePlanner_t sc_elbow_pitch;
static SCurvePlanner_t sc_wrist_pitch;
static SCurvePlanner_t sc_wrist_roll;
static SCurvePlanner_t sc_arm_lift;
static SCurvePlanner_t sc_arm_extend;
static uint8_t scurve_inited = 0;

// ========================================================
// S曲线平滑层：初始化 + 统一更新
// ========================================================
// 2. 替换初始化函数：驯服狂暴的 S 曲线参数，并强制读取物理真实角度
static void Half_Auto_Smooth_Init(Grab_Ctrl_Cmd_s *cmd)
{
    // 🌟 护盾 3：开局直接提取物理真实角度，杜绝凭空抽搐
    RobotInstance *robot = RobotGet();
    float init_base   = (robot && robot->grab) ? robot->grab->grab_measure.base_joint   : cmd->base_joint;
    float init_roll   = (robot && robot->grab) ? robot->grab->grab_measure.elbow_roll   : cmd->elbow_roll;
    float init_pitch  = (robot && robot->grab) ? robot->grab->grab_measure.elbow_pitch  : cmd->elbow_pitch;
    float init_wpitch = (robot && robot->grab) ? robot->grab->grab_measure.wrist_pitch  : cmd->wrist_pitch;
    float init_wroll  = (robot && robot->grab) ? robot->grab->grab_measure.wrist_roll   : cmd->wrist_roll;
    float init_lift   = (robot && robot->grab) ? robot->grab->grab_measure.arm_lift     : cmd->arm_lift;
    float init_extend = (robot && robot->grab) ? robot->grab->grab_measure.arm_extend   : cmd->arm_extend;

    // 🌟 护盾 4：全面软化 S 曲线！限制速度与加速度，保护车体
    SCurvePlanner_Reset(&sc_base, init_base);
    sc_base.max_vel = 80.0f; sc_base.max_accel = 200.0f; sc_base.max_jerk = 500.0f;

    SCurvePlanner_Reset(&sc_elbow_roll, init_roll);
    sc_elbow_roll.max_vel = 120.0f; sc_elbow_roll.max_accel = 300.0f; sc_elbow_roll.max_jerk = 1000.0f;

    SCurvePlanner_Reset(&sc_elbow_pitch, init_pitch);
    sc_elbow_pitch.max_vel = 120.0f; sc_elbow_pitch.max_accel = 300.0f; sc_elbow_pitch.max_jerk = 1000.0f;

    SCurvePlanner_Reset(&sc_wrist_pitch, init_wpitch);
    sc_wrist_pitch.max_vel = 150.0f; sc_wrist_pitch.max_accel = 400.0f; sc_wrist_pitch.max_jerk = 2000.0f;

    SCurvePlanner_Reset(&sc_wrist_roll, init_wroll);
    sc_wrist_roll.max_vel = 150.0f; sc_wrist_roll.max_accel = 400.0f; sc_wrist_roll.max_jerk = 2000.0f;

    SCurvePlanner_Reset(&sc_arm_lift, init_lift);
    sc_arm_lift.max_vel = 200.0f; sc_arm_lift.max_accel = 1000.0f; sc_arm_lift.max_jerk = 10000.0f;

    SCurvePlanner_Reset(&sc_arm_extend, init_extend);
    sc_arm_extend.max_vel = 400.0f; sc_arm_extend.max_accel = 2000.0f; sc_arm_extend.max_jerk = 10000.0f;

    scurve_inited = 1;
}

static void Half_Auto_Smooth_Update(Grab_Ctrl_Cmd_s *cmd, float freq)
{
    // 1. 从 cmd 读取模式函数写入的目标值
    sc_base.target        = cmd->base_joint;
    sc_elbow_roll.target  = cmd->elbow_roll;
    sc_elbow_pitch.target = cmd->elbow_pitch;
    sc_wrist_pitch.target = cmd->wrist_pitch;
    sc_wrist_roll.target  = cmd->wrist_roll;
    sc_arm_lift.target    = cmd->arm_lift;
    sc_arm_extend.target  = cmd->arm_extend;

    // 2. 逐关节 S曲线更新 (jerk限幅 → accel限幅 → vel限幅 → pos更新)
    SCurvePlanner_Update(&sc_base, freq);
    SCurvePlanner_Update(&sc_elbow_roll, freq);
    SCurvePlanner_Update(&sc_elbow_pitch, freq);
    SCurvePlanner_Update(&sc_wrist_pitch, freq);
    SCurvePlanner_Update(&sc_wrist_roll, freq);
    SCurvePlanner_Update(&sc_arm_lift, freq);
    SCurvePlanner_Update(&sc_arm_extend, freq);

    // 3. 将平滑后的值写回 cmd (gripper_state 等非浮点量不受影响)
    cmd->base_joint   = sc_base.pos;
    cmd->elbow_roll   = sc_elbow_roll.pos;
    cmd->elbow_pitch  = sc_elbow_pitch.pos;
    cmd->wrist_pitch  = sc_wrist_pitch.pos;
    cmd->wrist_roll   = sc_wrist_roll.pos;
    cmd->arm_lift     = sc_arm_lift.pos;
    cmd->arm_extend   = sc_arm_extend.pos;
}

void Half_auto_reset(Grab_Ctrl_Cmd_s *cmd)
{
    normal_step = 0;
    climb_step = 0;

    if (scurve_inited && cmd != NULL)
    {
        RobotInstance *robot = RobotGet();
        if (robot != NULL && robot->grab != NULL)
        {
            SCurvePlanner_Reset(&sc_base, robot->grab->grab_measure.base_joint);
            SCurvePlanner_Reset(&sc_elbow_roll, robot->grab->grab_measure.elbow_roll);
            SCurvePlanner_Reset(&sc_elbow_pitch, robot->grab->grab_measure.elbow_pitch);
            SCurvePlanner_Reset(&sc_wrist_pitch, robot->grab->grab_measure.wrist_pitch);
            SCurvePlanner_Reset(&sc_wrist_roll, robot->grab->grab_measure.wrist_roll);
            SCurvePlanner_Reset(&sc_arm_lift, robot->grab->grab_measure.arm_lift);
            SCurvePlanner_Reset(&sc_arm_extend, robot->grab->grab_measure.arm_extend);
        }
    }
}

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t press_l,
                      uint8_t press_l_last, uint8_t press_r, uint8_t press_r_last)
{
    // 🌟 护盾 1：使用内部静态变量记忆状态，免疫底层串口丢包导致的连发！
    static uint8_t safe_last_press_l = 0;
    static uint8_t safe_last_press_r = 0;

    uint8_t click_l = (press_l && !safe_last_press_l);
    uint8_t click_r = (press_r && !safe_last_press_r);

    safe_last_press_l = press_l;
    safe_last_press_r = press_r;

    // 首次调用时用当前关节角度初始化S曲线规划器
    if (!scurve_inited) Half_Auto_Smooth_Init(grab_ctrl_cmd);

    if (grab_ctrl_cmd->is_climb_mode)
    {
        if (click_r) climb_step = 0;
        if (click_l && climb_step < 13) climb_step++;

        climb_step_prep(grab_ctrl_cmd, climb_step);
        // 🌟 护盾 2：匹配 RobotCMDTask 的真实 200Hz 运行频率
        Half_Auto_Smooth_Update(grab_ctrl_cmd, 200.0f);
        return;
    }

    climb_step = 0;

    if (click_r)
    {
        half_control_list = (half_control_list + 1) % 4;
        normal_step = 0;
    }

    uint8_t max_step = 0;
    if (half_control_list == Store_First_Energy_Unit) max_step = 23;
    else if (half_control_list == Store_Second_Energy_Unit) max_step = 20;
    else if (half_control_list == Grab_First_Energy_Unit) max_step = 19;
    else if (half_control_list == Grab_Second_Energy_Unit) max_step = 20;

    // 左键推进进度，点满了就死死卡在最后一步
    if (click_l)
    {
        if (normal_step < max_step) normal_step++;
    }

    switch (half_control_list)
    {
    case Store_First_Energy_Unit:   store_first_energy_unit(grab_ctrl_cmd, normal_step); break;
    case Store_Second_Energy_Unit:  store_second_energy_unit(grab_ctrl_cmd, normal_step); break;
    case Grab_First_Energy_Unit:    grab_first_energy_unit(grab_ctrl_cmd, chassis_ctrl_cmd, normal_step); break;
    case Grab_Second_Energy_Unit:   grab_second_energy_unit(grab_ctrl_cmd, chassis_ctrl_cmd, normal_step); break;
    default: break;
    }

    Half_Auto_Smooth_Update(grab_ctrl_cmd, 200.0f);
}
/**
 * @brief 上台阶预备姿态 (一键收起机械臂，防止撞击台阶)
 */
void climb_step_prep(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    grab_ctrl_cmd->arm_lift = 0.0f;

    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = -7.40f;
        grab_ctrl_cmd->elbow_pitch = -36.47f;
        grab_ctrl_cmd->wrist_pitch = 31.73f;
        grab_ctrl_cmd->wrist_roll = -0.18f;
        grab_ctrl_cmd->arm_extend = 224.0f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 83.53f;
        grab_ctrl_cmd->elbow_pitch = -9.85f;
        grab_ctrl_cmd->wrist_pitch = 73.21f;
        grab_ctrl_cmd->wrist_roll = 12.13f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 224.0f;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 89.17f;
        grab_ctrl_cmd->elbow_pitch = 11.25f;
        grab_ctrl_cmd->wrist_pitch = 61.30f;
        grab_ctrl_cmd->wrist_roll = 11.78f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 224.0f;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 83.68f;
        grab_ctrl_cmd->elbow_pitch = 28.99f;
        grab_ctrl_cmd->wrist_pitch = 46.71f;
        grab_ctrl_cmd->wrist_roll = -19.47f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 224.0f;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 83.55f;
        grab_ctrl_cmd->elbow_pitch = 28.77f;
        grab_ctrl_cmd->wrist_pitch = 38.50f;
        grab_ctrl_cmd->wrist_roll = -27.64f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 560.0f; // 🌟 开始伸手
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 84.12f;
        grab_ctrl_cmd->elbow_pitch = 28.77f;
        grab_ctrl_cmd->wrist_pitch = 37.62f;
        grab_ctrl_cmd->wrist_roll = -27.73f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f; // 🌟 伸到极限
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 84.18f;
        grab_ctrl_cmd->elbow_pitch = 28.77f;
        grab_ctrl_cmd->wrist_pitch = 38.32f;
        grab_ctrl_cmd->wrist_roll = -28.12f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 86.06f;
        grab_ctrl_cmd->elbow_pitch = 45.93f;
        grab_ctrl_cmd->wrist_pitch = 51.28f;
        grab_ctrl_cmd->wrist_roll = -26.50f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 76.71f;
        grab_ctrl_cmd->elbow_pitch = 57.91f;
        grab_ctrl_cmd->wrist_pitch = 45.75f;
        grab_ctrl_cmd->wrist_roll = -48.12f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 0.00f;
        grab_ctrl_cmd->elbow_roll = 83.55f;
        grab_ctrl_cmd->elbow_pitch = 69.69f;
        grab_ctrl_cmd->wrist_pitch = 48.78f;
        grab_ctrl_cmd->wrist_roll = -40.03f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = -19.82f; // 🌟 云台开始转向
        grab_ctrl_cmd->elbow_roll = 87.13f;
        grab_ctrl_cmd->elbow_pitch = 51.22f;
        grab_ctrl_cmd->wrist_pitch = -13.23f;
        grab_ctrl_cmd->wrist_roll = -1.49f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 11:
        grab_ctrl_cmd->base_joint = -17.58f;
        grab_ctrl_cmd->elbow_roll = 91.66f;
        grab_ctrl_cmd->elbow_pitch = 49.74f;
        grab_ctrl_cmd->wrist_pitch = 33.57f;
        grab_ctrl_cmd->wrist_roll = -48.65f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 12:
        grab_ctrl_cmd->base_joint = -25.58f;
        grab_ctrl_cmd->elbow_roll = 85.65f;
        grab_ctrl_cmd->elbow_pitch = 51.90f;
        grab_ctrl_cmd->wrist_pitch = 57.74f;
        grab_ctrl_cmd->wrist_roll = -53.70f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = -32.04f;
        grab_ctrl_cmd->elbow_roll = 85.62f;
        grab_ctrl_cmd->elbow_pitch = 44.38f;
        grab_ctrl_cmd->wrist_pitch = 60.64f;
        grab_ctrl_cmd->wrist_roll = -54.45f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        grab_ctrl_cmd->arm_extend = 784.0f;
        break;
    default:
        // 保持最后姿态
        break;
    }
}

// ========================================================
// 模式 0：存第一个能量单元 (共 24 步)
// ========================================================
void store_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = 8.92f;
        grab_ctrl_cmd->elbow_roll = 7.55f;
        grab_ctrl_cmd->elbow_pitch = 15.88f;
        grab_ctrl_cmd->wrist_pitch = 10.22f;
        grab_ctrl_cmd->wrist_roll = 0.40f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 8.96f;
        grab_ctrl_cmd->elbow_roll = 25.50f;
        grab_ctrl_cmd->elbow_pitch = 20.64f;
        grab_ctrl_cmd->wrist_pitch = 14.09f;
        grab_ctrl_cmd->wrist_roll = 1.06f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 9.71f;
        grab_ctrl_cmd->elbow_roll = 48.25f;
        grab_ctrl_cmd->elbow_pitch = 22.48f;
        grab_ctrl_cmd->wrist_pitch = 20.78f;
        grab_ctrl_cmd->wrist_roll = 1.06f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 11.38f;
        grab_ctrl_cmd->elbow_roll = 75.05f;
        grab_ctrl_cmd->elbow_pitch = 31.79f;
        grab_ctrl_cmd->wrist_pitch = 20.82f;
        grab_ctrl_cmd->wrist_roll = -11.57f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 15.02f;
        grab_ctrl_cmd->elbow_roll = 81.80f;
        grab_ctrl_cmd->elbow_pitch = 53.95f;
        grab_ctrl_cmd->wrist_pitch = 41.78f;
        grab_ctrl_cmd->wrist_roll = -11.62f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 14.50f;
        grab_ctrl_cmd->elbow_roll = 87.15f;
        grab_ctrl_cmd->elbow_pitch = 60.80f;
        grab_ctrl_cmd->wrist_pitch = 43.29f;
        grab_ctrl_cmd->wrist_roll = -22.96f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 10.32f;
        grab_ctrl_cmd->elbow_roll = 88.60f;
        grab_ctrl_cmd->elbow_pitch = 64.93f;
        grab_ctrl_cmd->wrist_pitch = 42.83f;
        grab_ctrl_cmd->wrist_roll = -33.63f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 9.35f;
        grab_ctrl_cmd->elbow_roll = 86.04f;
        grab_ctrl_cmd->elbow_pitch = 69.65f;
        grab_ctrl_cmd->wrist_pitch = 42.83f;
        grab_ctrl_cmd->wrist_roll = -50.90f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 6.85f;
        grab_ctrl_cmd->elbow_roll = 87.39f;
        grab_ctrl_cmd->elbow_pitch = 70.52f;
        grab_ctrl_cmd->wrist_pitch = 41.32f;
        grab_ctrl_cmd->wrist_roll = -59.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 8.87f;
        grab_ctrl_cmd->elbow_roll = 93.03f;
        grab_ctrl_cmd->elbow_pitch = 73.10f;
        grab_ctrl_cmd->wrist_pitch = 41.36f;
        grab_ctrl_cmd->wrist_roll = -53.77f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = 9.44f;
        grab_ctrl_cmd->elbow_roll = 96.07f;
        grab_ctrl_cmd->elbow_pitch = 75.53f;
        grab_ctrl_cmd->wrist_pitch = 41.32f;
        grab_ctrl_cmd->wrist_roll = -52.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 11:
        grab_ctrl_cmd->base_joint = 8.78f;
        grab_ctrl_cmd->elbow_roll = 99.18f;
        grab_ctrl_cmd->elbow_pitch = 77.12f;
        grab_ctrl_cmd->wrist_pitch = 41.32f;
        grab_ctrl_cmd->wrist_roll = -41.17f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 12:
        grab_ctrl_cmd->base_joint = 12.65f;
        grab_ctrl_cmd->elbow_roll = 99.18f;
        grab_ctrl_cmd->elbow_pitch = 81.49f;
        grab_ctrl_cmd->wrist_pitch = 41.32f;
        grab_ctrl_cmd->wrist_roll = -40.67f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = 14.28f;
        grab_ctrl_cmd->elbow_roll = 99.98f;
        grab_ctrl_cmd->elbow_pitch = 83.74f;
        grab_ctrl_cmd->wrist_pitch = 39.81f;
        grab_ctrl_cmd->wrist_roll = -39.86f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 14: // 🌟 此处夹爪状态由 1 变 0，开始松开
        grab_ctrl_cmd->base_joint = 11.11f;
        grab_ctrl_cmd->elbow_roll = 99.04f;
        grab_ctrl_cmd->elbow_pitch = 79.05f;
        grab_ctrl_cmd->wrist_pitch = 39.86f;
        grab_ctrl_cmd->wrist_roll = -39.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 15:
        grab_ctrl_cmd->base_joint = 25.53f;
        grab_ctrl_cmd->elbow_roll = 100.27f;
        grab_ctrl_cmd->elbow_pitch = 80.14f;
        grab_ctrl_cmd->wrist_pitch = 39.90f;
        grab_ctrl_cmd->wrist_roll = -39.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 16:
        grab_ctrl_cmd->base_joint = 32.78f;
        grab_ctrl_cmd->elbow_roll = 94.91f;
        grab_ctrl_cmd->elbow_pitch = 79.48f;
        grab_ctrl_cmd->wrist_pitch = 39.86f;
        grab_ctrl_cmd->wrist_roll = -36.98f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 17:
        grab_ctrl_cmd->base_joint = 36.20f;
        grab_ctrl_cmd->elbow_roll = 75.72f;
        grab_ctrl_cmd->elbow_pitch = 78.37f;
        grab_ctrl_cmd->wrist_pitch = 39.86f;
        grab_ctrl_cmd->wrist_roll = -36.46f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 18:
        grab_ctrl_cmd->base_joint = 29.66f;
        grab_ctrl_cmd->elbow_roll = 58.70f;
        grab_ctrl_cmd->elbow_pitch = 69.21f;
        grab_ctrl_cmd->wrist_pitch = 39.83f;
        grab_ctrl_cmd->wrist_roll = -30.26f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 19:
        grab_ctrl_cmd->base_joint = 23.59f;
        grab_ctrl_cmd->elbow_roll = 46.09f;
        grab_ctrl_cmd->elbow_pitch = 61.63f;
        grab_ctrl_cmd->wrist_pitch = 39.83f;
        grab_ctrl_cmd->wrist_roll = -25.21f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 20:
        grab_ctrl_cmd->base_joint = 18.50f;
        grab_ctrl_cmd->elbow_roll = 31.35f;
        grab_ctrl_cmd->elbow_pitch = 60.36f;
        grab_ctrl_cmd->wrist_pitch = 39.88f;
        grab_ctrl_cmd->wrist_roll = -15.65f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 21:
        grab_ctrl_cmd->base_joint = 7.38f;
        grab_ctrl_cmd->elbow_roll = 18.44f;
        grab_ctrl_cmd->elbow_pitch = 31.42f;
        grab_ctrl_cmd->wrist_pitch = 39.90f;
        grab_ctrl_cmd->wrist_roll = -7.91f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 22:
        grab_ctrl_cmd->base_joint = 0.52f;
        grab_ctrl_cmd->elbow_roll = 13.80f;
        grab_ctrl_cmd->elbow_pitch = 17.26f;
        grab_ctrl_cmd->wrist_pitch = 39.90f;
        grab_ctrl_cmd->wrist_roll = -5.73f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 23:
        grab_ctrl_cmd->base_joint = -0.75f;
        grab_ctrl_cmd->elbow_roll = 5.41f;
        grab_ctrl_cmd->elbow_pitch = -6.13f;
        grab_ctrl_cmd->wrist_pitch = 39.83f;
        grab_ctrl_cmd->wrist_roll = -5.55f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    default:
        // 结束时保持松开并停止
        break;
    }
}

// ========================================================
// 模式 1：存第二个能量单元 (共 21 步)
// ========================================================
void store_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, uint8_t step)
{
    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = -4.36f;
        grab_ctrl_cmd->elbow_roll = 5.04f;
        grab_ctrl_cmd->elbow_pitch = 18.02f;
        grab_ctrl_cmd->wrist_pitch = 18.33f;
        grab_ctrl_cmd->wrist_roll = 7.88f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 0.48f;
        grab_ctrl_cmd->elbow_roll = 44.75f;
        grab_ctrl_cmd->elbow_pitch = 16.47f;
        grab_ctrl_cmd->wrist_pitch = 36.01f;
        grab_ctrl_cmd->wrist_roll = 7.03f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 13.84f;
        grab_ctrl_cmd->elbow_roll = 74.65f;
        grab_ctrl_cmd->elbow_pitch = 24.21f;
        grab_ctrl_cmd->wrist_pitch = 36.03f;
        grab_ctrl_cmd->wrist_roll = 2.11f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 27.94f;
        grab_ctrl_cmd->elbow_roll = 77.91f;
        grab_ctrl_cmd->elbow_pitch = 48.21f;
        grab_ctrl_cmd->wrist_pitch = 50.76f;
        grab_ctrl_cmd->wrist_roll = -8.62f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 34.36f;
        grab_ctrl_cmd->elbow_roll = 83.77f;
        grab_ctrl_cmd->elbow_pitch = 63.26f;
        grab_ctrl_cmd->wrist_pitch = 50.76f;
        grab_ctrl_cmd->wrist_roll = -17.19f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 34.93f;
        grab_ctrl_cmd->elbow_roll = 83.79f;
        grab_ctrl_cmd->elbow_pitch = 76.55f;
        grab_ctrl_cmd->wrist_pitch = 42.83f;
        grab_ctrl_cmd->wrist_roll = -24.71f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 35.15f;
        grab_ctrl_cmd->elbow_roll = 86.17f;
        grab_ctrl_cmd->elbow_pitch = 85.86f;
        grab_ctrl_cmd->wrist_pitch = 35.27f;
        grab_ctrl_cmd->wrist_roll = -32.16f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 35.63f;
        grab_ctrl_cmd->elbow_roll = 86.78f;
        grab_ctrl_cmd->elbow_pitch = 90.45f;
        grab_ctrl_cmd->wrist_pitch = 30.76f;
        grab_ctrl_cmd->wrist_roll = -38.44f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 35.50f;
        grab_ctrl_cmd->elbow_roll = 89.06f;
        grab_ctrl_cmd->elbow_pitch = 94.76f;
        grab_ctrl_cmd->wrist_pitch = 30.79f;
        grab_ctrl_cmd->wrist_roll = -41.63f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 30.32f;
        grab_ctrl_cmd->elbow_roll = 93.76f;
        grab_ctrl_cmd->elbow_pitch = 102.28f;
        grab_ctrl_cmd->wrist_pitch = 31.27f;
        grab_ctrl_cmd->wrist_roll = -43.83f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = 30.67f;
        grab_ctrl_cmd->elbow_roll = 97.19f;
        grab_ctrl_cmd->elbow_pitch = 103.00f;
        grab_ctrl_cmd->wrist_pitch = 41.39f;
        grab_ctrl_cmd->wrist_roll = -50.13f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 11:
        grab_ctrl_cmd->base_joint = 37.08f;
        grab_ctrl_cmd->elbow_roll = 101.73f;
        grab_ctrl_cmd->elbow_pitch = 103.09f;
        grab_ctrl_cmd->wrist_pitch = 49.36f;
        grab_ctrl_cmd->wrist_roll = -46.94f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 12: // 🌟 此处为放矿点，夹爪开始松开
        grab_ctrl_cmd->base_joint = 35.02f;
        grab_ctrl_cmd->elbow_roll = 101.10f;
        grab_ctrl_cmd->elbow_pitch = 99.35f;
        grab_ctrl_cmd->wrist_pitch = 49.32f;
        grab_ctrl_cmd->wrist_roll = -47.59f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = 42.53f;
        grab_ctrl_cmd->elbow_roll = 104.31f;
        grab_ctrl_cmd->elbow_pitch = 97.67f;
        grab_ctrl_cmd->wrist_pitch = 52.36f;
        grab_ctrl_cmd->wrist_roll = -47.35f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 14:
        grab_ctrl_cmd->base_joint = 46.09f;
        grab_ctrl_cmd->elbow_roll = 103.44f;
        grab_ctrl_cmd->elbow_pitch = 91.68f;
        grab_ctrl_cmd->wrist_pitch = 52.34f;
        grab_ctrl_cmd->wrist_roll = -47.33f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 15:
        grab_ctrl_cmd->base_joint = 49.21f;
        grab_ctrl_cmd->elbow_roll = 100.62f;
        grab_ctrl_cmd->elbow_pitch = 78.56f;
        grab_ctrl_cmd->wrist_pitch = 52.36f;
        grab_ctrl_cmd->wrist_roll = -46.89f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 16:
        grab_ctrl_cmd->base_joint = 48.20f;
        grab_ctrl_cmd->elbow_roll = 79.33f;
        grab_ctrl_cmd->elbow_pitch = 70.67f;
        grab_ctrl_cmd->wrist_pitch = 52.27f;
        grab_ctrl_cmd->wrist_roll = -34.72f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 17:
        grab_ctrl_cmd->base_joint = 41.57f;
        grab_ctrl_cmd->elbow_roll = 81.06f;
        grab_ctrl_cmd->elbow_pitch = 43.24f;
        grab_ctrl_cmd->wrist_pitch = 35.73f;
        grab_ctrl_cmd->wrist_roll = -34.68f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 18:
        grab_ctrl_cmd->base_joint = 26.45f;
        grab_ctrl_cmd->elbow_roll = 33.93f;
        grab_ctrl_cmd->elbow_pitch = 19.51f;
        grab_ctrl_cmd->wrist_pitch = 32.32f;
        grab_ctrl_cmd->wrist_roll = -11.49f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 19:
        grab_ctrl_cmd->base_joint = 18.36f;
        grab_ctrl_cmd->elbow_roll = 14.33f;
        grab_ctrl_cmd->elbow_pitch = 7.03f;
        grab_ctrl_cmd->wrist_pitch = 46.50f;
        grab_ctrl_cmd->wrist_roll = -8.34f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 20:
        grab_ctrl_cmd->base_joint = -2.73f;
        grab_ctrl_cmd->elbow_roll = -0.58f;
        grab_ctrl_cmd->elbow_pitch = -0.03f;
        grab_ctrl_cmd->wrist_pitch = 39.38f;
        grab_ctrl_cmd->wrist_roll = 2.57f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    default:
        break;
    }
}

// ========================================================
// 模式 2：取第一个能量单元 (共 20 步)
// ========================================================
void grab_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step)
{
    // 取矿动作，底盘高度锁定为 0%
    chassis_ctrl_cmd->lift_ratio = 0.0f;

    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = -0.31f;
        grab_ctrl_cmd->elbow_roll = 10.02f;
        grab_ctrl_cmd->elbow_pitch = 9.19f;
        grab_ctrl_cmd->wrist_pitch = 8.78f;
        grab_ctrl_cmd->wrist_roll = 7.27f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 0.48f;
        grab_ctrl_cmd->elbow_roll = 41.65f;
        grab_ctrl_cmd->elbow_pitch = 6.59f;
        grab_ctrl_cmd->wrist_pitch = 8.97f;
        grab_ctrl_cmd->wrist_roll = 5.58f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 23.90f;
        grab_ctrl_cmd->elbow_roll = 66.72f;
        grab_ctrl_cmd->elbow_pitch = 43.11f;
        grab_ctrl_cmd->wrist_pitch = 25.69f;
        grab_ctrl_cmd->wrist_roll = 0.43f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 29.22f;
        grab_ctrl_cmd->elbow_roll = 78.15f;
        grab_ctrl_cmd->elbow_pitch = 62.13f;
        grab_ctrl_cmd->wrist_pitch = 28.69f;
        grab_ctrl_cmd->wrist_roll = -2.20f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 29.96f;
        grab_ctrl_cmd->elbow_roll = 86.37f;
        grab_ctrl_cmd->elbow_pitch = 75.22f;
        grab_ctrl_cmd->wrist_pitch = 28.69f;
        grab_ctrl_cmd->wrist_roll = -5.21f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 28.51f;
        grab_ctrl_cmd->elbow_roll = 91.59f;
        grab_ctrl_cmd->elbow_pitch = 82.48f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = -16.97f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 26.93f;
        grab_ctrl_cmd->elbow_roll = 97.12f;
        grab_ctrl_cmd->elbow_pitch = 88.84f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = -28.12f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 28.78f;
        grab_ctrl_cmd->elbow_roll = 99.15f;
        grab_ctrl_cmd->elbow_pitch = 94.24f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -32.21f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 16.08f;
        grab_ctrl_cmd->elbow_roll = 99.13f;
        grab_ctrl_cmd->elbow_pitch = 88.58f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -34.98f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 6.54f;
        grab_ctrl_cmd->elbow_roll = 100.25f;
        grab_ctrl_cmd->elbow_pitch = 85.14f;
        grab_ctrl_cmd->wrist_pitch = 27.83f;
        grab_ctrl_cmd->wrist_roll = -34.94f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = 5.79f;
        grab_ctrl_cmd->elbow_roll = 100.06f;
        grab_ctrl_cmd->elbow_pitch = 81.36f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -34.96f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 11: // 🌟 此处夹爪状态由 0 变 1，执行抓取
        grab_ctrl_cmd->base_joint = 5.84f;
        grab_ctrl_cmd->elbow_roll = 100.03f;
        grab_ctrl_cmd->elbow_pitch = 81.38f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = -34.94f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 12:
        grab_ctrl_cmd->base_joint = 8.92f;
        grab_ctrl_cmd->elbow_roll = 93.60f;
        grab_ctrl_cmd->elbow_pitch = 73.01f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -34.94f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = 9.05f;
        grab_ctrl_cmd->elbow_roll = 85.54f;
        grab_ctrl_cmd->elbow_pitch = 68.05f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -38.39f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 14:
        grab_ctrl_cmd->base_joint = 9.88f;
        grab_ctrl_cmd->elbow_roll = 72.77f;
        grab_ctrl_cmd->elbow_pitch = 63.77f;
        grab_ctrl_cmd->wrist_pitch = 27.86f;
        grab_ctrl_cmd->wrist_roll = -32.75f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 15:
        grab_ctrl_cmd->base_joint = 10.67f;
        grab_ctrl_cmd->elbow_roll = 58.24f;
        grab_ctrl_cmd->elbow_pitch = 46.81f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = -22.44f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 16:
        grab_ctrl_cmd->base_joint = 7.64f;
        grab_ctrl_cmd->elbow_roll = 49.28f;
        grab_ctrl_cmd->elbow_pitch = 34.89f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = -1.17f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 17:
        grab_ctrl_cmd->base_joint = 1.36f;
        grab_ctrl_cmd->elbow_roll = 36.69f;
        grab_ctrl_cmd->elbow_pitch = 21.69f;
        grab_ctrl_cmd->wrist_pitch = 27.88f;
        grab_ctrl_cmd->wrist_roll = 4.56f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 18:
        grab_ctrl_cmd->base_joint = -2.25f;
        grab_ctrl_cmd->elbow_roll = 13.52f;
        grab_ctrl_cmd->elbow_pitch = 1.45f;
        grab_ctrl_cmd->wrist_pitch = 31.33f;
        grab_ctrl_cmd->wrist_roll = 10.72f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 19:
        grab_ctrl_cmd->base_joint = -2.77f;
        grab_ctrl_cmd->elbow_roll = 13.52f;
        grab_ctrl_cmd->elbow_pitch = 1.43f;
        grab_ctrl_cmd->wrist_pitch = 31.55f;
        grab_ctrl_cmd->wrist_roll = 10.72f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    default:
        break;
    }
}
// ========================================================
// 模式 3：取第二个能量单元 (共 21 步)
// ========================================================
void grab_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step)
{
    // 取矿动作，底盘高度锁定为 0%
    chassis_ctrl_cmd->lift_ratio = 0.0f;

    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = 1.84f;
        grab_ctrl_cmd->elbow_roll = 6.31f;
        grab_ctrl_cmd->elbow_pitch = 5.93f;
        grab_ctrl_cmd->wrist_pitch = 8.95f;
        grab_ctrl_cmd->wrist_roll = 6.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 9.84f;
        grab_ctrl_cmd->elbow_roll = 45.12f;
        grab_ctrl_cmd->elbow_pitch = 10.35f;
        grab_ctrl_cmd->wrist_pitch = 28.29f;
        grab_ctrl_cmd->wrist_roll = 6.00f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 17.79f;
        grab_ctrl_cmd->elbow_roll = 50.74f;
        grab_ctrl_cmd->elbow_pitch = 18.22f;
        grab_ctrl_cmd->wrist_pitch = 50.59f;
        grab_ctrl_cmd->wrist_roll = 1.89f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 29.88f;
        grab_ctrl_cmd->elbow_roll = 73.25f;
        grab_ctrl_cmd->elbow_pitch = 46.54f;
        grab_ctrl_cmd->wrist_pitch = 66.02f;
        grab_ctrl_cmd->wrist_roll = -6.68f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 38.31f;
        grab_ctrl_cmd->elbow_roll = 83.79f;
        grab_ctrl_cmd->elbow_pitch = 68.10f;
        grab_ctrl_cmd->wrist_pitch = 65.76f;
        grab_ctrl_cmd->wrist_roll = -13.13f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 44.69f;
        grab_ctrl_cmd->elbow_roll = 95.22f;
        grab_ctrl_cmd->elbow_pitch = 82.59f;
        grab_ctrl_cmd->wrist_pitch = 56.27f;
        grab_ctrl_cmd->wrist_roll = -27.31f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 45.52f;
        grab_ctrl_cmd->elbow_roll = 99.18f;
        grab_ctrl_cmd->elbow_pitch = 91.68f;
        grab_ctrl_cmd->wrist_pitch = 50.24f;
        grab_ctrl_cmd->wrist_roll = -32.99f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 42.40f;
        grab_ctrl_cmd->elbow_roll = 102.63f;
        grab_ctrl_cmd->elbow_pitch = 94.72f;
        grab_ctrl_cmd->wrist_pitch = 48.77f;
        grab_ctrl_cmd->wrist_roll = -41.28f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 37.66f;
        grab_ctrl_cmd->elbow_roll = 102.85f;
        grab_ctrl_cmd->elbow_pitch = 104.03f;
        grab_ctrl_cmd->wrist_pitch = 40.93f;
        grab_ctrl_cmd->wrist_roll = -39.09f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 31.24f;
        grab_ctrl_cmd->elbow_roll = 102.80f;
        grab_ctrl_cmd->elbow_pitch = 101.75f;
        grab_ctrl_cmd->wrist_pitch = 41.23f;
        grab_ctrl_cmd->wrist_roll = -37.93f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = 28.08f;
        grab_ctrl_cmd->elbow_roll = 103.24f;
        grab_ctrl_cmd->elbow_pitch = 103.13f;
        grab_ctrl_cmd->wrist_pitch = 34.02f;
        grab_ctrl_cmd->wrist_roll = -37.93f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 11: // 🌟 此处夹爪状态由 0 变 1，执行抓取
        grab_ctrl_cmd->base_joint = 28.51f;
        grab_ctrl_cmd->elbow_roll = 102.59f;
        grab_ctrl_cmd->elbow_pitch = 100.42f;
        grab_ctrl_cmd->wrist_pitch = 33.98f;
        grab_ctrl_cmd->wrist_roll = -37.95f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 12:
        grab_ctrl_cmd->base_joint = 27.28f;
        grab_ctrl_cmd->elbow_roll = 97.32f;
        grab_ctrl_cmd->elbow_pitch = 88.40f;
        grab_ctrl_cmd->wrist_pitch = 33.80f;
        grab_ctrl_cmd->wrist_roll = -38.61f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = 25.57f;
        grab_ctrl_cmd->elbow_roll = 90.59f;
        grab_ctrl_cmd->elbow_pitch = 84.25f;
        grab_ctrl_cmd->wrist_pitch = 33.82f;
        grab_ctrl_cmd->wrist_roll = -39.88f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 14:
        grab_ctrl_cmd->base_joint = 28.60f;
        grab_ctrl_cmd->elbow_roll = 79.29f;
        grab_ctrl_cmd->elbow_pitch = 76.58f;
        grab_ctrl_cmd->wrist_pitch = 33.82f;
        grab_ctrl_cmd->wrist_roll = -39.88f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 15:
        grab_ctrl_cmd->base_joint = 18.28f;
        grab_ctrl_cmd->elbow_roll = 58.19f;
        grab_ctrl_cmd->elbow_pitch = 65.23f;
        grab_ctrl_cmd->wrist_pitch = 33.80f;
        grab_ctrl_cmd->wrist_roll = -22.90f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 16:
        grab_ctrl_cmd->base_joint = 12.87f;
        grab_ctrl_cmd->elbow_roll = 35.40f;
        grab_ctrl_cmd->elbow_pitch = 55.46f;
        grab_ctrl_cmd->wrist_pitch = 33.80f;
        grab_ctrl_cmd->wrist_roll = -19.88f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 17:
        grab_ctrl_cmd->base_joint = 7.46f;
        grab_ctrl_cmd->elbow_roll = 18.98f;
        grab_ctrl_cmd->elbow_pitch = 40.93f;
        grab_ctrl_cmd->wrist_pitch = 33.82f;
        grab_ctrl_cmd->wrist_roll = -19.68f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 18:
        grab_ctrl_cmd->base_joint = 2.76f;
        grab_ctrl_cmd->elbow_roll = 8.38f;
        grab_ctrl_cmd->elbow_pitch = 27.92f;
        grab_ctrl_cmd->wrist_pitch = 33.85f;
        grab_ctrl_cmd->wrist_roll = -15.75f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 19:
        grab_ctrl_cmd->base_joint = -1.68f;
        grab_ctrl_cmd->elbow_roll = 5.34f;
        grab_ctrl_cmd->elbow_pitch = 4.82f;
        grab_ctrl_cmd->wrist_pitch = 34.46f;
        grab_ctrl_cmd->wrist_roll = -4.67f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 20:
        grab_ctrl_cmd->base_joint = -0.75f;
        grab_ctrl_cmd->elbow_roll = 5.34f;
        grab_ctrl_cmd->elbow_pitch = 7.73f;
        grab_ctrl_cmd->wrist_pitch = 18.31f;
        grab_ctrl_cmd->wrist_roll = 4.71f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    default:
        // 结束时保持最后抓紧的状态
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    }
}