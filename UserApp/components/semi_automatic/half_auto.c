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
static void Half_Auto_Smooth_Init(Grab_Ctrl_Cmd_s *cmd)
{
    // 角度类关节 (度/秒, 度/秒², 度/秒³)
    SCurvePlanner_Reset(&sc_base, cmd->base_joint);
    sc_base.max_vel = 150.0f; sc_base.max_accel = 1500.0f; sc_base.max_jerk = 8000.0f;

    SCurvePlanner_Reset(&sc_elbow_roll, cmd->elbow_roll);
    sc_elbow_roll.max_vel = 300.0f; sc_elbow_roll.max_accel = 1500.0f; sc_elbow_roll.max_jerk = 15000.0f;

    SCurvePlanner_Reset(&sc_elbow_pitch, cmd->elbow_pitch);
    sc_elbow_pitch.max_vel = 300.0f; sc_elbow_pitch.max_accel = 1500.0f; sc_elbow_pitch.max_jerk = 15000.0f;

    SCurvePlanner_Reset(&sc_wrist_pitch, cmd->wrist_pitch);
    sc_wrist_pitch.max_vel = 300.0f; sc_wrist_pitch.max_accel = 1500.0f; sc_wrist_pitch.max_jerk = 15000.0f;

    SCurvePlanner_Reset(&sc_wrist_roll, cmd->wrist_roll);
    sc_wrist_roll.max_vel = 300.0f; sc_wrist_roll.max_accel = 1500.0f; sc_wrist_roll.max_jerk = 15000.0f;

    // 线性执行器 (mm/秒, mm/秒², mm/秒³)
    SCurvePlanner_Reset(&sc_arm_lift, cmd->arm_lift);
    sc_arm_lift.max_vel = 400.0f; sc_arm_lift.max_accel = 2000.0f; sc_arm_lift.max_jerk = 20000.0f;

    SCurvePlanner_Reset(&sc_arm_extend, cmd->arm_extend);
    sc_arm_extend.max_vel = 800.0f; sc_arm_extend.max_accel = 5000.0f; sc_arm_extend.max_jerk = 10000.0f;

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

void Half_auto_reset(void)
{
    normal_step = 0;
    climb_step = 0;
}

void Half_auto_update(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t press_l,
                      uint8_t press_l_last, uint8_t press_r, uint8_t press_r_last)
{
    // 首次调用时用当前关节角度初始化S曲线规划器
    if (!scurve_inited) Half_Auto_Smooth_Init(grab_ctrl_cmd);

    // ========================================================
    // 1. 上台阶专属控制域 (拦截所有常规操作)
    // ========================================================
    if (grab_ctrl_cmd->is_climb_mode)
    {
        // 🌟 防呆设计：上台阶时如果不小心多按了左键，直接按【右键】一键重置爬楼进度！
        if (press_r && !press_r_last)
        {
            climb_step = 0;
        }

        if (press_l && !press_l_last)
        {
            if (climb_step < 13) climb_step++; // 左键只增加 climb_step，上限13
        }

        climb_step_prep(grab_ctrl_cmd, climb_step);
        Half_Auto_Smooth_Update(grab_ctrl_cmd, 500.0f);
        return;
    }

    // ========================================================
    // 2. 常规取/存矿控制域 (退出上台阶后恢复)
    // ========================================================
    climb_step = 0; // 只要退出了爬楼模式，爬楼进度立刻清零，随时准备下次爬楼

    // 右键切路线，并清零取矿步数
    if (press_r && !press_r_last)
    {
        half_control_list = (half_control_list + 1) % 4;
        normal_step = 0;
    }

    // 左键推进取矿进度
    if (press_l && !press_l_last)
    {
        if (normal_step < 30) normal_step++; // 上限更新为27
    }

    // 执行对应的常规半自动
    switch (half_control_list)
    {
    case Store_First_Energy_Unit:
        store_first_energy_unit(grab_ctrl_cmd, normal_step);
        break;
    case Store_Second_Energy_Unit:
        store_second_energy_unit(grab_ctrl_cmd, normal_step);
        break;
    case Grab_First_Energy_Unit:
        grab_first_energy_unit(grab_ctrl_cmd, chassis_ctrl_cmd, normal_step);
        break;
    case Grab_Second_Energy_Unit:
        grab_second_energy_unit(grab_ctrl_cmd, chassis_ctrl_cmd, normal_step);
        break;
    default:
        break;
    }

    // S曲线平滑层：统一输出 (gripper_state 不受影响)
    Half_Auto_Smooth_Update(grab_ctrl_cmd, 500.0f);
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
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
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
        // 结束时保持松开并停止
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    }
}

// ========================================================
// 模式 2：夹六点钟的能量单元 (共 18 步，底盘高度 0%)
// ========================================================
void grab_first_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step)
{
    // 整个六点钟夹取动作中，底盘高度锁定为 0% (完全收回状态)
    chassis_ctrl_cmd->lift_ratio = 0.0f;

    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = 9.62f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 4.68f;
        grab_ctrl_cmd->wrist_pitch = 45.48f;
        grab_ctrl_cmd->wrist_roll = -2.10f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 42.93f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = -2.30f;
        grab_ctrl_cmd->wrist_pitch = 46.53f;
        grab_ctrl_cmd->wrist_roll = -2.59f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 53.08f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 56.27f;
        grab_ctrl_cmd->wrist_pitch = -43.76f;
        grab_ctrl_cmd->wrist_roll = -2.46f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 69.34f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 88.37f;
        grab_ctrl_cmd->wrist_pitch = -88.15f;
        grab_ctrl_cmd->wrist_roll = -2.19f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 84.63f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 81.31f;
        grab_ctrl_cmd->wrist_pitch = -82.00f;
        grab_ctrl_cmd->wrist_roll = -2.06f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 5:
        grab_ctrl_cmd->base_joint = 84.06f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 61.14f;
        grab_ctrl_cmd->wrist_pitch = -54.97f;
        grab_ctrl_cmd->wrist_roll = -2.10f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 6:
        grab_ctrl_cmd->base_joint = 77.29f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 57.14f;
        grab_ctrl_cmd->wrist_pitch = -51.63f;
        grab_ctrl_cmd->wrist_roll = -2.02f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 82.00f;
        grab_ctrl_cmd->elbow_roll = -4.79f;
        grab_ctrl_cmd->elbow_pitch = 49.32f;
        grab_ctrl_cmd->wrist_pitch = -40.73f;
        grab_ctrl_cmd->wrist_roll = -2.28f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 8:
        grab_ctrl_cmd->base_joint = 80.55f;
        grab_ctrl_cmd->elbow_roll = -4.77f;
        grab_ctrl_cmd->elbow_pitch = 54.21f;
        grab_ctrl_cmd->wrist_pitch = -45.83f;
        grab_ctrl_cmd->wrist_roll = -2.24f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 79.32f;
        grab_ctrl_cmd->elbow_roll = -4.77f;
        grab_ctrl_cmd->elbow_pitch = 51.22f;
        grab_ctrl_cmd->wrist_pitch = -44.03f;
        grab_ctrl_cmd->wrist_roll = -2.15f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = 78.92f;
        grab_ctrl_cmd->elbow_roll = -4.75f;
        grab_ctrl_cmd->elbow_pitch = 48.03f;
        grab_ctrl_cmd->wrist_pitch = -49.83f;
        grab_ctrl_cmd->wrist_roll = -2.46f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 11:
        grab_ctrl_cmd->base_joint = 77.65f;
        grab_ctrl_cmd->elbow_roll = -4.77f;
        grab_ctrl_cmd->elbow_pitch = 48.22f;
        grab_ctrl_cmd->wrist_pitch = -65.08f;
        grab_ctrl_cmd->wrist_roll = -2.19f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 12:
        grab_ctrl_cmd->base_joint = 71.45f;
        grab_ctrl_cmd->elbow_roll = -4.77f;
        grab_ctrl_cmd->elbow_pitch = 48.22f;
        grab_ctrl_cmd->wrist_pitch = -66.92f;
        grab_ctrl_cmd->wrist_roll = -2.15f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 13:
        grab_ctrl_cmd->base_joint = 45.70f;
        grab_ctrl_cmd->elbow_roll = -2.85f;
        grab_ctrl_cmd->elbow_pitch = 41.47f;
        grab_ctrl_cmd->wrist_pitch = -54.09f;
        grab_ctrl_cmd->wrist_roll = -2.06f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 14:
        grab_ctrl_cmd->base_joint = 22.19f;
        grab_ctrl_cmd->elbow_roll = -2.85f;
        grab_ctrl_cmd->elbow_pitch = 37.60f;
        grab_ctrl_cmd->wrist_pitch = -45.39f;
        grab_ctrl_cmd->wrist_roll = -2.32f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 15:
        grab_ctrl_cmd->base_joint = 6.37f;
        grab_ctrl_cmd->elbow_roll = -3.52f;
        grab_ctrl_cmd->elbow_pitch = 23.09f;
        grab_ctrl_cmd->wrist_pitch = 0.57f;
        grab_ctrl_cmd->wrist_roll = -2.02f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 16: 
        grab_ctrl_cmd->base_joint = -1.01f;
        grab_ctrl_cmd->elbow_roll = -3.48f;
        grab_ctrl_cmd->elbow_pitch = 3.55f;
        grab_ctrl_cmd->wrist_pitch = 77.47f;
        grab_ctrl_cmd->wrist_roll = -2.24f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 17: 
        grab_ctrl_cmd->base_joint = -1.53f;
        grab_ctrl_cmd->elbow_roll = -3.48f;
        grab_ctrl_cmd->elbow_pitch = 3.55f;
        grab_ctrl_cmd->wrist_pitch = 77.65f;
        grab_ctrl_cmd->wrist_roll = -2.24f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    default:
        // 结束时保持最后抓紧的状态
        break;
    }
}

// ========================================================
// 模式 3：取四点钟方向的能量单元 (共 11 步，底盘高度 0%)
// ========================================================
void grab_second_energy_unit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd, uint8_t step)
{
    // 锁定底盘高度为 0%
    chassis_ctrl_cmd->lift_ratio = 0.0f;

    switch (step)
    {
    case 0:
        grab_ctrl_cmd->base_joint = 2.46f;
        grab_ctrl_cmd->elbow_roll = -4.36f;
        grab_ctrl_cmd->elbow_pitch = 3.79f;
        grab_ctrl_cmd->wrist_pitch = 38.36f;
        grab_ctrl_cmd->wrist_roll = -0.79f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 1:
        grab_ctrl_cmd->base_joint = 47.94f;
        grab_ctrl_cmd->elbow_roll = -4.38f;
        grab_ctrl_cmd->elbow_pitch = -3.87f;
        grab_ctrl_cmd->wrist_pitch = 40.12f;
        grab_ctrl_cmd->wrist_roll = 1.97f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 2:
        grab_ctrl_cmd->base_joint = 56.24f;
        grab_ctrl_cmd->elbow_roll = -24.53f;
        grab_ctrl_cmd->elbow_pitch = 31.72f;
        grab_ctrl_cmd->wrist_pitch = 1.58f;
        grab_ctrl_cmd->wrist_roll = 11.33f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 3:
        grab_ctrl_cmd->base_joint = 61.17f;
        grab_ctrl_cmd->elbow_roll = -29.84f;
        grab_ctrl_cmd->elbow_pitch = 63.15f;
        grab_ctrl_cmd->wrist_pitch = -30.71f;
        grab_ctrl_cmd->wrist_roll = 5.75f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 4:
        grab_ctrl_cmd->base_joint = 68.24f;
        grab_ctrl_cmd->elbow_roll = -9.89f;
        grab_ctrl_cmd->elbow_pitch = 59.37f;
        grab_ctrl_cmd->wrist_pitch = -34.62f;
        grab_ctrl_cmd->wrist_roll = 28.47f;
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        break;
    case 5: // 🚩 夹取点：由开合改变状态
        grab_ctrl_cmd->base_joint = 70.97f;
        grab_ctrl_cmd->elbow_roll = -9.89f;
        grab_ctrl_cmd->elbow_pitch = 59.37f;
        grab_ctrl_cmd->wrist_pitch = -34.98f;
        grab_ctrl_cmd->wrist_roll = 28.52f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 6: //
        grab_ctrl_cmd->base_joint = 59.23f;
        grab_ctrl_cmd->elbow_roll = -13.95f;
        grab_ctrl_cmd->elbow_pitch = 54.23f;
        grab_ctrl_cmd->wrist_pitch = -43.90f;
        grab_ctrl_cmd->wrist_roll = 31.55f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 7:
        grab_ctrl_cmd->base_joint = 55.41f;
        grab_ctrl_cmd->elbow_roll = -13.93f;
        grab_ctrl_cmd->elbow_pitch = 54.67f;
        grab_ctrl_cmd->wrist_pitch = -43.06f;
        grab_ctrl_cmd->wrist_roll = 28.47f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 8: //
        grab_ctrl_cmd->base_joint = 37.44f;
        grab_ctrl_cmd->elbow_roll = -13.75f;
        grab_ctrl_cmd->elbow_pitch = 51.11f;
        grab_ctrl_cmd->wrist_pitch = -41.26f;
        grab_ctrl_cmd->wrist_roll = 12.12f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 9:
        grab_ctrl_cmd->base_joint = 14.50f;
        grab_ctrl_cmd->elbow_roll = -13.75f;
        grab_ctrl_cmd->elbow_pitch = 34.39f;
        grab_ctrl_cmd->wrist_pitch = -17.84f;
        grab_ctrl_cmd->wrist_roll = -1.75f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    case 10:
        grab_ctrl_cmd->base_joint = -5.00f;
        grab_ctrl_cmd->elbow_roll = -13.71f;
        grab_ctrl_cmd->elbow_pitch = 12.03f;
        grab_ctrl_cmd->wrist_pitch = 57.04f;
        grab_ctrl_cmd->wrist_roll = -8.26f;
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        break;
    default:
        // 结束时保持最后抓着矿的状态
        break;
    }
}
