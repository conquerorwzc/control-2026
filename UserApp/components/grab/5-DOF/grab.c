//
// Created by ROG on 2025/11/17.
//
/* Private includes ----------------------------------------------------------*/
#include "grab.h"
#include "daemon.h"
#include "user_lib.h"

/* Private macro -------------------------------------------------------------*/
#define PULLEY_GEAR_RATIO 2.125f        // 带轮传动比 17:8
#define BEVEL_GEAR_RATIO 1.6667f        // 锥齿轮传动比 5:3
#define PLANAR_GEAR_RATIO 1.571428f     // 平面齿轮传动比 11:7
#define MOTOR2006_REDUCTION_RATIO 36.0f // 2006 ecd减速比36
// 👇 新增：3508电机减速比 (内部行星齿轮减速比为 19:1)
#define MOTOR3508_REDUCTION_RATIO 51.0f
// 如果你的抬升机构外部还有同步带/齿轮，传动比宏定义也要加上，例如：
// #define LIFT_PULLEY_RATIO 1.5f
#define DM_HOMING_TOLERANCE 5.0f     // DM大臂物理归零的角度容差 (度)
#define DM_CALI_MAX_TICKS 5000       // 阶段一：大臂归零最大允许时间 5 秒 (假设1ms调度)
#define WRIST_CALI_MAX_TICKS 3000    // 阶段二：腕部抬头堵转最大允许时间 6 秒
#define WRIST_CALI_SPEED 0.10f       // 腕部抬升速度
#define WRIST_CALI_CHECK_TICKS 500   // 堵转检测时间
#define WRIST_CALI_TOLERANCE 300.0f  // 堵转容差度数
#define WRIST_CALI_STALL_CURRENT 800 // 堵转电流阈值

// 腕部堵转标定开关 (1: 开启自动撞墙标定 | 0: 关闭，把上电位置直接当做 0 度)
#define USE_WRIST_STALL_CALI 1
// 腕部 Pitch 电机物理挂载开关 (1: 启用发力并检测 | 0: 彻底断电卸力并不参与检测)
#define USE_WRIST_LEFT_MOTOR 1  // 左侧电机
#define USE_WRIST_RIGHT_MOTOR 1 // 右侧电机
// 腕部软件限位安全系数
#define WRIST_SOFT_LIMIT_MARGIN 0.90f

// Roll电机极限安全物理转速 (设定 5000 RPM 为疯转红线)
#define ROLL_SAFE_MAX_RPM 5000.0f
#define ROLL_SAFE_MAX_APS 13000.0f
#define ROLL_SAFE_MAX_DELTA_ANGLE 720.0f

// 机械臂抬升相关参数
#define LIFT_HEIGHT_MAX 420.0f

// ==================== 【图传双向标定参数】 ====================
#define VIDEO_CALI_SPEED_P 0.5f         // M3508 俯仰寻零速度 (度/tick) -> 改为约 100度/秒，温柔寻零
#define VIDEO_CALI_STALL_CURRENT_Y 2500 // M3508 堵转认定电流
#define VIDEO_CALI_STALL_CURRENT_P 2500 // M3508 堵转认定电流 -> 稍微放宽一点防止启动瞬间误触
#define VIDEO_CALI_MAX_TICKS 5000       // 单次寻零超时时间 (10秒)
#define VIDEO_CALI_SLIP_LIMIT_P 60.0f   // P电机目标角度最大超前量 -> 改为 60 防止 PID 算力被卡死导致输出电流不够
#define VIDEO_CALI_MAXOUT_P 6000.0f     // 寻零期间，P电机最大输出(温柔摸墙)
#define VIDEO_SAFE_RANGE_RATIO 0.95f
#define VIDEO_CALI_STALL_SPEED      20.0f   // 转速阈值。低于 30 APS 才判堵转
#define VIDEO_CALI_STALL_TICKS      300     // 连续 150 帧 (0.3秒) 堵转条件才判定成功
/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_M = 0;
static float total_angle_init_Video_yaw = 0;
static float total_angle_init_Video_pitch = 0;
// 👇 新增：抬升电机的上电物理初始零点
static float total_angle_init_arm_lift = 0;
static int error_clear_trigger = 0;
static uint8_t cali_first_run = 1;
/* Private function prototypes -----------------------------------------------*/
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config); // 机械臂初始化，返回一个机械臂示例指针
void GrabTask();                                              // 机械臂任务函数
static void GrabCmdTask();                                    // 机械臂控制命令处理函数
static void MotorTask();                                      // 电机任务函数
static void Grab_Position_Calculate(GrabInstance *grab);      // 计算电机目标位置
static void GrabCalibrationTask(void);                        // 机械臂两段式安全标定任务
static void Grab_Real_Angle_Calculate(GrabInstance *grab);    // 计算机械臂实际角度
static void GrabClearError(void);
static void Error_Check();
static void Wrist_Cali_Check();
static void VideoCalibrationTask(void);
static void Video_Position_Calculate(GrabInstance *grab);
/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化机械臂
 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *)zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *)zmalloc(sizeof(ArmInstance));
    grab_instance->video = (VideoInstance *)zmalloc(sizeof(VideoInstance));

    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
    grab_instance->actuator->grab_djimotor[2] = DJIMotorInit(&Grab_init_config->Grab_motor_config[8]);
    grab_instance->arm->arm_lift_motor = DJIMotorInit(&Grab_init_config->Grab_motor_config[9]);
    // 在没有上电的情况下先不发使能帧给dm电机，即不初始化
    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]); // v2
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);      // v3
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);      // v4
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);      // v4

    grab_instance->video->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    grab_instance->video->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);
    grab_instance->video->video_cali_state = VIDEO_CALI_START;
    // 先赋值grab指针，再访问grab_instance中的成员
    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 初始化电机初始角度，必须要发生在grab的赋值之后
    osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    total_angle_init_M = grab_instance->actuator->grab_djimotor[2]->measure.total_angle;

    // 👇 新增：记录抬升电机的上电初始物理角度
    if (grab->arm->arm_lift_motor != NULL)
    {
        total_angle_init_arm_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->arm->arm_lift_min = total_angle_init_arm_lift;
        grab->arm->arm_lift_max = total_angle_init_arm_lift + LIFT_HEIGHT_MAX;
    }
    if (grab->video->grab_djimotor[0] != NULL)
    {
        total_angle_init_Video_yaw = grab->video->grab_djimotor[0]->measure.total_angle;
    }
    if (grab->video->grab_djimotor[1] != NULL)
    {
        total_angle_init_Video_pitch = grab->video->grab_djimotor[1]->measure.total_angle;
    }
    if (Grab_init_config->Grab_cali_mode == GRAB_CALI_MODE)
    {
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[0]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[1]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[2]);
        DMMotorCaliEncoder(grab->actuator->grab_dmmotor[0]);
    }
    // 初始化时直接将错误码清零即可
    grab->error_code = GRAB_NO_ERROR;
    return grab_instance;
}

/**
 * @brief 机械臂任务函数
 */
void GrabTask()
{
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;
    GrabCmdTask(); // 累加百分比和限幅
    Wrist_Cali_Check(); // 检查手动标定触发

    if (error_clear_trigger == 1) {
        GrabClearError();
    }

    // ================= 模块 1: 机械臂逻辑 =================
    if (grab->actuator->wrist_cali.state != CALI_STAGE_DONE)
    {
        GrabCalibrationTask(); // 标定中：由标定函数控制目标
    }
    else
    {
        Grab_Position_Calculate(grab); // 标定完：由遥控指令控制目标
    }

    // ================= 模块 2: 图传云台逻辑 =================
    if (grab->video->video_cali_state != VIDEO_CALI_DONE)
    {
        VideoCalibrationTask(); // 标定中：由图传标定函数控制 F/P_target
    }
    else
    {
        Video_Position_Calculate(grab); // 标定完：开启百分比解算和软着陆
    }

    Grab_Real_Angle_Calculate(grab); // 计算机械臂实际物理位置
    MotorTask(); // 统一发送给电机
}

static void GrabCmdTask()
{
    if (grab->actuator->wrist_cali.state == CALI_STAGE_DONE)
    {
        if (grab_ctrl_cmd->wrist_pitch > grab->actuator->wrist_cali.max_pitch)
        {
            grab_ctrl_cmd->wrist_pitch = grab->actuator->wrist_cali.max_pitch;
        }
        if (grab_ctrl_cmd->wrist_pitch < grab->actuator->wrist_cali.min_pitch)
        {
            grab_ctrl_cmd->wrist_pitch = grab->actuator->wrist_cali.min_pitch;
        }
    }

    if (grab_ctrl_cmd->arm_lift >= grab->arm->arm_lift_max)
    {
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_max;
    }
    else if (grab_ctrl_cmd->arm_lift < grab->arm->arm_lift_min)
    {
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_min;
    }


    // ================= 图传云台状态更新与硬限幅 =================
    // 键盘输入为离散量 (-1, 0, 1)，步长调回适合键盘的比例
    float video_step = 1.0f / 1550.0f;

    // Yaw只要初始化过了 (不在 START 或 ERROR) 就可以控制
    if (grab->video->video_cali_state != VIDEO_CALI_START && grab->video->video_cali_state != VIDEO_CALI_ERROR)
    {
        grab->video->Video_yaw += grab_ctrl_cmd->video_yaw * video_step;
        // 移除 0~1 的限幅，允许 Yaw 无限旋转
    }

    // Pitch必须等待标定完全完成才能控制
    if (grab->video->video_cali_state == VIDEO_CALI_DONE)
    {
        grab->video->Video_pitch += grab_ctrl_cmd->video_pitch * video_step;
        if (grab->video->Video_pitch >= 1.0f)
        {
            grab->video->Video_pitch = 1.0f;
        }
        else if (grab->video->Video_pitch < 0.0f)
        {
            grab->video->Video_pitch = 0.0f;
        }
    }
    // ================= 赋值给底层实体 =================
    grab->arm->base_joint = grab_ctrl_cmd->base_joint;
    grab->arm->elbow_roll = grab_ctrl_cmd->elbow_roll;
    grab->arm->elbow_pitch = grab_ctrl_cmd->elbow_pitch;
    grab->actuator->wrist_pitch = grab_ctrl_cmd->wrist_pitch;
    grab->actuator->wrist_roll = grab_ctrl_cmd->wrist_roll;
    grab->actuator->torque = grab_ctrl_cmd->torque;
    grab->arm->arm_lift = grab_ctrl_cmd->arm_lift;
}
static void MotorTask()
{
    Error_Check();
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);
        // 👇 新增：断电时卸力抬升电机
        DJIMotorStop(grab->arm->arm_lift_motor);
        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DJIMotorStop(grab->actuator->grab_djimotor[2]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);

        DJIMotorStop(grab->video->grab_djimotor[0]);
        DJIMotorStop(grab->video->grab_djimotor[1]);
    }
    else
    {
        // 循环处理所有DMMotor，只对在线的电机进行使能和PID计算
        for (int i = 0; i < 3; i++)
        {
            if (DaemonIsOnline(grab->arm->grab_dmmotor[i]->daemon))
            {
                DMMotorEnable(grab->arm->grab_dmmotor[i]);
                switch (i)
                {
                case 0:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->base_joint * DEGREE_2_RAD);
                    break;
                case 1:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_roll * DEGREE_2_RAD);
                    break;
                case 2:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_pitch * DEGREE_2_RAD);
                    break;
                }
            }
        }

        // 循环处理所有DJIMotor（actuator部分）
        for (int i = 0; i < 3; i++)
        {
            // 👇 新增：在线则使能 3508 抬升电机，并下发 PID 目标
            if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
            {
                DJIMotorEnable(grab->arm->arm_lift_motor);
                DJIMotorSetPIDRef(grab->arm->arm_lift_motor, grab->grab_ctrl_cmd.arm_lift_target);
            }
            if (DaemonIsOnline(grab->actuator->grab_djimotor[i]->daemon))
            {
                DJIMotorEnable(grab->actuator->grab_djimotor[i]);
                switch (i)
                {
                case 0: // 右电机 (R)
#if USE_WRIST_RIGHT_MOTOR
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->R_target);
#else
                    DJIMotorStop(grab->actuator->grab_djimotor[i]); // 未启用的电机直接断电卸力
#endif
                    break;
                case 1: // 左电机 (L)
#if USE_WRIST_LEFT_MOTOR
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->L_target);
#else
                    DJIMotorStop(grab->actuator->grab_djimotor[i]); // 未启用的电机直接断电卸力
#endif
                    break;
                case 2: // Roll 电机 (M)
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->M_target);
                    break;
                }
            }
        }

        // 处理actuator的DMMotor
        if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
        {
            DMMotorEnable(grab->actuator->grab_dmmotor[0]);
            DMMotorSetRef(grab->actuator->grab_dmmotor[0], grab->actuator->T_target);
        }

        // 👉 你的图传电机保留代码：
        // 循环处理所有DJIMotor（Video部分）
        // 循环处理所有DJIMotor（Video部分）
        for (int i = 0; i < 2; i++)
        {
            // 🌟 如果指针为空，直接跳过，绝对不允许访问 daemon！
            if (grab->video->grab_djimotor[i] != NULL)
            {
                if (DaemonIsOnline(grab->video->grab_djimotor[i]->daemon))
                {
                    DJIMotorEnable(grab->video->grab_djimotor[i]);
                    switch (i)
                    {
                    case 0:
                        DJIMotorSetPIDRef(grab->video->grab_djimotor[i], grab->video->Y_target);
                        break;
                    case 1:
                        DJIMotorSetPIDRef(grab->video->grab_djimotor[i], grab->video->P_target);
                        break;
                    }
                }
                else 
                {
                    // 🌟 必须调用 Stop：这会彻底清空电机底层维持的 PID 旧积分，
                    // 杜绝“重连后直接把掉线期间积累的巨大 I 爆发出引起猛烈跳转”！
                    DJIMotorStop(grab->video->grab_djimotor[i]);
                }
            }
        }
    }
}

/**
 * @brief 根据pitch和roll角度解算出电机的转动角度
 * @param arm 机械臂结构体指针
 */
static void Grab_Position_Calculate(GrabInstance *grab)
{
    /**
     * L = -pitch (同向驱动)
     * R = pitch (同向驱动)
     * M = +roll  (独立电机驱动平面齿轮)
     */

    grab->actuator->R_target =
        total_angle_init_R + grab->actuator->wrist_pitch * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

    grab->actuator->L_target =
        total_angle_init_L - grab->actuator->wrist_pitch * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

    grab->actuator->M_target =
        total_angle_init_M + grab->actuator->wrist_roll * MOTOR2006_REDUCTION_RATIO * PLANAR_GEAR_RATIO;


    // 目标角度 = 初始物理角度 + (指令设定高度 * 减速比 * 外设传动比)
    grab->grab_ctrl_cmd.arm_lift_target =
        total_angle_init_arm_lift + grab_ctrl_cmd->arm_lift * MOTOR3508_REDUCTION_RATIO /* * LIFT_PULLEY_RATIO */;

    grab->actuator->T_target = grab->actuator->torque;



   }

/**
 * @brief 图传云台独立逆解算 (百分比映射 + 90%安全限幅 + LPF软着陆)
 */
static void Video_Position_Calculate(GrabInstance *grab)
{
    if (grab->video->video_cali_state != VIDEO_CALI_START && grab->video->video_cali_state != VIDEO_CALI_ERROR)
    {
        // 取消 90 度软限位限制，允许无限旋转。
        // 改变 1 的 Video_yaw 相当于 180 度。 0.0 作为最初的记录零点 (控制量清零)。
        float theoretical_target_y = total_angle_init_Video_yaw + grab->video->Video_yaw * 180.0f;
        grab->video->Y_target += (theoretical_target_y - grab->video->Y_target) * 0.15f;
    }

    // 🛡️ 只有图传标定完成了，才允许 Pitch 进行软着陆计算
    if (grab->video->video_cali_state == VIDEO_CALI_DONE)
    {
        float p_stroke = grab->video->video_max_p - grab->video->video_min_p;
        float margin = (1.0f - VIDEO_SAFE_RANGE_RATIO) / 2.0f;
        float safe_ratio_p = margin + grab->video->Video_pitch * VIDEO_SAFE_RANGE_RATIO;

        if (safe_ratio_p < margin) safe_ratio_p = margin;
        if (safe_ratio_p > 1.0f - margin) safe_ratio_p = 1.0f - margin;

        float theoretical_target_p = grab->video->video_min_p + p_stroke * safe_ratio_p;
        grab->video->P_target += (theoretical_target_p - grab->video->P_target) * 0.15f;
    }
}
/**
 * @brief 三段式安全标定任务 (底层归零 -> 找最高点90度 -> 找最低点)
 */
static void GrabCalibrationTask(void)
{
    static float cali_pitch = 0.0f;
    static float last_r_angle = 0, last_l_angle = 0;
    static uint16_t block_cnt = 0;
    static uint32_t timeout_cnt = 0;

    // 1. 急停/未使能感知与记忆擦除
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        cali_first_run = 1;
        block_cnt = 0;
        timeout_cnt = 0;
        grab->actuator->wrist_cali.state = CALI_STAGE_DM_WAIT_ZERO;
        return;
    }

    // 2. 首次运行初始化
    if (cali_first_run)
    {
        uint8_t all_online = 1;

#if USE_WRIST_RIGHT_MOTOR
        if (!DaemonIsOnline(grab->actuator->grab_djimotor[0]->daemon))
            all_online = 0;
#endif
#if USE_WRIST_LEFT_MOTOR
        if (!DaemonIsOnline(grab->actuator->grab_djimotor[1]->daemon))
            all_online = 0;
#endif
        if (!DaemonIsOnline(grab->actuator->grab_djimotor[2]->daemon))
            all_online = 0;

        if (!all_online)
            return; // 没上线死等

        total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
        total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
        total_angle_init_M = grab->actuator->grab_djimotor[2]->measure.total_angle;

        last_r_angle = total_angle_init_R;
        last_l_angle = total_angle_init_L;
        cali_pitch = 0.0f;
        timeout_cnt = 0;

#if USE_WRIST_STALL_CALI
        grab->actuator->wrist_cali.state = CALI_STAGE_DM_WAIT_ZERO;
#else
        // 如果关闭了自动堵转，我们直接赋予宽松的默认软限位
        grab->actuator->wrist_cali.max_pitch = 90.0f;
        grab->actuator->wrist_cali.min_pitch = -90.0f;
        grab->actuator->wrist_cali.state = CALI_STAGE_DONE;
#endif

        cali_first_run = 0;
    }

    // 3. 核心状态机
    switch (grab->actuator->wrist_cali.state)
    {
    case CALI_STAGE_DM_WAIT_ZERO: {
        timeout_cnt++;

        // 强行占领大臂底层控制权
        grab_ctrl_cmd->base_joint = 0.0f;
        grab_ctrl_cmd->elbow_pitch = 0.0f;
        grab_ctrl_cmd->elbow_roll = 0.0f;
        grab->arm->base_joint = 0.0f;
        grab->arm->elbow_pitch = 0.0f;
        grab->arm->elbow_roll = 0.0f;

        grab->actuator->R_target = total_angle_init_R;
        grab->actuator->L_target = total_angle_init_L;
        grab->actuator->M_target = total_angle_init_M;

        float curr_base = grab->arm->grab_dmmotor[0]->measure.total_angle * RAD_2_DEGREE;
        float curr_elbow_r = grab->arm->grab_dmmotor[1]->measure.total_angle * RAD_2_DEGREE;
        float curr_elbow_p = grab->arm->grab_dmmotor[2]->measure.total_angle * RAD_2_DEGREE;

        if (fabsf(curr_base) < DM_HOMING_TOLERANCE && fabsf(curr_elbow_r) < DM_HOMING_TOLERANCE &&
            fabsf(curr_elbow_p) < DM_HOMING_TOLERANCE)
        {
            timeout_cnt = 0;
            grab->actuator->wrist_cali.state = CALI_STAGE_WRIST_FIND_MAX;
        }
        else if (timeout_cnt > DM_CALI_MAX_TICKS)
        {
            grab->actuator->wrist_cali.state = CALI_STAGE_ERROR;
        }
        break;
    }

    // =========================================================
    // 阶段二：向上寻找最高限位 (90度)
    // =========================================================
    case CALI_STAGE_WRIST_FIND_MAX: {
        timeout_cnt++;
        grab->actuator->M_target = total_angle_init_M;

        cali_pitch += WRIST_CALI_SPEED; // 往上抬升

        grab->actuator->R_target = total_angle_init_R + (cali_pitch)*MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
        grab->actuator->L_target = total_angle_init_L - (cali_pitch)*MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

        block_cnt++;
        if (block_cnt >= WRIST_CALI_CHECK_TICKS)
        {
            float curr_r = grab->actuator->grab_djimotor[0]->measure.total_angle;
            float curr_l = grab->actuator->grab_djimotor[1]->measure.total_angle;
            float diff_r = fabsf(curr_r - last_r_angle);
            float diff_l = fabsf(curr_l - last_l_angle);
            float curr_amp_r = fabsf((float)grab->actuator->grab_djimotor[0]->measure.real_current);
            float curr_amp_l = fabsf((float)grab->actuator->grab_djimotor[1]->measure.real_current);

            uint8_t stall_triggered = 1;
#if USE_WRIST_RIGHT_MOTOR
            if (!(diff_r < WRIST_CALI_TOLERANCE && curr_amp_r > WRIST_CALI_STALL_CURRENT))
                stall_triggered = 0;
#endif
#if USE_WRIST_LEFT_MOTOR
            if (!(diff_l < WRIST_CALI_TOLERANCE && curr_amp_l > WRIST_CALI_STALL_CURRENT))
                stall_triggered = 0;
#endif

            if (stall_triggered && (USE_WRIST_LEFT_MOTOR || USE_WRIST_RIGHT_MOTOR))
            {
                float ratio_multiplier = MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

                // 💥 撞死最高限位！解算绝对零点
#if USE_WRIST_RIGHT_MOTOR
                total_angle_init_R = curr_r - (90.0f * ratio_multiplier);
#endif
#if USE_WRIST_LEFT_MOTOR
                total_angle_init_L = curr_l - (-90.0f * ratio_multiplier);
#endif

                // 👉 设定最高软件限位：90度 * 0.98 = 88.2度
                grab->actuator->wrist_cali.max_pitch = 90.0f * WRIST_SOFT_LIMIT_MARGIN;

                // 准备进入下一阶段
                cali_pitch = 90.0f; // 从当前90度位置开始向下退
                timeout_cnt = 0;
                block_cnt = 0;
                last_r_angle = curr_r;
                last_l_angle = curr_l;

                grab->actuator->wrist_cali.state = CALI_STAGE_WRIST_FIND_MIN;
                return; // 结束本轮循环
            }
            last_r_angle = curr_r;
            last_l_angle = curr_l;
            block_cnt = 0;
        }

        if (timeout_cnt > WRIST_CALI_MAX_TICKS)
            grab->actuator->wrist_cali.state = CALI_STAGE_ERROR;
        break;
    }

    // =========================================================
    // 阶段三：向下寻找最低限位
    // =========================================================
    case CALI_STAGE_WRIST_FIND_MIN: {
        timeout_cnt++;
        grab->actuator->M_target = total_angle_init_M;

        cali_pitch -= WRIST_CALI_SPEED; // 往下压

        grab->actuator->R_target = total_angle_init_R + (cali_pitch)*MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
        grab->actuator->L_target = total_angle_init_L - (cali_pitch)*MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

        block_cnt++;
        if (block_cnt >= WRIST_CALI_CHECK_TICKS)
        {
            float curr_r = grab->actuator->grab_djimotor[0]->measure.total_angle;
            float curr_l = grab->actuator->grab_djimotor[1]->measure.total_angle;
            float diff_r = fabsf(curr_r - last_r_angle);
            float diff_l = fabsf(curr_l - last_l_angle);
            float curr_amp_r = fabsf((float)grab->actuator->grab_djimotor[0]->measure.real_current);
            float curr_amp_l = fabsf((float)grab->actuator->grab_djimotor[1]->measure.real_current);

            uint8_t stall_triggered = 1;
#if USE_WRIST_RIGHT_MOTOR
            if (!(diff_r < WRIST_CALI_TOLERANCE && curr_amp_r > WRIST_CALI_STALL_CURRENT))
                stall_triggered = 0;
#endif
#if USE_WRIST_LEFT_MOTOR
            if (!(diff_l < WRIST_CALI_TOLERANCE && curr_amp_l > WRIST_CALI_STALL_CURRENT))
                stall_triggered = 0;
#endif

            if (stall_triggered && (USE_WRIST_LEFT_MOTOR || USE_WRIST_RIGHT_MOTOR))
            {
                float ratio_multiplier = MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
                float real_min_pitch = 0.0f;

#if USE_WRIST_LEFT_MOTOR
                // 根据左电机当前值反推当前度数
                real_min_pitch = -(curr_l - total_angle_init_L) / ratio_multiplier;
#elif USE_WRIST_RIGHT_MOTOR
                // 如果只用右电机，则通过右电机反推
                real_min_pitch = (curr_r - total_angle_init_R) / ratio_multiplier;
#endif

                // 👉 设定最低软件限位 (最低真实物理角度 * 0.98安全系数)
                // (如果最低角度是 -30度，乘以0.98就是 -29.4度，完美向安全区收缩)
                grab->actuator->wrist_cali.min_pitch = real_min_pitch * WRIST_SOFT_LIMIT_MARGIN;

                // 防抽搐对齐：标定结束后，立刻把当前指令切到最低安全限位处
                grab_ctrl_cmd->wrist_pitch = grab->actuator->wrist_cali.min_pitch;
                grab_ctrl_cmd->wrist_roll = 0.0f;

                grab->actuator->wrist_cali.state = CALI_STAGE_DONE; // 大功告成！
            }

            last_r_angle = curr_r;
            last_l_angle = curr_l;
            block_cnt = 0;
        }

        if (timeout_cnt > WRIST_CALI_MAX_TICKS)
            grab->actuator->wrist_cali.state = CALI_STAGE_ERROR;
        break;
    }

    case CALI_STAGE_DONE: {
        return;
    }
    case CALI_STAGE_ERROR: {
        grab_ctrl_cmd->base_joint = 0.0f;
        grab_ctrl_cmd->elbow_pitch = 0.0f;
        grab_ctrl_cmd->elbow_roll = 0.0f;
        grab->actuator->R_target = total_angle_init_R;
        grab->actuator->L_target = total_angle_init_L;
        grab->actuator->M_target = total_angle_init_M;
        return;
    }
    default:
        grab->actuator->wrist_cali.state = CALI_STAGE_ERROR;
        break;
    }
}

/**
 * @brief 传动逆解算：全要素机械臂真实物理状态更新
 * @param grab 机械臂结构体指针
 */
static void Grab_Real_Angle_Calculate(GrabInstance *grab)
{
    // =========================================================
    // 1. 解算 DM 大臂部分 (直接读取弧度并转为角度)
    // =========================================================
    if (DaemonIsOnline(grab->arm->grab_dmmotor[0]->daemon))
    {
        grab->grab_measure.base_joint = grab->arm->grab_dmmotor[0]->measure.total_angle * RAD_2_DEGREE;
    }
    if (DaemonIsOnline(grab->arm->grab_dmmotor[1]->daemon))
    {
        grab->grab_measure.elbow_roll = grab->arm->grab_dmmotor[1]->measure.total_angle * RAD_2_DEGREE;
    }
    if (DaemonIsOnline(grab->arm->grab_dmmotor[2]->daemon))
    {
        grab->grab_measure.elbow_pitch = grab->arm->grab_dmmotor[2]->measure.total_angle * RAD_2_DEGREE;
    }

    // =========================================================
    // 2. 解算 DJI 腕部部分 (相对编码器，依赖绝对零点 total_angle_init)
    // =========================================================
    // 必须等待撞墙标定成功，找准了相对零点，算出来的真实物理角度才有意义
    if (grab->actuator->wrist_cali.state == CALI_STAGE_DONE)
    {
        float ratio_pitch = MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
        float ratio_roll = MOTOR2006_REDUCTION_RATIO * PLANAR_GEAR_RATIO;

        float curr_r = grab->actuator->grab_djimotor[0]->measure.total_angle;
        float curr_l = grab->actuator->grab_djimotor[1]->measure.total_angle;
        float curr_m = grab->actuator->grab_djimotor[2]->measure.total_angle;

        // Pitch 轴逆解算 (自适应左/右发力电机)
#if USE_WRIST_LEFT_MOTOR
        // 左电机：L_target = Init_L - Pitch * ratio  =>  Pitch = -(L - Init_L) / ratio
        grab->grab_measure.wrist_pitch = -(curr_l - total_angle_init_L) / ratio_pitch;
#elif USE_WRIST_RIGHT_MOTOR
        // 右电机：R_target = Init_R + Pitch * ratio  =>  Pitch = (R - Init_R) / ratio
        grab->grab_measure.wrist_pitch = (curr_r - total_angle_init_R) / ratio_pitch;
#endif

        // Roll 轴逆解算
        // M_target = Init_M + Roll * ratio => Roll = (M - Init_M) / ratio
        grab->grab_measure.wrist_roll = (curr_m - total_angle_init_M) / ratio_roll;
    }

    // =========================================================
    // 3. 解算图传电机与夹爪力矩 (目前预留，如果你没接电机就不算)
    // =========================================================
    if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
    {
        // 假设你要读末端夹爪 DM 电机的实时反馈电流/力矩
        grab->grab_measure.torque = grab->actuator->grab_dmmotor[0]->measure.torque;
    }

    // 图传部分，等你把图传电机加回来再把下面注释解开

    // =========================================================
    // 3. 解算图传电机 (🌟 加入终极防空指针装甲)
    // =========================================================
    // 必须先判断指针存在，再去判断 Daemon 状态！
    if (grab->video->grab_djimotor[0] != NULL)
    {
        if (DaemonIsOnline(grab->video->grab_djimotor[0]->daemon))
        {
            grab->grab_measure.video_yaw =
                grab->video->grab_djimotor[0]->measure.total_angle - total_angle_init_Video_yaw;
        }
    }

    if (grab->video->grab_djimotor[1] != NULL)
    {
        if (DaemonIsOnline(grab->video->grab_djimotor[1]->daemon))
        {
            grab->grab_measure.video_pitch =
                grab->video->grab_djimotor[1]->measure.total_angle - total_angle_init_Video_pitch;
        }
    }

    // 👇 新增：抬升电机真实物理高度解算
    if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
    {
        float curr_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->grab_measure.arm_lift =
            (curr_lift - total_angle_init_arm_lift) / MOTOR3508_REDUCTION_RATIO /* / LIFT_PULLEY_RATIO */;
    }
}

static void Error_Check()
{
    // Roll 轴保护已移除（超速/超角检测在重连时易误触导致机械臂意外断电）
    // 如需恢复，可在此处重新添加对 grab_djimotor[2] 的监控逻辑
}
/**
 * @brief 清除机械臂的错误状态，允许重新使能
 */
void GrabClearError(void)
{
    if (grab != NULL)
    {
        grab->error_code = GRAB_NO_ERROR;
    }
}

static void Wrist_Cali_Check()
{
    if (grab_ctrl_cmd->wrist_roll_cali == 1)
    {
        total_angle_init_M = grab->actuator->grab_djimotor[2]->measure.total_angle;
        grab_ctrl_cmd->wrist_roll_cali = 0;
    }
    if (grab_ctrl_cmd->wrist_pitch_cali == 1)
    {
        grab->actuator->wrist_cali.state = CALI_STAGE_DM_WAIT_ZERO;
        cali_first_run = 1;
        grab_ctrl_cmd->wrist_pitch_cali = 0;
    }

    // 🌟 新增：检测到图传标定请求，强制重置状态机
    if (grab_ctrl_cmd->video_cali == 1)
    {
        if (grab->video->video_cali_state == VIDEO_CALI_WAIT_BTN || grab->video->video_cali_state == VIDEO_CALI_DONE)
        {
            grab->video->video_cali_state = VIDEO_CALI_START_PITCH;
        }
        grab_ctrl_cmd->video_cali = 0;
    }
}
static void VideoCalibrationTask(void)
{
    if (grab->video->grab_djimotor[0] == NULL || grab->video->grab_djimotor[1] == NULL) return;

    // 1. 最高优先级指令：整机切入断电保护模式时，强制恢复所有状态为出厂刚上电水平！
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF) {
        grab->video->video_cali_state = VIDEO_CALI_START;
        grab->video->Video_yaw = 0.0f; // 断电瞬间立刻把底层控制期望清零
        grab_ctrl_cmd->video_yaw = 0.0f; // 顺带把上层发送的 cmd 指令也强制抹除
        grab_ctrl_cmd->video_pitch = 0.0f;
        return;
    }

    // 2. 硬件断联保护：一旦任何一个电机掉线（如杜邦线松动、CAN被干扰等）
    // 必须立刻将状态机打回原形！因为重连时相对多圈编码器角度已经失效，必须拿上线第一帧作为新的基准！
    if (!DaemonIsOnline(grab->video->grab_djimotor[0]->daemon) || 
        !DaemonIsOnline(grab->video->grab_djimotor[1]->daemon)) {
        grab->video->video_cali_state = VIDEO_CALI_START;
        grab->video->Video_yaw = 0.0f; // 掉线瞬间立刻把底层控制期望清零
        grab_ctrl_cmd->video_yaw = 0.0f; // 同上，拔线的瞬间抹除一切上层指令残留
        grab_ctrl_cmd->video_pitch = 0.0f;
        return;
    }

    // 3. 安全检查完毕，如果已经标定圆满结束，直接跳过任务以节省算力
    if (grab->video->video_cali_state == VIDEO_CALI_DONE) return;

    static float cali_target_p = 0.0f;
    static uint16_t p_stall_cnt = 0; // 🌟 连续堵转计数器
    static uint32_t timeout_cnt = 0;
    static uint8_t p_done = 0;

    // 获取当前物理反馈
    float curr_y = grab->video->grab_djimotor[0]->measure.total_angle;
    float curr_p = grab->video->grab_djimotor[1]->measure.total_angle;
    float curr_amp_p = fabsf((float)grab->video->grab_djimotor[1]->measure.real_current);
    float curr_spd_p = fabsf((float)grab->video->grab_djimotor[1]->measure.speed_aps);

    static float hold_target_p = 0.0f;

    switch (grab->video->video_cali_state)
    {
    case VIDEO_CALI_START:
        total_angle_init_Video_yaw = curr_y; // Yaw 上电立刻获取当前中心点
        
        // 🌟 完全清零：Yaw 积累的控制量在重置时彻底归 0，不再是以前的 0.5
        grab->video->Video_yaw = 0.0f; 
        
        // 🌟 核心：为防止“瞎转”，必须在此立刻初始化目标值为当前位置
        grab->video->Y_target = curr_y; 
        hold_target_p = curr_p; 
        grab->video->P_target = hold_target_p;
        
        grab->video->video_cali_state = VIDEO_CALI_WAIT_BTN;
        break;

    case VIDEO_CALI_WAIT_BTN:
        // 此状态下 Pitch 挂起防卡线，Yaw 不受堵转影响
        // 🌟 必须锁死在上电初始位置，不能随波逐流等于 curr_p，否则会抗拒不了重力掉下来
        grab->video->P_target = hold_target_p; 
        break;

    case VIDEO_CALI_START_PITCH:
        cali_target_p = curr_p;
        p_stall_cnt = 0; timeout_cnt = 0;
        p_done = 0;
        grab->video->video_cali_state = VIDEO_CALI_FIND_MAX;
        break;

    case VIDEO_CALI_FIND_MAX:
    case VIDEO_CALI_FIND_MIN:
        timeout_cnt++;
        // 设置限速输出力矩
        grab->video->grab_djimotor[1]->motor_controller.speed_PID.MaxOut = VIDEO_CALI_MAXOUT_P;

        // 根据阶段决定增量方向
        float dir = (grab->video->video_cali_state == VIDEO_CALI_FIND_MAX) ? 1.0f : -1.0f;

        if (!p_done) cali_target_p += dir * VIDEO_CALI_SPEED_P;

        // 虚拟弹簧限位
        if (cali_target_p > curr_p + VIDEO_CALI_SLIP_LIMIT_P) cali_target_p = curr_p + VIDEO_CALI_SLIP_LIMIT_P;
        if (cali_target_p < curr_p - VIDEO_CALI_SLIP_LIMIT_P) cali_target_p = curr_p - VIDEO_CALI_SLIP_LIMIT_P;

        // Yaw 不能被强行等于 curr_y，否则会和鼠标平滑控制互殴！删去！
        grab->video->P_target = cali_target_p;

        // 🌟 核心：改用转速 + 电流 双重判定
        // 判断 P 电机 (M3508)
        if (!p_done) {
            if (curr_spd_p < VIDEO_CALI_STALL_SPEED && curr_amp_p > VIDEO_CALI_STALL_CURRENT_P) p_stall_cnt++;
            else p_stall_cnt = 0;
            if (p_stall_cnt > VIDEO_CALI_STALL_TICKS) {
                p_done = 1;
                if (grab->video->video_cali_state == VIDEO_CALI_FIND_MAX) grab->video->video_max_p = curr_p;
                else grab->video->video_min_p = curr_p;
            }
        }

        // 检查阶段切换
        if (p_done) {
            if (grab->video->video_cali_state == VIDEO_CALI_FIND_MAX) {
                grab->video->video_cali_state = VIDEO_CALI_FIND_MIN;
                p_done = 0; p_stall_cnt = 0; timeout_cnt = 0;
            } else {
                // 🌟 终点：Pitch 标定成功，强制归中
                total_angle_init_Video_pitch = (grab->video->video_max_p + grab->video->video_min_p) / 2.0f;
                // Yaw 维持最初独立分配的值，不可篡改
                
                grab->video->Video_pitch = 0.5f;   // 归中
                grab_ctrl_cmd->video_pitch = 0.0f;
                grab->video->grab_djimotor[1]->motor_controller.speed_PID.MaxOut = 15000.0f;
                grab->video->video_cali_state = VIDEO_CALI_DONE;
            }
        }
        if (timeout_cnt > VIDEO_CALI_MAX_TICKS) grab->video->video_cali_state = VIDEO_CALI_ERROR;
        break;

    case VIDEO_CALI_ERROR:
        grab->video->Y_target = curr_y;
        grab->video->P_target = curr_p;
        break;
    }
}