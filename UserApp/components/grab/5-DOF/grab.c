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
#define MOTOR3508_P51_REDUCTION_RATIO 51.0f // 3508电机减速比
#define MOTOR3508_P19_REDUCTION_RATIO 19.0f // 3508电机减速比

#define DM_HOMING_TOLERANCE 5.0f     // DM大臂物理归零的角度容差 (度)
#define DM_CALI_MAX_TICKS 5000       // 阶段一：大臂归零最大允许时间 5 秒 (假设1ms调度)
#define WRIST_CALI_MAX_TICKS 3000    // 阶段二：腕部抬头堵转最大允许时间 6 秒
#define WRIST_CALI_SPEED 0.10f       // 腕部抬升速度
#define WRIST_CALI_CHECK_TICKS 500   // 堵转检测时间
#define WRIST_CALI_TOLERANCE 300.0f  // 堵转容差度数
#define WRIST_CALI_STALL_CURRENT 800 // 堵转电流阈值

#define EXTEND_CALI_MAX_TICKS 5000    // 前伸堵转最大允许时间 5 秒
#define EXTEND_CALI_SPEED 0.2f       // 前伸回缩寻找最小值的速度 (需根据实际传动比微调)
#define EXTEND_CALI_CHECK_TICKS 300   // 堵转检测时间周期
#define EXTEND_CALI_TOLERANCE 50.0f  // 堵转容差度数 (编码器差值)
#define EXTEND_CALI_STALL_CURRENT 1000 // 堵转电流阈值

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

/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_M = 0;
static float total_angle_init_arm_lift = 0; // 抬升电机的上电物理初始零点
static float total_angle_init_arm_extend = 0; // 前伸电机的上电物理初始零点
static int error_clear_trigger = 0;
static uint8_t cali_first_run = 1;

static Grab_Param_s grab_param; // 新增：用于存储从 config 传过来的限位与灵敏度配置

static GPIO_Init_Config_s gpio_init_config_micro_switch = {
    .GPIO_Pin = Micro_switch_Pin,
    .GPIOx = Micro_switch_GPIO_Port,
    .pin_state = GPIO_PIN_RESET,
};
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
static void ArmExtendCalibrationTask(void);

/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化机械臂
 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *)zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *)zmalloc(sizeof(ArmInstance));
    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
    grab_instance->actuator->grab_djimotor[2] = DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    grab_instance->arm->arm_lift_motor = DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);
    grab_instance->arm->arm_extend_motor = DJIMotorInit(&Grab_init_config->Grab_motor_config[8]); // 新增前伸电机
    grab_instance->arm->micro_switch_gpio = GPIORegister(&gpio_init_config_micro_switch);
    // 在没有上电的情况下先不发使能帧给dm电机，即不初始化
    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]); // v2
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);      // v3
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);      // v4
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);      // v4

    // 先赋值grab指针，再访问grab_instance中的成员
    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 👇 核心修改：将传入的配置拷贝到静态变量中，供全局使用
    grab_param = Grab_init_config->Grab_param;

    // 初始化电机初始角度，必须要发生在grab的赋值之后
    osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    total_angle_init_M = grab_instance->actuator->grab_djimotor[2]->measure.total_angle;

    // 记录抬升电机的上电初始物理角度
    if (grab->arm->arm_lift_motor != NULL)
    {
        total_angle_init_arm_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->arm->arm_lift_min = 0;
        grab->arm->arm_lift_max = total_angle_init_arm_lift + grab_param.arm_lift_max;
    }
    // 记录前伸电机的上电初始物理角度
    if (grab->arm->arm_extend_motor != NULL)
    {
        total_angle_init_arm_extend = grab->arm->arm_extend_motor->measure.total_angle;
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
        GrabCalibrationTask(); // 标定中：由腕部标定函数控制目标
    }
    else
    {
        Grab_Position_Calculate(grab); // 标定完：由遥控指令解算目标
    }


    if (grab->arm->extend_cali.state != EXTEND_CALI_DONE)
    {
        ArmExtendCalibrationTask(); // 未标定完前，接管并覆盖 arm_extend_target
    }

    Grab_Real_Angle_Calculate(grab); // 计算机械臂实际物理位置
    MotorTask(); // 统一发送给电机
}

static void GrabCmdTask()
{
    // ====== 原有：腕部限位 ======
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

    // ====== 原有：抬升限位 ======
    if (grab_ctrl_cmd->arm_lift >= grab->arm->arm_lift_max)
    {
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_max;
    }
    else if (grab_ctrl_cmd->arm_lift < grab->arm->arm_lift_min)
    {
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_min;
    }

    if (grab->arm->extend_cali.state == EXTEND_CALI_DONE)
    {
        if (grab_ctrl_cmd->arm_extend > grab->arm->extend_cali.max_extend)
        {
            grab_ctrl_cmd->arm_extend = grab->arm->extend_cali.max_extend;
        }
        else if (grab_ctrl_cmd->arm_extend < grab->arm->extend_cali.min_extend)
        {
            grab_ctrl_cmd->arm_extend = grab->arm->extend_cali.min_extend;
        }
    }

    // 👇 修改：使用 grab_param 配置项进行三大关节严格物理限位 👇
    // 1. 大 Pitch 限位
    if (grab_ctrl_cmd->elbow_pitch > grab_param.elbow_pitch_max) grab_ctrl_cmd->elbow_pitch = grab_param.elbow_pitch_max;
    else if (grab_ctrl_cmd->elbow_pitch < grab_param.elbow_pitch_min) grab_ctrl_cmd->elbow_pitch = grab_param.elbow_pitch_min;

    // 2. 大 Yaw 限位
    if (grab_ctrl_cmd->base_joint > grab_param.base_joint_max) grab_ctrl_cmd->base_joint = grab_param.base_joint_max;
    else if (grab_ctrl_cmd->base_joint < grab_param.base_joint_min) grab_ctrl_cmd->base_joint = grab_param.base_joint_min;

    // 3. 大 Roll 限位
    if (grab_ctrl_cmd->elbow_roll > grab_param.elbow_roll_max) grab_ctrl_cmd->elbow_roll = grab_param.elbow_roll_max;
    else if (grab_ctrl_cmd->elbow_roll < grab_param.elbow_roll_min) grab_ctrl_cmd->elbow_roll = grab_param.elbow_roll_min;

    // ================= 赋值给底层实体 =================
    grab->arm->base_joint = grab_ctrl_cmd->base_joint;
    grab->arm->elbow_roll = grab_ctrl_cmd->elbow_roll;
    grab->arm->elbow_pitch = grab_ctrl_cmd->elbow_pitch;
    grab->actuator->wrist_pitch = grab_ctrl_cmd->wrist_pitch;
    grab->actuator->wrist_roll = grab_ctrl_cmd->wrist_roll;
    grab->actuator->torque = grab_ctrl_cmd->torque;
    grab->arm->arm_lift = grab_ctrl_cmd->arm_lift;
    grab->grab_ctrl_cmd.arm_extend = grab_ctrl_cmd->arm_extend; // 测试使用，仅直接透传
}

static void MotorTask()
{
    Error_Check();
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);
        // 断电时卸力抬升电机
        DJIMotorStop(grab->arm->arm_lift_motor);
        DJIMotorStop(grab->arm->arm_extend_motor); // 断电时卸力前伸电机
        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DJIMotorStop(grab->actuator->grab_djimotor[2]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);
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
            // 在线则使能 3508 抬升电机，并下发 PID 目标
            if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
            {
                DJIMotorEnable(grab->arm->arm_lift_motor);
                DJIMotorSetPIDRef(grab->arm->arm_lift_motor, grab->grab_ctrl_cmd.arm_lift_target);
            }
            // 测试前伸电机负载：保持设定位置
            if (DaemonIsOnline(grab->arm->arm_extend_motor->daemon))
            {
                DJIMotorEnable(grab->arm->arm_extend_motor);
                DJIMotorSetPIDRef(grab->arm->arm_extend_motor, grab->grab_ctrl_cmd.arm_extend_target);
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
    }
}

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
        total_angle_init_arm_lift + grab_ctrl_cmd->arm_lift * MOTOR3508_P51_REDUCTION_RATIO;
    
    // 前伸电机目标计算：同样简单映射角度环
    grab->grab_ctrl_cmd.arm_extend_target = 
        total_angle_init_arm_extend + grab_ctrl_cmd->arm_extend * MOTOR3508_P19_REDUCTION_RATIO;

    grab->actuator->T_target = grab->actuator->torque;
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
    // 3. 解算图传电机与夹爪力矩
    // =========================================================
    if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
    {
        grab->grab_measure.torque = grab->actuator->grab_dmmotor[0]->measure.torque;
    }

    // 👇 新增：抬升电机真实物理高度解算
    if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
    {
        float curr_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->grab_measure.arm_lift =
            (curr_lift - total_angle_init_arm_lift) / MOTOR3508_P51_REDUCTION_RATIO /* / LIFT_PULLEY_RATIO */;
    }
    
    // 前伸电机真实位置解算
    if (DaemonIsOnline(grab->arm->arm_extend_motor->daemon))
    {
        float curr_extend = grab->arm->arm_extend_motor->measure.total_angle;
        grab->grab_measure.arm_extend =
            (curr_extend - total_angle_init_arm_extend) / MOTOR3508_P19_REDUCTION_RATIO;
    }
    if (grab->arm->micro_switch_gpio != NULL)
    {
        grab->grab_measure.micro_switch_state = GPIORead(grab->arm->micro_switch_gpio);
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
}

/**
 * @brief 前伸电机防呆版微动开关标定任务 (先退后进，寻找精确最小值)
 */
static void ArmExtendCalibrationTask(void)
{
    static float cali_extend = 0.0f;
    static uint32_t timeout_cnt = 0;

    // 1. 急停或断电状态下复位状态机
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        grab->arm->extend_cali.state = EXTEND_CALI_WAIT;
        timeout_cnt = 0;
        return;
    }

    // 没上线死等
    if (!DaemonIsOnline(grab->arm->arm_extend_motor->daemon)) return;

    // 2. 核心状态机
    switch (grab->arm->extend_cali.state)
    {
        case EXTEND_CALI_WAIT: {
            cali_extend = 0.0f;
            timeout_cnt = 0;

            // 👇 防呆判断：一上电时，看看有没有压在开关上
            if (grab->arm->micro_switch_gpio != NULL &&
                GPIORead(grab->arm->micro_switch_gpio) == GPIO_PIN_RESET)
            {
                // 如果压着开关，先进入“离开”状态
                grab->arm->extend_cali.state = EXTEND_CALI_LEAVE_SWITCH;
            }
            else
            {
                // 如果没压着，直接进入“寻找”状态
                grab->arm->extend_cali.state = EXTEND_CALI_FIND_MIN;
            }
            break;
        }

        // =========================================================
        // 新增阶段：离开微动开关
        // =========================================================
        case EXTEND_CALI_LEAVE_SWITCH: {
            timeout_cnt++;
            cali_extend += EXTEND_CALI_SPEED; // 👈 往前伸，让机构离开微动开关

            grab->grab_ctrl_cmd.arm_extend_target = total_angle_init_arm_extend + cali_extend * MOTOR3508_P19_REDUCTION_RATIO;

            // 检查是否已经脱离开关 (读到高电平 SET)
            if (grab->arm->micro_switch_gpio != NULL &&
                GPIORead(grab->arm->micro_switch_gpio) == GPIO_PIN_SET)
            {
                timeout_cnt = 0; // 重置超时计数
                // 成功脱离，开始往回缩去寻找精确零点
                grab->arm->extend_cali.state = EXTEND_CALI_FIND_MIN;
            }

            if (timeout_cnt > EXTEND_CALI_MAX_TICKS) {
                grab->arm->extend_cali.state = EXTEND_CALI_ERROR;
            }
            break;
        }

        // =========================================================
        // 核心阶段：往回缩寻找精确物理零点
        // =========================================================
        case EXTEND_CALI_FIND_MIN: {
            timeout_cnt++;
            // 💡 这里你可以把 EXTEND_CALI_SPEED 乘以 0.5f，让寻找速度慢一半，精度更高
            cali_extend -= EXTEND_CALI_SPEED; // 👈 往回缩

            grab->grab_ctrl_cmd.arm_extend_target = total_angle_init_arm_extend + cali_extend * MOTOR3508_P19_REDUCTION_RATIO;

            // 撞到开关 (读到低电平 RESET)
            if (grab->arm->micro_switch_gpio != NULL &&
                GPIORead(grab->arm->micro_switch_gpio) == GPIO_PIN_RESET)
            {
                float curr_angle = grab->arm->arm_extend_motor->measure.total_angle;

                // 💥 撞到物理零点！
                total_angle_init_arm_extend = curr_angle;

                // 👉 设定软限位：最小为 0，最大为 0 + 800
                grab->arm->extend_cali.min_extend = 0.0f;
                grab->arm->extend_cali.max_extend = 800.0f;

                // 同步将当前的操控指令切到 0 处
                grab_ctrl_cmd->arm_extend = grab->arm->extend_cali.min_extend;

                // 大功告成
                grab->arm->extend_cali.state = EXTEND_CALI_DONE;
            }

            if (timeout_cnt > EXTEND_CALI_MAX_TICKS) {
                grab->arm->extend_cali.state = EXTEND_CALI_ERROR;
            }
            break;
        }

        case EXTEND_CALI_DONE:
        case EXTEND_CALI_ERROR:
            // 结束后控制权交还给常规解算
            break;
    }
}