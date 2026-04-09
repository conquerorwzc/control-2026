//
// Created by ROG on 2025/11/17.
//
/* Private includes ----------------------------------------------------------*/
#include "grab.h"
#include "daemon.h"
#include "user_lib.h"

/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_M = 0;
static float total_angle_init_arm_lift = 0;
static float total_angle_init_arm_extend = 0;
static int error_clear_trigger = 0;
static uint8_t cali_first_run = 1;
static uint8_t extend_switch_broken = 0; // 记录开关是否损坏

static Grab_Param_s grab_param; // 核心：全局配置参数载体

static GPIO_Init_Config_s gpio_init_config_micro_switch = {
    .GPIO_Pin = Micro_switch_Pin,
    .GPIOx = Micro_switch_GPIO_Port,
    .pin_state = GPIO_PIN_RESET,
};

/* Private function prototypes -----------------------------------------------*/
static void GrabCmdTask();
static void MotorTask();
static void Grab_Position_Calculate(GrabInstance *grab);
static void Grab_Real_Angle_Calculate(GrabInstance *grab);
static void GrabClearError(void);
static void Error_Check();
static void Wrist_Cali_Check();
static void Wrist_Cali_Update(Calibration_t *self);
static void Extend_Cali_Update(Calibration_t *self);

void Execute_Calibration(Calibration_t *cali_obj)
{
    if (cali_obj->state == CALI_DONE || cali_obj->state == CALI_ERROR)
        return;

    if (cali_obj->Execute_Logic != NULL)
    {
        cali_obj->Execute_Logic(cali_obj);
    }

    if (cali_obj->timeout_cnt > cali_obj->max_timeout)
    {
        cali_obj->state = CALI_ERROR;
    }
}

/* ========================================================================= */
/* 初始化与主任务调度                                                          */
/* ========================================================================= */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *)zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *)zmalloc(sizeof(ArmInstance));

    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
    grab_instance->actuator->grab_djimotor[2] = DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    grab_instance->arm->arm_lift_motor = DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);
    grab_instance->arm->arm_extend_motor = DJIMotorInit(&Grab_init_config->Grab_motor_config[8]);
    grab_instance->arm->micro_switch_gpio = GPIORegister(&gpio_init_config_micro_switch);

    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]);
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);

    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    grab_param = Grab_init_config->Grab_param;

    grab_ctrl_cmd->base_joint = 10.0f;
    grab->arm->base_joint = 10.0f;

    osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    total_angle_init_M = grab_instance->actuator->grab_djimotor[2]->measure.total_angle;

    if (grab->arm->arm_lift_motor != NULL)
    {
        total_angle_init_arm_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->arm->arm_lift_min = 0;
        grab->arm->arm_lift_max = total_angle_init_arm_lift + grab_param.arm_lift_max;
    }
    if (grab->arm->arm_extend_motor != NULL)
    {
        total_angle_init_arm_extend = grab->arm->arm_extend_motor->measure.total_angle;
    }

    // 腕部标定对象初始化
    grab->actuator->wrist_cali_obj.state = CALI_RUNNING;
    grab->actuator->wrist_cali_obj.internal_step = 0;
    grab->actuator->wrist_cali_obj.timeout_cnt = 0;
    grab->actuator->wrist_cali_obj.max_timeout = grab_param.wrist_cali_max_ticks + grab_param.dm_cali_max_ticks;
    grab->actuator->wrist_cali_obj.host_ptr = grab;
    grab->actuator->wrist_cali_obj.Execute_Logic = Wrist_Cali_Update;

    // 前伸标定对象初始化
    grab->arm->extend_cali_obj.state = CALI_RUNNING;
    grab->arm->extend_cali_obj.internal_step = 0;
    grab->arm->extend_cali_obj.timeout_cnt = 0;
    grab->arm->extend_cali_obj.max_timeout = grab_param.extend_cali_max_ticks;
    grab->arm->extend_cali_obj.host_ptr = grab;
    grab->arm->extend_cali_obj.Execute_Logic = Extend_Cali_Update;

    if (Grab_init_config->Grab_cali_mode == GRAB_CALI_MODE)
    {
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[0]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[1]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[2]);
        DMMotorCaliEncoder(grab->actuator->grab_dmmotor[0]);
    }
    grab->error_code = GRAB_NO_ERROR;
    return grab_instance;
}

void GrabTask()
{
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;
    GrabCmdTask();
    Wrist_Cali_Check();

    if (error_clear_trigger == 1)
    {
        GrabClearError();
    }

    // =========================================================
    // 面向对象的机械臂标定调度
    // =========================================================
    // 1. 腕部标定调度
    if (grab->actuator->wrist_cali_obj.state != CALI_DONE)
    {
        Execute_Calibration(&grab->actuator->wrist_cali_obj);
    }
    else
    {
        Grab_Position_Calculate(grab); // 腕部标定完才允许指令解算
    }

    // 2. 前伸标定调度
    if (grab->arm->extend_cali_obj.state != CALI_DONE)
    {
        Execute_Calibration(&grab->arm->extend_cali_obj);
    }

    Grab_Real_Angle_Calculate(grab);
    MotorTask();
}

/* ========================================================================= */
/* 子类实现：腕部标定与前伸标定特有逻辑                                         */
/* ========================================================================= */

/**
 * @brief 腕部三段式标定逻辑
 */
static void Wrist_Cali_Update(Calibration_t *self)
{
    GrabInstance *g_inst = (GrabInstance *)self->host_ptr;
    static float cali_pitch = 0.0f;
    static float last_r_angle = 0, last_l_angle = 0;
    static uint16_t block_cnt = 0;

    // 急停复位
    if (g_inst->grab_ctrl_cmd.grab_mode == GRAB_POWER_OFF)
    {
        cali_first_run = 1;
        block_cnt = 0;
        self->timeout_cnt = 0;
        self->internal_step = 0;
        return;
    }

    // 首次运行检查上线
    if (cali_first_run)
    {
        uint8_t all_online = 1;
        if (grab_param.use_wrist_right_motor && !DaemonIsOnline(g_inst->actuator->grab_djimotor[0]->daemon))
            all_online = 0;
        if (grab_param.use_wrist_left_motor && !DaemonIsOnline(g_inst->actuator->grab_djimotor[1]->daemon))
            all_online = 0;
        if (!DaemonIsOnline(g_inst->actuator->grab_djimotor[2]->daemon))
            all_online = 0;
        if (!all_online)
            return;

        total_angle_init_R = g_inst->actuator->grab_djimotor[0]->measure.total_angle;
        total_angle_init_L = g_inst->actuator->grab_djimotor[1]->measure.total_angle;
        total_angle_init_M = g_inst->actuator->grab_djimotor[2]->measure.total_angle;

        last_r_angle = total_angle_init_R;
        last_l_angle = total_angle_init_L;
        cali_pitch = 0.0f;
        self->timeout_cnt = 0;

        if (grab_param.use_wrist_stall_cali)
        {
            self->internal_step = 0; // 进入阶段0：DM复位
        }
        else
        {
            g_inst->actuator->max_pitch = 90.0f;
            g_inst->actuator->min_pitch = -90.0f;
            self->state = CALI_DONE;
        }
        cali_first_run = 0;
    }

    self->timeout_cnt++;

    switch (self->internal_step)
    {
    case 0: { // 阶段0：等待DM大臂物理归零
        grab_ctrl_cmd->base_joint = 10.0f;
        grab_ctrl_cmd->elbow_pitch = 0.0f;
        grab_ctrl_cmd->elbow_roll = 0.0f;
        g_inst->arm->base_joint = 10.0f;
        g_inst->arm->elbow_pitch = 0.0f;
        g_inst->arm->elbow_roll = 0.0f;

        g_inst->actuator->R_target = total_angle_init_R;
        g_inst->actuator->L_target = total_angle_init_L;
        g_inst->actuator->M_target = total_angle_init_M;

        float curr_base = g_inst->arm->grab_dmmotor[0]->measure.total_angle * RAD_2_DEGREE;
        float curr_elbow_r = g_inst->arm->grab_dmmotor[1]->measure.total_angle * RAD_2_DEGREE;
        float curr_elbow_p = g_inst->arm->grab_dmmotor[2]->measure.total_angle * RAD_2_DEGREE;

        if (fabsf(curr_base - 10.0f) < grab_param.dm_homing_tolerance &&
             fabsf(curr_elbow_r) < grab_param.dm_homing_tolerance &&
             fabsf(curr_elbow_p) < grab_param.dm_homing_tolerance)
        {
            self->timeout_cnt = 0;
            self->internal_step = 1;
        }
        break;
    }
    case 1: { // 阶段1：向上找最高限位
        g_inst->actuator->M_target = total_angle_init_M;
        cali_pitch += grab_param.wrist_cali_speed;

        g_inst->actuator->R_target =
            total_angle_init_R + (cali_pitch)*grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;
        g_inst->actuator->L_target =
            total_angle_init_L - (cali_pitch)*grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;

        block_cnt++;
        if (block_cnt >= grab_param.wrist_cali_check_ticks)
        {
            float curr_r = g_inst->actuator->grab_djimotor[0]->measure.total_angle;
            float curr_l = g_inst->actuator->grab_djimotor[1]->measure.total_angle;
            float diff_r = fabsf(curr_r - last_r_angle);
            float diff_l = fabsf(curr_l - last_l_angle);
            float curr_amp_r = fabsf((float)g_inst->actuator->grab_djimotor[0]->measure.real_current);
            float curr_amp_l = fabsf((float)g_inst->actuator->grab_djimotor[1]->measure.real_current);

            uint8_t stall_triggered = 1;
            if (grab_param.use_wrist_right_motor &&
                !(diff_r < grab_param.wrist_cali_tolerance && curr_amp_r > grab_param.wrist_cali_stall_current))
                stall_triggered = 0;
            if (grab_param.use_wrist_left_motor &&
                !(diff_l < grab_param.wrist_cali_tolerance && curr_amp_l > grab_param.wrist_cali_stall_current))
                stall_triggered = 0;

            if (stall_triggered && (grab_param.use_wrist_left_motor || grab_param.use_wrist_right_motor))
            {
                float ratio_multiplier = grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;
                if (grab_param.use_wrist_right_motor)
                    total_angle_init_R = curr_r - (90.0f * ratio_multiplier);
                if (grab_param.use_wrist_left_motor)
                    total_angle_init_L = curr_l - (-90.0f * ratio_multiplier);

                g_inst->actuator->max_pitch = 90.0f * grab_param.wrist_soft_limit_margin;

                cali_pitch = 90.0f;
                self->timeout_cnt = 0;
                block_cnt = 0;
                last_r_angle = curr_r;
                last_l_angle = curr_l;
                self->internal_step = 2;
                return;
            }
            last_r_angle = curr_r;
            last_l_angle = curr_l;
            block_cnt = 0;
        }
        break;
    }
    case 2: { // 阶段2：向下找最低限位
        g_inst->actuator->M_target = total_angle_init_M;
        cali_pitch -= grab_param.wrist_cali_speed;

        g_inst->actuator->R_target =
            total_angle_init_R + (cali_pitch)*grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;
        g_inst->actuator->L_target =
            total_angle_init_L - (cali_pitch)*grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;

        block_cnt++;
        if (block_cnt >= grab_param.wrist_cali_check_ticks)
        {
            float curr_r = g_inst->actuator->grab_djimotor[0]->measure.total_angle;
            float curr_l = g_inst->actuator->grab_djimotor[1]->measure.total_angle;
            float diff_r = fabsf(curr_r - last_r_angle);
            float diff_l = fabsf(curr_l - last_l_angle);
            float curr_amp_r = fabsf((float)g_inst->actuator->grab_djimotor[0]->measure.real_current);
            float curr_amp_l = fabsf((float)g_inst->actuator->grab_djimotor[1]->measure.real_current);

            uint8_t stall_triggered = 1;
            if (grab_param.use_wrist_right_motor &&
                !(diff_r < grab_param.wrist_cali_tolerance && curr_amp_r > grab_param.wrist_cali_stall_current))
                stall_triggered = 0;
            if (grab_param.use_wrist_left_motor &&
                !(diff_l < grab_param.wrist_cali_tolerance && curr_amp_l > grab_param.wrist_cali_stall_current))
                stall_triggered = 0;

            if (stall_triggered && (grab_param.use_wrist_left_motor || grab_param.use_wrist_right_motor))
            {
                float ratio_multiplier = grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;
                float real_min_pitch = 0.0f;

                if (grab_param.use_wrist_left_motor)
                    real_min_pitch = -(curr_l - total_angle_init_L) / ratio_multiplier;
                else if (grab_param.use_wrist_right_motor)
                    real_min_pitch = (curr_r - total_angle_init_R) / ratio_multiplier;

                g_inst->actuator->min_pitch = real_min_pitch * grab_param.wrist_soft_limit_margin;

                g_inst->grab_ctrl_cmd.wrist_pitch = g_inst->actuator->min_pitch;
                g_inst->grab_ctrl_cmd.wrist_roll = 0.0f;

                self->state = CALI_DONE; // 大功告成！
            }
            last_r_angle = curr_r;
            last_l_angle = curr_l;
            block_cnt = 0;
        }
        break;
    }
    }
}

/**
 * @brief OOP化：前伸电机防呆版微动开关标定 (重载防跳齿 + 防半路卡死终极版)
 */
static void Extend_Cali_Update(Calibration_t *self)
{
    GrabInstance *g_inst = (GrabInstance *)self->host_ptr;
    static uint16_t extend_stall_cnt = 0; // 堵转看门狗计数器

    // 急停或掉电时，清空所有状态
    if (g_inst->grab_ctrl_cmd.grab_mode == GRAB_POWER_OFF) {
        self->timeout_cnt = 0;
        self->internal_step = 0;
        extend_stall_cnt = 0;
        return;
    }

    // 电机未上线时保护，不执行逻辑
    if (!DaemonIsOnline(g_inst->arm->arm_extend_motor->daemon)) return;

    self->timeout_cnt++;

    switch (self->internal_step)
    {
    case 0: { // 阶段0：侦测上电状态
        extend_stall_cnt = 0;
        if (g_inst->arm->micro_switch_gpio != NULL && GPIORead(g_inst->arm->micro_switch_gpio) == GPIO_PIN_RESET)
            self->internal_step = 1; // 如果一开始就压着开关，先去阶段1往外退一点
        else
            self->internal_step = 2; // 如果没压着，直接去阶段2往回收
        break;
    }
    case 1: { // 阶段1：脱离微动开关 (稍微往外伸一点)
        g_inst->grab_ctrl_cmd.arm_extend += grab_param.extend_cali_speed;

        if (g_inst->arm->micro_switch_gpio != NULL && GPIORead(g_inst->arm->micro_switch_gpio) == GPIO_PIN_SET) {
            self->timeout_cnt = 0;
            self->internal_step = 2; // 开关弹起，进入阶段2正式标定
        }
        break;
    }
    case 2: { // 阶段2：重载回拉，寻找物理零点
        // 速度一定要慢，防止PID误差瞬间拉大
        g_inst->grab_ctrl_cmd.arm_extend -= grab_param.extend_cali_speed;

        uint8_t switch_triggered = (g_inst->arm->micro_switch_gpio != NULL &&
                                    GPIORead(g_inst->arm->micro_switch_gpio) == GPIO_PIN_RESET);

        // 💥 重载专用看门狗：实时监控电流和速度
        float curr_amp = fabsf((float)g_inst->arm->arm_extend_motor->measure.real_current);
        float curr_speed = fabsf((float)g_inst->arm->arm_extend_motor->measure.speed_rpm);
        uint8_t stall_triggered = 0;

        // 【调试参数】：5000.0f 是堵转阈值，10.0f 是速度阈值
        if (curr_amp > 5000.0f && curr_speed < 10.0f) {
            extend_stall_cnt++;
            // 【极速刹车】：持续 20 个 tick (0.04 秒)，立刻判定撞底/卡死！
            if (extend_stall_cnt > 20) {
                stall_triggered = 1;
            }
        } else {
            extend_stall_cnt = 0; // 一旦恢复正常，计数器清零
        }

        // =======================================================
        // 🌟 核心分流：真归零 vs 假卡顿
        // =======================================================
        if (switch_triggered)
        {
            // 【情况 A：完美撞底】—— 摸到开关了！
            // 更新物理坐标系
            total_angle_init_arm_extend = g_inst->arm->arm_extend_motor->measure.total_angle;

            // 设定安全限位
            g_inst->arm->min_extend = 0.0f;
            g_inst->arm->max_extend = 800.0f;
            g_inst->grab_ctrl_cmd.arm_extend = 0.0f;

            extend_switch_broken = 0; // 开关正常工作
            self->state = CALI_DONE;  // 标定大功告成！
            extend_stall_cnt = 0;
        }
        else if (stall_triggered)
        {
            // 【情况 B：半路卡死 或 开关撞碎】—— 电流爆了，但开关没按下去！
            // 1. 立刻刹车：把目标位置改为当前的实际物理位置，PID误差瞬间清零，电流瞬间卸掉，绝对不跳齿！
            float curr_actual_extend = (g_inst->arm->arm_extend_motor->measure.total_angle - total_angle_init_arm_extend) / grab_param.motor3508_p19_reduction_ratio;
            g_inst->grab_ctrl_cmd.arm_extend = curr_actual_extend;

            // 2. 报错拦截：绝对不更新0点！进入异常状态！
            self->state = CALI_ERROR;
            extend_stall_cnt = 0;
        }
        break;
    }
    }
}
/* ========================================================================= */
/* 限位解算与发送任务                                                         */
/* ========================================================================= */
static void GrabCmdTask()
{
    if (grab->actuator->wrist_cali_obj.state == CALI_DONE)
    {
        if (grab_ctrl_cmd->wrist_pitch > grab->actuator->max_pitch)
            grab_ctrl_cmd->wrist_pitch = grab->actuator->max_pitch;
        if (grab_ctrl_cmd->wrist_pitch < grab->actuator->min_pitch)
            grab_ctrl_cmd->wrist_pitch = grab->actuator->min_pitch;
    }

    if (grab_ctrl_cmd->arm_lift >= grab->arm->arm_lift_max)
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_max;
    else if (grab_ctrl_cmd->arm_lift < grab->arm->arm_lift_min)
        grab_ctrl_cmd->arm_lift = grab->arm->arm_lift_min;

    static uint8_t last_climb_mode = 0;

    if (grab->arm->extend_cali_obj.state == CALI_DONE)
    {
        // 边缘检测：如果你【刚退出】上台阶模式
        if (last_climb_mode == 1 && grab_ctrl_cmd->is_climb_mode == 0)
        {
            // 重新激活标定状态机！这会让电机平稳地往回收缩，直到碰触微动开关，彻底消除跳齿误差
            grab->arm->extend_cali_obj.state = CALI_RUNNING;
            grab->arm->extend_cali_obj.internal_step = 0;
            grab->arm->extend_cali_obj.timeout_cnt = 0;
        }

        if (grab_ctrl_cmd->is_climb_mode)
        {
            // 只有在上台阶模式下，才允许在 0~max 之间正常伸缩
            if (grab_ctrl_cmd->arm_extend > grab->arm->max_extend)
                grab_ctrl_cmd->arm_extend = grab->arm->max_extend;
            else if (grab_ctrl_cmd->arm_extend < grab->arm->min_extend)
                grab_ctrl_cmd->arm_extend = grab->arm->min_extend;
        }
        else
        {
            // 非上台阶模式，强制锁死目标在绝对物理零点
            grab_ctrl_cmd->arm_extend = 0.0f;
        }

        // 0自动归位
        if (grab_ctrl_cmd->arm_extend <= 0.01f && !extend_switch_broken)
        {
            if (grab->arm->micro_switch_gpio != NULL && GPIORead(grab->arm->micro_switch_gpio) == GPIO_PIN_SET)
            {
                // 开关松了？立刻转为缓慢回拉寻找开关！
                grab->arm->extend_cali_obj.state = CALI_RUNNING;
                grab->arm->extend_cali_obj.internal_step = 2; // 直接跳到阶段 2 往回收
                grab->arm->extend_cali_obj.timeout_cnt = 0;
            }
        }
    }
    last_climb_mode = grab_ctrl_cmd->is_climb_mode;

    last_climb_mode = grab_ctrl_cmd->is_climb_mode; // 记录状态供下次比较
    // =========================================================
    if (grab_ctrl_cmd->elbow_pitch > grab_param.elbow_pitch_max)
        grab_ctrl_cmd->elbow_pitch = grab_param.elbow_pitch_max;
    else if (grab_ctrl_cmd->elbow_pitch < grab_param.elbow_pitch_min)
        grab_ctrl_cmd->elbow_pitch = grab_param.elbow_pitch_min;

    if (grab_ctrl_cmd->base_joint > grab_param.base_joint_max)
        grab_ctrl_cmd->base_joint = grab_param.base_joint_max;
    else if (grab_ctrl_cmd->base_joint < grab_param.base_joint_min)
        grab_ctrl_cmd->base_joint = grab_param.base_joint_min;

    if (grab_ctrl_cmd->elbow_roll > grab_param.elbow_roll_max)
        grab_ctrl_cmd->elbow_roll = grab_param.elbow_roll_max;
    else if (grab_ctrl_cmd->elbow_roll < grab_param.elbow_roll_min)
        grab_ctrl_cmd->elbow_roll = grab_param.elbow_roll_min;

    grab->arm->base_joint = grab_ctrl_cmd->base_joint;
    grab->arm->elbow_roll = grab_ctrl_cmd->elbow_roll;
    grab->arm->elbow_pitch = grab_ctrl_cmd->elbow_pitch;
    grab->actuator->wrist_pitch = grab_ctrl_cmd->wrist_pitch;
    grab->actuator->wrist_roll = grab_ctrl_cmd->wrist_roll;
    grab->actuator->gripper_state = grab_ctrl_cmd->gripper_state;
    grab->arm->arm_lift = grab_ctrl_cmd->arm_lift;
    grab->grab_ctrl_cmd.arm_extend = grab_ctrl_cmd->arm_extend;
}

static void MotorTask()
{
    Error_Check();
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);
        DJIMotorStop(grab->arm->arm_lift_motor);
        DJIMotorStop(grab->arm->arm_extend_motor);
        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DJIMotorStop(grab->actuator->grab_djimotor[2]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);
    }
    else
    {
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

        for (int i = 0; i < 3; i++)
        {
            if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
            {
                DJIMotorEnable(grab->arm->arm_lift_motor);
                DJIMotorSetPIDRef(grab->arm->arm_lift_motor, grab->grab_ctrl_cmd.arm_lift_target);
            }
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
                case 0:
                    if (grab_param.use_wrist_right_motor)
                        DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->R_target);
                    else
                        DJIMotorStop(grab->actuator->grab_djimotor[i]);
                    break;
                case 1:
                    if (grab_param.use_wrist_left_motor)
                        DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->L_target);
                    else
                        DJIMotorStop(grab->actuator->grab_djimotor[i]);
                    break;
                case 2:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->M_target);
                    break;
                }
            }
        }

        if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
        {
            DMMotorEnable(grab->actuator->grab_dmmotor[0]);
            DMMotorSetRef(grab->actuator->grab_dmmotor[0], grab->actuator->T_target);
        }
    }
}

static void Grab_Position_Calculate(GrabInstance *grab)
{
    grab->actuator->R_target = total_angle_init_R + grab->actuator->wrist_pitch * grab_param.motor2006_reduction_ratio *
                                                        grab_param.pulley_gear_ratio;
    grab->actuator->L_target = total_angle_init_L - grab->actuator->wrist_pitch * grab_param.motor2006_reduction_ratio *
                                                        grab_param.pulley_gear_ratio;
    grab->actuator->M_target = total_angle_init_M + grab->actuator->wrist_roll * grab_param.motor2006_reduction_ratio *
                                                        grab_param.planar_gear_ratio;

    grab->grab_ctrl_cmd.arm_lift_target =
        total_angle_init_arm_lift + grab_ctrl_cmd->arm_lift * grab_param.motor3508_p51_reduction_ratio;
    grab->grab_ctrl_cmd.arm_extend_target =
        total_angle_init_arm_extend + grab_ctrl_cmd->arm_extend * grab_param.motor3508_p19_reduction_ratio;

    if (grab->actuator->gripper_state == GRIPPER_CLOSE)
    {
        grab->actuator->T_target = grab_param.gripper_close_torque;
    }
    else
    {
        grab->actuator->T_target = grab_param.gripper_open_torque;
    }
}

static void Grab_Real_Angle_Calculate(GrabInstance *grab)
{
    if (DaemonIsOnline(grab->arm->grab_dmmotor[0]->daemon))
        grab->grab_measure.base_joint = grab->arm->grab_dmmotor[0]->measure.total_angle * RAD_2_DEGREE;
    if (DaemonIsOnline(grab->arm->grab_dmmotor[1]->daemon))
        grab->grab_measure.elbow_roll = grab->arm->grab_dmmotor[1]->measure.total_angle * RAD_2_DEGREE;
    if (DaemonIsOnline(grab->arm->grab_dmmotor[2]->daemon))
        grab->grab_measure.elbow_pitch = grab->arm->grab_dmmotor[2]->measure.total_angle * RAD_2_DEGREE;

    if (grab->actuator->wrist_cali_obj.state == CALI_DONE)
    {
        float ratio_pitch = grab_param.motor2006_reduction_ratio * grab_param.pulley_gear_ratio;
        float ratio_roll = grab_param.motor2006_reduction_ratio * grab_param.planar_gear_ratio;
        float curr_r = grab->actuator->grab_djimotor[0]->measure.total_angle;
        float curr_l = grab->actuator->grab_djimotor[1]->measure.total_angle;
        float curr_m = grab->actuator->grab_djimotor[2]->measure.total_angle;

        if (grab_param.use_wrist_left_motor)
        {
            grab->grab_measure.wrist_pitch = -(curr_l - total_angle_init_L) / ratio_pitch;
        }
        else if (grab_param.use_wrist_right_motor)
        {
            grab->grab_measure.wrist_pitch = (curr_r - total_angle_init_R) / ratio_pitch;
        }
        grab->grab_measure.wrist_roll = (curr_m - total_angle_init_M) / ratio_roll;
    }

    if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
        grab->grab_measure.torque = grab->actuator->grab_dmmotor[0]->measure.torque;
    if (DaemonIsOnline(grab->arm->arm_lift_motor->daemon))
    {
        float curr_lift = grab->arm->arm_lift_motor->measure.total_angle;
        grab->grab_measure.arm_lift =
            (curr_lift - total_angle_init_arm_lift) / grab_param.motor3508_p51_reduction_ratio;
    }
    if (DaemonIsOnline(grab->arm->arm_extend_motor->daemon))
    {
        float curr_extend = grab->arm->arm_extend_motor->measure.total_angle;
        grab->grab_measure.arm_extend =
            (curr_extend - total_angle_init_arm_extend) / grab_param.motor3508_p19_reduction_ratio;
    }
    if (grab->arm->micro_switch_gpio != NULL)
    {
        grab->grab_measure.micro_switch_state = GPIORead(grab->arm->micro_switch_gpio);
    }
}

static void Error_Check()
{
}

void GrabClearError(void)
{
    if (grab != NULL)
        grab->error_code = GRAB_NO_ERROR;
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
        grab->actuator->wrist_cali_obj.internal_step = 0;
        grab->actuator->wrist_cali_obj.state = CALI_RUNNING;
        cali_first_run = 1;
        grab_ctrl_cmd->wrist_pitch_cali = 0;
    }
}