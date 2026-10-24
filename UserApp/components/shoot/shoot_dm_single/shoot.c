#include "shoot.h"

#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "user_lib.h"
#include <math.h>

static ShootInstance *shoot;

typedef enum
{
    LOADER_NORMAL = 0, // 正常拨弹，尚无有效的受阻检测窗口。
    LOADER_CHECK,      // 受阻条件连续成立，累计时间和角度范围；仍执行正常拨弹。
    LOADER_BACK        // 已确认卡弹，暂时接管输出并追踪固定退弹目标。
} Loader_State_e;

static Loader_State_e loader_state;

// 非零指令按幅值限制在有效转速范围内，保留原旋转方向；零指令用于关闭。
static float LimitFrictionSpeed(float speed)
{
    if (speed == 0.0f)
        return 0.0f;
    float magnitude = fabsf(speed);
    if (magnitude < shoot->config->friction_speed_min)
        magnitude = shoot->config->friction_speed_min;
    if (magnitude > shoot->config->friction_speed_max)
        magnitude = shoot->config->friction_speed_max;
    return copysignf(magnitude, speed);
}

ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config)
{
    shoot = (ShootInstance *)zmalloc(sizeof(ShootInstance));
    // 固定参数只保存引用，配置对象需在整个发射任务期间保持有效。
    shoot->config = shoot_init_config;
    for (int i = 0; i < FRICTION_NUM; ++i)
    {
        shoot->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
        DJIMotorStop(shoot->friction_motor[i]);
    }
    shoot->loader_motor = DMMotorInit(&shoot_init_config->loader_motor_config);
    // 当前 DM 驱动没有复制此指针；存储属于实例，生命周期覆盖整个任务。
    shoot->loader_motor->motor_controller.speed_feedforward_ptr = &shoot->loader_speed_feedforward;
    DMMotorStop(shoot->loader_motor);
    return shoot;
}

static void FrictionControl(void)
{
    Shoot_Ctrl_Cmd_s *cmd = &shoot->shoot_ctrl_cmd;
    shoot->friction_set = cmd->shoot_mode == SHOOT_ON && cmd->friction_mode == FRICTION_ON
                              ? LimitFrictionSpeed(cmd->friction_speed)
                              : 0.0f;
    for (int i = 0; i < FRICTION_NUM; ++i)
    {
        DJIMotorInstance *motor = shoot->friction_motor[i];
        if (shoot->friction_set == 0.0f || !DaemonIsOnline(motor->daemon) || motor->motor_can_instance->rx_len != 8U)
        {
            // 关闭或反馈掉线时发送零电流，不再用旧反馈计算零速制动。
            DJIMotorStop(motor);
            motor->motor_controller.pid_ref = 0.0f;
            DJIMotorSetRef(motor, 0.0f);
            PIDClear(&motor->motor_controller.speed_PID);
            continue;
        }
        // 先更新输出再使能，避免恢复时发出上一周期残留的电流。
        DJIMotorSetPIDRef(motor, shoot->config->friction_coefficients[i] * shoot->friction_set);
        DJIMotorEnable(motor);
    }
}

// 统一复位动作状态并设置正常单发/连发参数；退弹只在进入时额外缩小两个输出限幅。
static void LoaderMode(Load_Mode_e mode, uint32_t now)
{
    DMMotorInstance *motor = shoot->loader_motor;
    // 停止、模式切换和退弹恢复共用此入口；下一次合格采样会重新建立检测窗口。
    loader_state = LOADER_NORMAL;
    // 连发直接用速度 PI；单发及自动退弹用原位置双环和对应的速度 PI。
    PIDSwitchConfig(&motor->motor_controller.speed_PID,
                    mode == LOAD_BURSTFIRE
                        ? &shoot->config->loader_burst_pid
                        : &shoot->config->loader_motor_config.controller_param_init_config.speed_PID);
    motor->motor_controller.angle_PID.MaxOut =
        shoot->config->loader_motor_config.controller_param_init_config.angle_PID.MaxOut;
    DMMotorOuterLoop(motor, mode == LOAD_BURSTFIRE ? SPEED_LOOP : ANGLE_LOOP);
    PIDClear(&motor->motor_controller.angle_PID);
    PIDClear(&motor->motor_controller.speed_PID);
    // Ref 同时保存连发斜坡状态，必须在清 PID 后重建。
    motor->motor_controller.speed_PID.Ref = mode == LOAD_BURSTFIRE ? motor->measure.velocity : 0.0f;
    shoot->loader_speed_feedforward = 0.0f;
    shoot->loader_torque_feedforward = 0.0f;
    // 按阶段作为单发调度、连发斜坡或退弹开始的时间基准。
    shoot->last_load_tick = now;
}

// 返回 1 表示状态机已接管本周期输出；speed 为正常拨弹的目标平均角速度。
static uint8_t LoaderJam(float speed, uint32_t now)
{
    // CHECK 使用独立计时；resume_angle 保存退弹前尚未完成的单发目标。
    static uint32_t start_tick;
    static float angle_min, angle_max, resume_angle;
    DMMotorInstance *motor = shoot->loader_motor;
    Load_Mode_e mode = shoot->shoot_ctrl_cmd.load_mode;
    float angle = motor->measure.total_angle;
    switch (loader_state)
    {
    case LOADER_NORMAL:
    case LOADER_CHECK:
    {
        uint8_t single = mode == LOAD_1_BULLET;
        // 单发只在仍需正向推进时检测；连发等斜坡达到目标速度一半后再检测。
        uint8_t active = single ? motor->motor_controller.pid_ref - angle > shoot->config->loader_angle_tolerance
                                : motor->motor_controller.speed_PID.Ref >= 0.5f * speed;
        // 慢速单发按目标平均速度缩小低速阈值，避免将正常缓慢推进判成卡弹。
        float low_speed = single ? fminf(1.0f, 0.2f * speed) : 1.0f;
        // 必须同时满足：正在推进、实际速度低、反馈正向力矩足够大；任一中断则重新计时。
        if (!active || fabsf(motor->measure.velocity) >= low_speed || motor->measure.torque < 1.5f)
        {
            loader_state = LOADER_NORMAL;
            return 0;
        }
        if (loader_state == LOADER_NORMAL)
        {
            // 从首个合格采样开始计时，不计入此前未满足条件的时间。
            start_tick = now;
            angle_min = angle_max = angle;
            loader_state = LOADER_CHECK;
            return 0;
        }
        angle_min = fminf(angle_min, angle);
        angle_max = fmaxf(angle_max, angle);
        if (angle_max - angle_min > 5.0f * DEGREE_2_RAD)
        {
            // 已移动超过候选范围，说明并非持续停滞；从当前角度重新累计窗口。
            start_tick = now;
            angle_min = angle_max = angle;
            return 0;
        }
        // 无符号时间差兼容毫秒计数回绕；连续满足 80 ms 才确认卡弹。
        if ((uint32_t)(now - start_tick) < 80U)
            return 0;

        // 仅在确认瞬间设置一次实际角度减 18°，后续周期不再累加退角。
        resume_angle = motor->motor_controller.pid_ref;
        LoaderMode(LOAD_1_BULLET, now);
        motor->motor_controller.pid_ref = angle - 18.0f * DEGREE_2_RAD;
        motor->motor_controller.angle_PID.MaxOut = 4.0f;
        motor->motor_controller.speed_PID.MaxOut = 3.0f;
        loader_state = LOADER_BACK;
    }
        // 确认当周期直接执行退弹输出，不额外等待一个控制周期。
    case LOADER_BACK:
        // 到位只看角度误差；即使未到位，满 200 ms 也立即恢复原模式。
        if (fabsf(motor->motor_controller.pid_ref - angle) < 3.0f * DEGREE_2_RAD ||
            (uint32_t)(now - shoot->last_load_tick) >= 200U)
        {
            LoaderMode(mode, now);
            // 单发继续完成原来的那一颗，避免退弹后再额外追加一个齿距。
            if (mode == LOAD_1_BULLET)
                motor->motor_controller.pid_ref = resume_angle;
            return 0; // 同周期恢复原发射模式，不等待、不追加单发目标。
        }
        DMMotorSetPIDRef(motor, motor->motor_controller.pid_ref);
        DMMotorEnable(motor);
        return 1;
    }
    return 0;
}

static void LoaderStop(void)
{
    DMMotorInstance *motor = shoot->loader_motor;
    uint32_t now = osKernelSysTick();
    DMMotorStop(motor);
    DMMotorSetRef(motor, 0.0f);
    // 仅首次进入停止时恢复参数并取消卡弹动作，保持停止时不重复清 PID。
    if (shoot->last_load_mode != LOAD_STOP)
        LoaderMode(shoot->last_load_mode, now);
    // 丢弃旧位置目标及斜坡参考，下次启动从实际反馈重新建立参考。
    motor->motor_controller.pid_ref = motor->measure.total_angle;
    motor->motor_controller.speed_PID.Ref = 0.0f;
    shoot->last_load_mode = LOAD_STOP;
}

// 目前仅用于连发：按实际拨盘角度所在的齿相位补偿阻力，返回力矩，单位 N*m。
static float LoaderFeedforward(void)
{
    // 将接触角前 1° 作为周期起点，避免上升段跨过取模边界。
    float phase = fmodf(shoot->loader_motor->measure.total_angle - shoot->config->loader_contact_angle +
                            1.0f * DEGREE_2_RAD,
                        shoot->config->loader_step_angle);
    if (phase < 0.0f)
        phase += shoot->config->loader_step_angle;
    // 接触角为 0°，当前 30° 齿距对应相位区间 [-1°, 29°)。
    phase = phase * RAD_2_DEGREE - 1.0f;

    float blend;
    if (phase < 6.0f)
        blend = (phase + 1.0f) / 7.0f;
    else if (phase <= 16.0f)
        return 3.6f;
    else if (phase < 19.0f)
        blend = (19.0f - phase) / 3.0f;
    else
        return 1.1f;
    // smoothstep：低阻 1.1 N*m，高阻 3.6 N*m，-1°~+6° 上升，+16°~+19° 下降。
    return 1.1f + 2.5f * blend * blend * (3.0f - 2.0f * blend);
}

static void LoaderBurst(float speed, uint32_t now)
{
    DMMotorInstance *motor = shoot->loader_motor;
    float change = 60.0f * (float)(uint32_t)(now - shoot->last_load_tick) * 0.001f; // rad/s，60 rad/s² 斜坡。
    shoot->last_load_tick = now;
    // 上次 PI 的 Ref 同时作为斜坡记忆值，每周期最多变化 change。
    float speed_ref = motor->motor_controller.speed_PID.Ref;
    speed_ref += fmaxf(-change, fminf(change, speed - speed_ref));
    shoot->loader_torque_feedforward = LoaderFeedforward();
    float torque = PIDCalculate(&motor->motor_controller.speed_PID, motor->measure.velocity, speed_ref);
    torque += shoot->loader_torque_feedforward; // 仅补偿阻力，不加入加速度前馈。
    // PI 与前馈相加后再限制最终力矩；连发在此计算输出，直接交给电机发送。
    LIMIT_MIN_MAX(torque, -motor->motor_controller.speed_PID.MaxOut, motor->motor_controller.speed_PID.MaxOut);
    DMMotorSetRef(motor, torque);
    DMMotorEnable(motor);
}

static void LoaderControl(void)
{
    const Shoot_Ctrl_Cmd_s *cmd = &shoot->shoot_ctrl_cmd;
    DMMotorInstance *motor = shoot->loader_motor;
    // 单发、连发命令直接指定正周期，单位为 ms；停止命令不使用周期。
    uint32_t period = cmd->load_period_ms;
    uint32_t now = osKernelSysTick();
    // 联动、停止与反馈检查统一放在入口，不增加摩擦轮到速判定。
    if (shoot->friction_set == 0.0f || cmd->load_mode == LOAD_STOP ||
        !DaemonIsOnline(motor->daemon) || motor->motor_can_instance->rx_len != 8U)
    {
        LoaderStop();
        return;
    }

    // 一齿角度 / 周期得到平均角速度（rad/s），供检测、连发和单发速度前馈共用。
    float speed = shoot->config->loader_step_angle * 1000.0f / (float)period;
    if (cmd->load_mode != shoot->last_load_mode)
    {
        // 外部模式改变会取消旧退弹动作，并从实际位置进入新模式。
        LoaderMode(cmd->load_mode, now);
        motor->motor_controller.pid_ref = motor->measure.total_angle;
        if (cmd->load_mode == LOAD_1_BULLET)
            shoot->last_load_tick = now - period; // 进入单发立即启动第一颗。
    }

    // CHECK 不接管输出；BACK 才跳过正常控制，回退结束当周期可立即恢复。
    if (!LoaderJam(speed, now))
    {
        if (cmd->load_mode == LOAD_BURSTFIRE)
            LoaderBurst(speed, now);
        else
        {
            float error = motor->motor_controller.pid_ref - motor->measure.total_angle;
            // 单发需同时到位且周期已过才拨下一颗
            if (fabsf(error) <= shoot->config->loader_angle_tolerance &&
                (uint32_t)(now - shoot->last_load_tick) >= period)
            {
                // 在原目标上加一齿，避免容差范围内的实际位置误差逐次造成相位漂移。
                motor->motor_controller.pid_ref += shoot->config->loader_step_angle;
                shoot->last_load_tick = now;
                error = motor->motor_controller.pid_ref - motor->measure.total_angle;
            }
            // 只在正向未到位时叠加速度前馈；到位或超调时清零，不添加单发力矩前馈。
            shoot->loader_speed_feedforward = error > shoot->config->loader_angle_tolerance ? speed : 0.0f;
            DMMotorSetPIDRef(motor, motor->motor_controller.pid_ref);
            DMMotorEnable(motor);
        }
    }
    // 只记录外部命令模式，自动退弹状态单独由 loader_state 表示。
    shoot->last_load_mode = cmd->load_mode;
}

void ShootTask(void)
{
    // 先计算摩擦轮指令，拨弹入口复用 friction_set 判断发射联动是否允许。
    FrictionControl();
    LoaderControl();
}
