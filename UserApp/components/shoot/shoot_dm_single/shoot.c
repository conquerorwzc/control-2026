#include "shoot.h"

#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "user_lib.h"
#include <float.h>
#include <math.h>

static ShootInstance *shoot;
static PID_Init_Config_s loader_step_pid;
static PID_Init_Config_s loader_burst_pid;
static float loader_angle_max_out;

static float LimitFrictionSpeed(float speed)
{
    if (speed == 0.0f)
        return 0.0f;
    float magnitude = fabsf(speed);
    if (magnitude < shoot->friction_speed_min)
        magnitude = shoot->friction_speed_min;
    if (magnitude > shoot->friction_speed_max)
        magnitude = shoot->friction_speed_max;
    return copysignf(magnitude, speed);
}

ShootInstance *ShootInit(Shoot_Init_Config_s *shoot_init_config)
{
    shoot = (ShootInstance *)zmalloc(sizeof(ShootInstance));
    for (int i = 0; i < FRICTION_NUM; ++i)
    {
        shoot->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
        shoot->friction_coefficients[i] = shoot_init_config->friction_coefficients[i];
        DJIMotorStop(shoot->friction_motor[i]);
    }
    shoot->friction_speed_min = shoot_init_config->friction_speed_min;
    shoot->friction_speed_max = shoot_init_config->friction_speed_max;
    shoot->loader_step_angle = shoot_init_config->loader_step_angle;
    shoot->loader_angle_tolerance = shoot_init_config->loader_angle_tolerance;
    shoot->loader_period_ms = shoot_init_config->loader_period_ms;
    shoot->loader_contact_angle = shoot_init_config->loader_contact_angle;
    loader_step_pid = shoot_init_config->loader_motor_config.controller_param_init_config.speed_PID;
    loader_burst_pid = shoot_init_config->loader_burst_pid;
    loader_angle_max_out = shoot_init_config->loader_motor_config.controller_param_init_config.angle_PID.MaxOut;
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
        DJIMotorSetPIDRef(motor, shoot->friction_coefficients[i] * shoot->friction_set);
        DJIMotorEnable(motor);
    }
}

// period 为当前连发周期；传入 0 清除卡弹标志及检测窗口。
static uint8_t LoaderJam(uint32_t period, uint32_t now)
{
    static uint8_t jammed;
    static uint32_t start_tick;
    static float angle_min = FLT_MAX, angle_max = -FLT_MAX;
    DMMotorInstance *motor = shoot->loader_motor;
    float angle = motor->measure.total_angle;
    if (period == 0)
    {
        jammed = 0;
        start_tick = now;
        angle_min = FLT_MAX;
        angle_max = -FLT_MAX;
        return 0;
    }
    if (jammed)
        return 1;

    float speed = shoot->loader_step_angle * 1000.0f / (float)period;
    if (shoot->loader_speed_ref < 0.5f * speed || fabsf(motor->measure.velocity) >= 1.0f ||
        motor->measure.torque < 1.5f)
    {
        start_tick = now;
        angle_min = FLT_MAX;
        angle_max = -FLT_MAX;
        return 0;
    }
    if (angle_min > angle_max) // 首个满足条件的采样点开始计时，不另设候选状态。
        start_tick = now;
    angle_min = fminf(angle_min, angle);
    angle_max = fmaxf(angle_max, angle);
    if (angle_max - angle_min > 5.0f * DEGREE_2_RAD)
    {
        start_tick = now;
        angle_min = angle_max = angle;
    }
    if ((uint32_t)(now - start_tick) >= 80U)
        jammed = 1;
    return jammed;
}

static void LoaderStop(void)
{
    DMMotorStop(shoot->loader_motor);
    DMMotorSetRef(shoot->loader_motor, 0.0f);
    LoaderJam(0, osKernelSysTick());
    shoot->loader_motor->motor_controller.angle_PID.MaxOut = loader_angle_max_out;
    if (shoot->last_load_mode != LOAD_STOP)
    {
        PIDSwitchConfig(&shoot->loader_motor->motor_controller.speed_PID,
                        shoot->last_load_mode == LOAD_BURSTFIRE ? &loader_burst_pid : &loader_step_pid);
        DMMotorOuterLoop(shoot->loader_motor, shoot->last_load_mode == LOAD_BURSTFIRE ? SPEED_LOOP : ANGLE_LOOP);
        PIDClear(&shoot->loader_motor->motor_controller.angle_PID);
        PIDClear(&shoot->loader_motor->motor_controller.speed_PID);
    }
    shoot->loader_target = shoot->loader_motor->measure.total_angle;
    shoot->loader_speed_feedforward = 0.0f;
    shoot->loader_speed_ref = 0.0f;
    shoot->loader_torque_feedforward = 0.0f;
    shoot->last_load_mode = LOAD_STOP;
}

static float LoaderFeedforward(void)
{
    // 将接触角前 5° 作为周期起点，避免提前加力段跨过取模边界。
    float phase = fmodf(shoot->loader_motor->measure.total_angle - shoot->loader_contact_angle + 5.0f * DEGREE_2_RAD,
                        shoot->loader_step_angle);
    if (phase < 0.0f)
        phase += shoot->loader_step_angle;
    phase = phase * RAD_2_DEGREE - 5.0f;

    float blend;
    if (phase < -3.0f)
        blend = (phase + 5.0f) / 2.0f;
    else if (phase <= 4.0f)
        return 3.0f;
    else if (phase < 6.0f)
        blend = (6.0f - phase) / 2.0f;
    else
        return 1.0f;
    // smoothstep：低阻 1 N*m，高阻 3 N*m，-5°~-3° 上升，+4°~+6° 下降。
    return 1.0f + 2.0f * blend * blend * (3.0f - 2.0f * blend);
}

static void LoaderBurst(uint32_t period, uint32_t now)
{
    DMMotorInstance *motor = shoot->loader_motor;
    float speed = shoot->loader_step_angle * 1000.0f / (float)period;
    float change = 60.0f * (float)(uint32_t)(now - shoot->last_load_tick) * 0.001f; // rad/s，60 rad/s² 斜坡。
    shoot->last_load_tick = now;
    shoot->loader_speed_ref += fmaxf(-change, fminf(change, speed - shoot->loader_speed_ref));
    shoot->loader_torque_feedforward = LoaderFeedforward();
    float torque = PIDCalculate(&motor->motor_controller.speed_PID, motor->measure.velocity, shoot->loader_speed_ref);
    torque += shoot->loader_torque_feedforward; // 仅补偿阻力，不加入加速度前馈。
    LIMIT_MIN_MAX(torque, -motor->motor_controller.speed_PID.MaxOut, motor->motor_controller.speed_PID.MaxOut);
    DMMotorSetRef(motor, torque);
    DMMotorEnable(motor);
}

static uint8_t LoaderBack(uint32_t now)
{
    DMMotorInstance *motor = shoot->loader_motor;
    // 正常连发为速度外环，切到位置外环时只锁定一次本次回退目标。
    if (motor->motor_settings.outer_loop_type != ANGLE_LOOP)
    {
        shoot->loader_target = motor->measure.total_angle - 18.0f * DEGREE_2_RAD;
        shoot->last_load_tick = now;
        shoot->loader_speed_ref = 0.0f;
        shoot->loader_speed_feedforward = 0.0f;
        shoot->loader_torque_feedforward = 0.0f;
        PIDSwitchConfig(&motor->motor_controller.speed_PID, &loader_step_pid);
        motor->motor_controller.angle_PID.MaxOut = 4.0f;
        motor->motor_controller.speed_PID.MaxOut = 3.0f;
        PIDClear(&motor->motor_controller.angle_PID);
        PIDClear(&motor->motor_controller.speed_PID);
        DMMotorOuterLoop(motor, ANGLE_LOOP);
    }

    if (fabsf(shoot->loader_target - motor->measure.total_angle) < 3.0f * DEGREE_2_RAD ||
        (uint32_t)(now - shoot->last_load_tick) >= 200U)
    {
        motor->motor_controller.angle_PID.MaxOut = loader_angle_max_out;
        PIDSwitchConfig(&motor->motor_controller.speed_PID, &loader_burst_pid);
        PIDClear(&motor->motor_controller.angle_PID);
        PIDClear(&motor->motor_controller.speed_PID);
        DMMotorOuterLoop(motor, SPEED_LOOP);
        shoot->loader_speed_ref = motor->measure.velocity;
        shoot->last_load_tick = now; // 恢复斜坡不累计回退期间的时间。
        LoaderJam(0, now);
        return 1;
    }
    DMMotorSetPIDRef(motor, shoot->loader_target);
    DMMotorEnable(motor);
    return 0;
}

static void LoaderControl(void)
{
    const Shoot_Ctrl_Cmd_s *cmd = &shoot->shoot_ctrl_cmd;
    DMMotorInstance *motor = shoot->loader_motor;
    uint32_t period = cmd->load_period_ms != 0 ? cmd->load_period_ms : shoot->loader_period_ms;
    // 只保留命令联动，不检查摩擦轮到速；拨弹盘自己离线时不继续追赶旧目标。
    if (cmd->shoot_mode != SHOOT_ON || cmd->friction_mode != FRICTION_ON || shoot->friction_set == 0.0f ||
        cmd->load_mode < LOAD_1_BULLET || cmd->load_mode > LOAD_STEP || period == 0 ||
        shoot->loader_step_angle <= 0.0f || !DaemonIsOnline(motor->daemon) || motor->motor_can_instance->rx_len != 8U)
    {
        LoaderStop();
        return;
    }

    uint32_t now = osKernelSysTick();
    uint8_t mode_changed = cmd->load_mode != shoot->last_load_mode;
    if (mode_changed)
    {
        LoaderJam(0, now);
        motor->motor_controller.angle_PID.MaxOut = loader_angle_max_out;
        shoot->loader_target = motor->measure.total_angle;
        shoot->last_load_tick = cmd->load_mode == LOAD_BURSTFIRE ? now : now - period;
        shoot->loader_speed_ref = cmd->load_mode == LOAD_BURSTFIRE ? motor->measure.velocity : 0.0f;
        shoot->loader_speed_feedforward = 0.0f;
        shoot->loader_torque_feedforward = 0.0f;
        PIDSwitchConfig(&motor->motor_controller.speed_PID,
                        cmd->load_mode == LOAD_BURSTFIRE ? &loader_burst_pid : &loader_step_pid);
        DMMotorOuterLoop(motor, cmd->load_mode == LOAD_BURSTFIRE ? SPEED_LOOP : ANGLE_LOOP);
        PIDClear(&motor->motor_controller.angle_PID);
        PIDClear(&motor->motor_controller.speed_PID);
    }

    if (cmd->load_mode == LOAD_BURSTFIRE)
    {
        if (!LoaderJam(period, now) || LoaderBack(now))
            LoaderBurst(period, now);
        shoot->last_load_mode = cmd->load_mode;
        return;
    }

    float direction = cmd->load_mode == LOAD_REVERSE ? -1.0f : 1.0f;
    uint8_t at_target = fabsf(shoot->loader_target - motor->measure.total_angle) <= shoot->loader_angle_tolerance;
    // 单发只响应模式进入；慢档和反转必须等上一步到位，不无限累积目标。
    if ((cmd->load_mode == LOAD_1_BULLET && mode_changed) ||
        (cmd->load_mode != LOAD_1_BULLET && at_target && (uint32_t)(now - shoot->last_load_tick) >= period))
    {
        shoot->loader_target += direction * shoot->loader_step_angle;
        shoot->last_load_tick = now;
    }
    float error = shoot->loader_target - motor->measure.total_angle;
    shoot->loader_speed_feedforward = error * direction > shoot->loader_angle_tolerance
                                          ? direction * shoot->loader_step_angle * 1000.0f / (float)period
                                          : 0.0f;
    DMMotorSetPIDRef(motor, shoot->loader_target);
    DMMotorEnable(motor);
    shoot->last_load_mode = cmd->load_mode;
}

void ShootTask(void)
{
    FrictionControl();
    LoaderControl();
}
