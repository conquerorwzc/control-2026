#include "shoot.h"

#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "user_lib.h"
#include <math.h>

static ShootInstance *shoot;
static PID_Init_Config_s loader_step_pid;
static PID_Init_Config_s loader_burst_pid;

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

static void LoaderStop(void)
{
    DMMotorStop(shoot->loader_motor);
    DMMotorSetRef(shoot->loader_motor, 0.0f);
    if (shoot->last_load_mode != LOAD_STOP)
    {
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
