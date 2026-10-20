#include "shoot.h"

#include "cmsis_os.h"
#include "daemon.h"
#include "user_lib.h"
#include <math.h>

static ShootInstance *shoot;

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
        // 关闭时保留原来的零速制动方式；总开关不能留下上一周期输出。
        DJIMotorEnable(shoot->friction_motor[i]);
        DJIMotorSetPIDRef(shoot->friction_motor[i], shoot->friction_coefficients[i] * shoot->friction_set);
    }
}

static void LoaderStop(void)
{
    DMMotorStop(shoot->loader_motor);
    if (shoot->last_load_mode != LOAD_STOP)
    {
        PIDClear(&shoot->loader_motor->motor_controller.angle_PID);
        PIDClear(&shoot->loader_motor->motor_controller.speed_PID);
    }
    shoot->loader_target = shoot->loader_motor->measure.total_angle;
    shoot->loader_speed_feedforward = 0.0f;
    shoot->last_load_mode = LOAD_STOP;
}

static void LoaderControl(void)
{
    const Shoot_Ctrl_Cmd_s *cmd = &shoot->shoot_ctrl_cmd;
    DMMotorInstance *motor = shoot->loader_motor;
    uint32_t period = cmd->load_period_ms != 0 ? cmd->load_period_ms : shoot->loader_period_ms;
    // 只保留命令联动，不检查摩擦轮到速；拨弹盘自己离线时不继续追赶旧目标。
    if (cmd->shoot_mode != SHOOT_ON || cmd->friction_mode != FRICTION_ON || shoot->friction_set == 0.0f ||
        cmd->load_mode < LOAD_1_BULLET || cmd->load_mode > LOAD_REVERSE || period == 0 ||
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
        shoot->last_load_tick = now - period; // 进入模式时立即允许第一步。
        PIDClear(&motor->motor_controller.angle_PID);
        PIDClear(&motor->motor_controller.speed_PID);
    }

    float direction = cmd->load_mode == LOAD_REVERSE ? -1.0f : 1.0f;
    uint8_t at_target = fabsf(shoot->loader_target - motor->measure.total_angle) <= shoot->loader_angle_tolerance;
    // 单发只响应模式进入；连发和反转必须等上一步到位，不无限累积目标。
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
    DMMotorEnable(motor);
    DMMotorSetPIDRef(motor, shoot->loader_target);
    shoot->last_load_mode = cmd->load_mode;
}

void ShootTask(void)
{
    FrictionControl();
    LoaderControl();
}
