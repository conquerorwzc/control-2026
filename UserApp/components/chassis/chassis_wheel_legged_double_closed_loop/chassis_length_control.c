/**
 ******************************************************************************
 * @file    chassis_length_control.c
 * @brief   双闭环轮腿单腿腿长 PID 纯计算实现
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_length_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* 腿长 PID 允许使用的通用 PID 优化位；D 项必须只使用 Jacobian 的 l_dot。 */
#define WHEEL_LEGGED_LENGTH_CONTROL_ALLOWED_PID_IMPROVE (PID_Integral_Limit | PID_Trapezoid_Intergral)

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t WheelLeggedLegLengthControlHasValidParameter(const WheelLeggedLegLengthControl_t *control,
                                                            float measured_length, float measured_length_dot);
static uint8_t WheelLeggedLegLengthControlPidConfigChanged(const WheelLeggedLegLengthControl_t *control);
static void WheelLeggedLegLengthControlSynchronizePidConfig(WheelLeggedLegLengthControl_t *control);
static void WheelLeggedLegLengthControlReset(WheelLeggedLegLengthControl_t *control);
static float WheelLeggedLegLengthControlLimitForce(float force, float force_limit);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化一条腿的通用 PID 配置和运行态。
 *
 * PID 初始化参数只来自 robot 层；component 不引用具体车辆的 robot_config.h。
 *
 * @param control 本腿独立的腿长 PID 状态。
 * @param config robot 层提供的本腿腿长 PID 初始化配置。
 */
void WheelLeggedLegLengthControlInit(WheelLeggedLegLengthControl_t *control,
                                     const WheelLeggedLegLengthControlInitConfig_t *config)
{
    if (control == NULL || config == NULL)
    {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->force_pid_config = config->force_pid_config;
    control->velocity_damping_kd = config->velocity_damping_kd;
    control->length_reference = config->length_reference;
    control->minimum_length = config->minimum_length;
    control->maximum_length = config->maximum_length;
    PIDInit(&control->force_pid, &control->force_pid_config);
}

/**
 * @brief 根据同一帧 FK 腿长和腿长速度计算一条腿的 PID 虚拟轴向力。
 *
 * 通用 PID 只计算 P/I：F_PI=PIDCalculate(l,l_ref)。D 项使用机构学实时给出的
 * l_dot，即 F_PID=F_PI-Kd_leg*l_dot；再与底盘层给出的重力前馈相加并限幅。
 * 本函数不访问电机、不访问 VMC，
 * 也不发送任何控制命令。
 *
 * @param control 本腿独立的腿长 PID 状态。
 * @param feedback_valid 当前 FK、雅可比和关节反馈均有效时为 1。
 * @param measured_length 当前真实虚拟腿长，单位 m。
 * @param measured_length_dot 当前真实虚拟腿长速度，单位 m/s。
 * @param gravity_feedforward 当前底盘层计算的本腿重力前馈，单位 N。
 */
void WheelLeggedLegLengthControlUpdate(WheelLeggedLegLengthControl_t *control, uint8_t feedback_valid,
                                       float measured_length, float measured_length_dot, float gravity_feedforward)
{
    float force_pi;
    float force_with_damping;

    if (control == NULL)
    {
        return;
    }

    control->measured_length = isfinite(measured_length) ? measured_length : 0.0f;
    control->measured_length_dot = isfinite(measured_length_dot) ? measured_length_dot : 0.0f;
    control->length_error = isfinite(control->length_reference) && isfinite(measured_length)
                                ? control->length_reference - measured_length
                                : 0.0f;
    if (feedback_valid == 0u || !isfinite(gravity_feedforward) ||
        WheelLeggedLegLengthControlHasValidParameter(control, measured_length, measured_length_dot) == 0u)
    {
        WheelLeggedLegLengthControlReset(control);
        return;
    }

    WheelLeggedLegLengthControlSynchronizePidConfig(control);
    force_pi = PIDCalculate(&control->force_pid, control->measured_length, control->length_reference);
    force_with_damping = force_pi - control->velocity_damping_kd * control->measured_length_dot;
    if (!isfinite(force_pi) || !isfinite(force_with_damping))
    {
        WheelLeggedLegLengthControlReset(control);
        return;
    }

    control->force_pid_command = force_with_damping;
    control->gravity_feedforward = gravity_feedforward;
    control->force_command = WheelLeggedLegLengthControlLimitForce(
        control->force_pid_command + control->gravity_feedforward, control->force_pid_config.MaxOut);
    control->valid = isfinite(control->force_command);
    if (control->valid == 0u)
    {
        WheelLeggedLegLengthControlReset(control);
    }
}

/**
 * @brief 检查腿长 PID 的参数、工作区和当前 FK 测量是否可用于计算。
 *
 * @param control 本腿独立的腿长 PID 状态。
 * @param measured_length 当前真实虚拟腿长，单位 m。
 * @param measured_length_dot 当前真实虚拟腿长速度，单位 m/s。
 * @return 全部合法时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedLegLengthControlHasValidParameter(const WheelLeggedLegLengthControl_t *control,
                                                            float measured_length, float measured_length_dot)
{
    return control != NULL && isfinite(control->length_reference) && isfinite(control->minimum_length) &&
           isfinite(control->maximum_length) && isfinite(control->velocity_damping_kd) &&
           isfinite(control->force_pid_config.Kp) && isfinite(control->force_pid_config.Ki) &&
           isfinite(control->force_pid_config.Kd) && isfinite(control->force_pid_config.MaxOut) &&
           isfinite(control->force_pid_config.DeadBand) && isfinite(control->force_pid_config.IntegralLimit) &&
           isfinite(control->force_pid_config.CoefA) && isfinite(control->force_pid_config.CoefB) &&
           isfinite(control->force_pid_config.Output_LPF_RC) && isfinite(control->force_pid_config.Derivative_LPF_RC) &&
           isfinite(measured_length) && isfinite(measured_length_dot) && control->minimum_length > 0.0f &&
           control->maximum_length > control->minimum_length && control->length_reference >= control->minimum_length &&
           control->length_reference <= control->maximum_length && measured_length >= control->minimum_length &&
           measured_length <= control->maximum_length && control->velocity_damping_kd >= 0.0f &&
           control->force_pid_config.Kp >= 0.0f && control->force_pid_config.Ki >= 0.0f &&
           control->force_pid_config.Kd == 0.0f && control->force_pid_config.MaxOut > 0.0f &&
           control->force_pid_config.DeadBand >= 0.0f && control->force_pid_config.IntegralLimit >= 0.0f &&
           (((uint32_t)control->force_pid_config.Improve & ~WHEEL_LEGGED_LENGTH_CONTROL_ALLOWED_PID_IMPROVE) == 0u);
}

/**
 * @brief 判断变量窗口修改的 PID 配置是否尚未同步到通用 PID 运行实例。
 *
 * @param control 本腿独立的腿长 PID 状态。
 * @return 配置与 PID 实例不一致时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedLegLengthControlPidConfigChanged(const WheelLeggedLegLengthControl_t *control)
{
    return control != NULL && (control->force_pid.Kp != control->force_pid_config.Kp ||
                               control->force_pid.Ki != control->force_pid_config.Ki ||
                               control->force_pid.Kd != control->force_pid_config.Kd ||
                               control->force_pid.MaxOut != control->force_pid_config.MaxOut ||
                               control->force_pid.DeadBand != control->force_pid_config.DeadBand ||
                               control->force_pid.Improve != control->force_pid_config.Improve ||
                               control->force_pid.IntegralLimit != control->force_pid_config.IntegralLimit ||
                               control->force_pid.CoefA != control->force_pid_config.CoefA ||
                               control->force_pid.CoefB != control->force_pid_config.CoefB ||
                               control->force_pid.Output_LPF_RC != control->force_pid_config.Output_LPF_RC ||
                               control->force_pid.Derivative_LPF_RC != control->force_pid_config.Derivative_LPF_RC);
}

/**
 * @brief 将变量窗口更新后的 PID 配置安全同步到通用 PID 运行实例。
 *
 * 配置发生变化时清空积分和历史状态，避免旧参数产生的积分直接带入新参数。
 *
 * @param control 本腿独立的腿长 PID 状态。
 */
static void WheelLeggedLegLengthControlSynchronizePidConfig(WheelLeggedLegLengthControl_t *control)
{
    if (WheelLeggedLegLengthControlPidConfigChanged(control) != 0u)
    {
        PIDInit(&control->force_pid, &control->force_pid_config);
    }
}

/**
 * @brief 重置通用 PID 的积分和历史状态，并清空当前腿长控制输出。
 *
 * @param control 本腿独立的腿长 PID 状态。
 */
static void WheelLeggedLegLengthControlReset(WheelLeggedLegLengthControl_t *control)
{
    if (control == NULL)
    {
        return;
    }

    PIDInit(&control->force_pid, &control->force_pid_config);
    control->force_pid_command = 0.0f;
    control->gravity_feedforward = 0.0f;
    control->force_command = 0.0f;
    control->valid = 0u;
}

/**
 * @brief 将有限的虚拟轴向力限制在指定绝对值范围内。
 *
 * @param force 待限幅的虚拟轴向力，单位 N。
 * @param force_limit 允许的最大绝对虚拟轴向力，单位 N。
 * @return 限幅后的虚拟轴向力；异常输入返回 0。
 */
static float WheelLeggedLegLengthControlLimitForce(float force, float force_limit)
{
    if (!isfinite(force) || !isfinite(force_limit) || force_limit < 0.0f)
    {
        return 0.0f;
    }
    if (force > force_limit)
    {
        return force_limit;
    }
    if (force < -force_limit)
    {
        return -force_limit;
    }
    return force;
}
