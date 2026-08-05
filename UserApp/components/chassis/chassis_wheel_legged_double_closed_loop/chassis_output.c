/**
 ******************************************************************************
 * @file    chassis_output.c
 * @brief   双闭环轮腿六台电机的唯一输出仲裁
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

#include <math.h>

#include "bsp_dwt.h"

/* Private define ------------------------------------------------------------*/
/* 单台 J4310 电机轴的调试力矩上限，单位 N*m；保留相对协议 +/-10 N*m 范围的余量。 */
#define WHEEL_LEGGED_JOINT_MOTOR_TORQUE_LIMIT 8.0f

/* 输出仲裁可接受的最大调度间隔；更长间隔按此值限速，避免恢复调度时突跳。 */
#define WHEEL_LEGGED_OUTPUT_MAX_RATE_DT 0.005f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t WheelLeggedChassisCanApplyClosedLoop(const WheelLeggedChassisInstance_t *chassis);
static uint16_t WheelLeggedChassisGetOutputBlockReason(const WheelLeggedChassisInstance_t *chassis);
static uint8_t WheelLeggedChassisHasAllMotorInstances(const WheelLeggedChassisInstance_t *chassis);
static uint8_t WheelLeggedChassisHasValidOutputConfig(const WheelLeggedChassisLqrOutputConfig_t *config);
static void WheelLeggedChassisSetAllMotorState(WheelLeggedChassisInstance_t *chassis, uint8_t enable);
static void WheelLeggedChassisSetAllTorqueReference(WheelLeggedChassisInstance_t *chassis, uint8_t apply_output,
                                                     float output_dt);
static void WheelLeggedSetLegTorqueReference(WheelLeggedLegInstance_t *leg, uint8_t apply_output);
static void WheelLeggedSetWheelTorqueReference(WheelLeggedWheelInstance_t *wheel, float requested_torque,
                                               float torque_limit, float torque_rate_limit, float output_dt,
                                               uint8_t apply_output);
static float WheelLeggedLimitTorque(float torque, float torque_limit);
static float WheelLeggedLimitTorqueRate(float target_torque, float previous_torque, float torque_rate_limit,
                                        float output_dt);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 从唯一出口仲裁六台底盘电机的 Stop、Enable 和力矩参考。
 *
 * 右拨杆中档只表达上层允许；实际出力还必须具备完整状态、LQR、腿长 PID、VMC、轮反馈、
 * 配置和六台电机实例。成功出力后的任一失效会锁定 Stop，必须先让右拨杆离开中档才能重置锁定。
 *
 * @param chassis 底盘对象。
 */
void WheelLeggedChassisApplyMotorOutput(WheelLeggedChassisInstance_t *chassis)
{
    const uint8_t remote_enable = chassis != NULL && chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_ON;
    uint8_t apply_output = 0u;
    float output_dt = 0.0f;

    if (chassis == NULL)
    {
        return;
    }

    chassis->output_block_reason = remote_enable != 0u ? WHEEL_LEGGED_OUTPUT_BLOCK_NONE
                                                       : WHEEL_LEGGED_OUTPUT_BLOCK_REMOTE_OFF;

    output_dt = DWT_GetDeltaT(&chassis->output_dwt_count);
    if (!isfinite(output_dt) || output_dt <= 0.0f)
    {
        output_dt = 0.0f;
    }
    else if (output_dt > WHEEL_LEGGED_OUTPUT_MAX_RATE_DT)
    {
        output_dt = WHEEL_LEGGED_OUTPUT_MAX_RATE_DT;
    }
    chassis->output_dt = output_dt;

    if (remote_enable == 0u)
    {
        chassis->output_fault_locked = 0u;
        chassis->output_ever_enabled = 0u;
    }
    else if (chassis->output_fault_locked == 0u && WheelLeggedChassisCanApplyClosedLoop(chassis) != 0u)
    {
        apply_output = 1u;
        chassis->output_ever_enabled = 1u;
    }
    else if (chassis->output_ever_enabled != 0u && chassis->output_fault_locked == 0u)
    {
        /* 先保存原始阻断原因，再追加 FAULT_LOCKED，避免只看到笼统的 2。 */
        chassis->output_fault_reason_latched = WheelLeggedChassisGetOutputBlockReason(chassis);
        chassis->output_fault_locked = 1u;
    }

    if (remote_enable != 0u)
    {
        chassis->output_block_reason = WheelLeggedChassisGetOutputBlockReason(chassis);
        if (chassis->output_fault_locked != 0u)
        {
            chassis->output_block_reason |= WHEEL_LEGGED_OUTPUT_BLOCK_FAULT_LOCKED;
        }
    }

    if (apply_output != chassis->motor_enabled)
    {
        WheelLeggedChassisSetAllMotorState(chassis, apply_output);
        chassis->motor_enabled = apply_output;
    }
    WheelLeggedChassisSetAllTorqueReference(chassis, apply_output, output_dt);
}

/**
 * @brief 判断当前是否满足完整四输入闭环的自动保护条件。
 *
 * @param chassis 底盘对象。
 * @return 所有实时状态、计算结果、电机实例与输出配置有效时返回 1。
 */
static uint8_t WheelLeggedChassisCanApplyClosedLoop(const WheelLeggedChassisInstance_t *chassis)
{
    return WheelLeggedChassisGetOutputBlockReason(chassis) == WHEEL_LEGGED_OUTPUT_BLOCK_NONE;
}

/**
 * @brief 汇总当前闭环输出被拦截的所有原因。
 *
 * @param chassis 底盘对象。
 * @return 输出阻断原因位掩码；返回 0 表示可以进入闭环输出。
 */
static uint16_t WheelLeggedChassisGetOutputBlockReason(const WheelLeggedChassisInstance_t *chassis)
{
    const uint16_t required_state_mask = WHEEL_LEGGED_STATE_VALID_LEFT_WHEEL |
                                         WHEEL_LEGGED_STATE_VALID_RIGHT_WHEEL | WHEEL_LEGGED_STATE_VALID_IMU |
                                         WHEEL_LEGGED_STATE_VALID_LEFT_LEG | WHEEL_LEGGED_STATE_VALID_RIGHT_LEG |
                                         WHEEL_LEGGED_STATE_VALID_WHEEL_ODOMETRY;
    uint16_t reason = WHEEL_LEGGED_OUTPUT_BLOCK_NONE;

    if (chassis == NULL)
    {
        return WHEEL_LEGGED_OUTPUT_BLOCK_STATE | WHEEL_LEGGED_OUTPUT_BLOCK_CONFIG |
               WHEEL_LEGGED_OUTPUT_BLOCK_MOTOR_INSTANCE;
    }
    if (chassis->chassis_state.origin_captured == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_ORIGIN;
    }
    if ((chassis->chassis_state.valid_mask & required_state_mask) != required_state_mask)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_STATE;
    }
    if (chassis->left_wheel.feedback_ready == 0u || chassis->right_wheel.feedback_ready == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_WHEEL_FEEDBACK;
    }
    if (chassis->lqr.valid == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_LQR;
    }
    if (chassis->left_leg.length_control.valid == 0u || chassis->right_leg.length_control.valid == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_LENGTH_CONTROL;
    }
    if (chassis->left_leg.vmc.valid == 0u || chassis->right_leg.vmc.valid == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_VMC;
    }
    if (WheelLeggedChassisHasValidOutputConfig(&chassis->lqr_output_config) == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_CONFIG;
    }
    if (WheelLeggedChassisHasAllMotorInstances(chassis) == 0u)
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_MOTOR_INSTANCE;
    }
    if (!isfinite(chassis->lqr.tp_right) || !isfinite(chassis->lqr.tp_left) ||
        !isfinite(chassis->lqr.tw_right) || !isfinite(chassis->lqr.tw_left) ||
        !isfinite(chassis->left_leg.length_control.force_command) ||
        !isfinite(chassis->right_leg.length_control.force_command) ||
        !isfinite(chassis->left_leg.vmc.front_motor_torque) ||
        !isfinite(chassis->left_leg.vmc.rear_motor_torque) ||
        !isfinite(chassis->right_leg.vmc.front_motor_torque) ||
        !isfinite(chassis->right_leg.vmc.rear_motor_torque))
    {
        reason |= WHEEL_LEGGED_OUTPUT_BLOCK_NONFINITE;
    }
    return reason;
}

/**
 * @brief 检查六台底盘电机实例都已经成功注册。
 *
 * @param chassis 底盘对象。
 * @return 全部实例非空时返回 1。
 */
static uint8_t WheelLeggedChassisHasAllMotorInstances(const WheelLeggedChassisInstance_t *chassis)
{
    return chassis != NULL && chassis->left_leg.front_joint.motor != NULL &&
           chassis->left_leg.rear_joint.motor != NULL && chassis->right_leg.front_joint.motor != NULL &&
           chassis->right_leg.rear_joint.motor != NULL && chassis->left_wheel.motor != NULL &&
           chassis->right_wheel.motor != NULL;
}

/**
 * @brief 检查首次小量闭环输出参数是否均为有限正数。
 *
 * @param config robot 配置中的 LQR 输出参数。
 * @return 参数合法时返回 1。
 */
static uint8_t WheelLeggedChassisHasValidOutputConfig(const WheelLeggedChassisLqrOutputConfig_t *config)
{
    return config != NULL && isfinite(config->supported_body_mass) && isfinite(config->pitch_torque_limit) &&
           isfinite(config->wheel_torque_limit) && isfinite(config->wheel_torque_rate_limit) &&
           isfinite(config->minimum_support_projection) && isfinite(config->prepare_length_tolerance) &&
           isfinite(config->prepare_length_rate_limit) && isfinite(config->prepare_leg_angle_limit) &&
           isfinite(config->prepare_body_angle_limit) && config->supported_body_mass > 0.0f &&
           config->pitch_torque_limit > 0.0f && config->wheel_torque_limit > 0.0f &&
           config->wheel_torque_rate_limit > 0.0f && config->minimum_support_projection > 0.0f &&
           config->prepare_length_tolerance > 0.0f && config->prepare_length_rate_limit > 0.0f &&
           config->prepare_leg_angle_limit > 0.0f && config->prepare_body_angle_limit > 0.0f &&
           config->prepare_stable_cycles > 0u;
}

/**
 * @brief 对六台电机统一下发 Stop 或 Enable，避免多处模块争夺模式。
 *
 * @param chassis 底盘对象。
 * @param enable 非零时使能，零时停止。
 */
static void WheelLeggedChassisSetAllMotorState(WheelLeggedChassisInstance_t *chassis, uint8_t enable)
{
    DMMotorInstance *motors[6];
    uint32_t motor_index;

    if (chassis == NULL)
    {
        return;
    }
    motors[0] = chassis->left_leg.front_joint.motor;
    motors[1] = chassis->left_leg.rear_joint.motor;
    motors[2] = chassis->right_leg.front_joint.motor;
    motors[3] = chassis->right_leg.rear_joint.motor;
    motors[4] = chassis->left_wheel.motor;
    motors[5] = chassis->right_wheel.motor;
    for (motor_index = 0u; motor_index < 6u; motor_index++)
    {
        if (motors[motor_index] != NULL)
        {
            if (enable != 0u)
            {
                DMMotorEnable(motors[motor_index]);
            }
            else
            {
                DMMotorStop(motors[motor_index]);
            }
        }
    }
}

/**
 * @brief 将 VMC 的两腿关节力矩和 LQR 的两轮 Tw 写入唯一电机出口。
 *
 * @param chassis 底盘对象。
 * @param apply_output 当前周期是否允许真实出力。
 * @param output_dt 输出斜率限制使用的周期，单位 s。
 */
static void WheelLeggedChassisSetAllTorqueReference(WheelLeggedChassisInstance_t *chassis, uint8_t apply_output,
                                                     float output_dt)
{
    float left_wheel_torque = 0.0f;
    float right_wheel_torque = 0.0f;

    if (chassis == NULL)
    {
        return;
    }
    if (chassis->balance_phase == WHEEL_LEGGED_BALANCE_PHASE_ACTIVE)
    {
        left_wheel_torque = chassis->lqr.tw_left;
        right_wheel_torque = chassis->lqr.tw_right;
    }
    WheelLeggedSetLegTorqueReference(&chassis->left_leg, apply_output);
    WheelLeggedSetLegTorqueReference(&chassis->right_leg, apply_output);
    WheelLeggedSetWheelTorqueReference(&chassis->left_wheel, left_wheel_torque,
                                       chassis->lqr_output_config.wheel_torque_limit,
                                       chassis->lqr_output_config.wheel_torque_rate_limit, output_dt, apply_output);
    WheelLeggedSetWheelTorqueReference(&chassis->right_wheel, right_wheel_torque,
                                       chassis->lqr_output_config.wheel_torque_limit,
                                       chassis->lqr_output_config.wheel_torque_rate_limit, output_dt, apply_output);
}

/**
 * @brief 写入一条腿的两个 J4310 电机轴力矩，失效时强制写零。
 *
 * @param leg 腿对象。
 * @param apply_output 当前周期是否允许真实出力。
 */
static void WheelLeggedSetLegTorqueReference(WheelLeggedLegInstance_t *leg, uint8_t apply_output)
{
    float front_torque = 0.0f;
    float rear_torque = 0.0f;

    if (leg == NULL)
    {
        return;
    }
    if (apply_output != 0u)
    {
        front_torque = WheelLeggedLimitTorque(leg->vmc.front_motor_torque, WHEEL_LEGGED_JOINT_MOTOR_TORQUE_LIMIT);
        rear_torque = WheelLeggedLimitTorque(leg->vmc.rear_motor_torque, WHEEL_LEGGED_JOINT_MOTOR_TORQUE_LIMIT);
    }
    leg->vmc.front_motor_torque_output = front_torque;
    leg->vmc.rear_motor_torque_output = rear_torque;
    if (leg->front_joint.motor != NULL)
    {
        DMMotorSetRef(leg->front_joint.motor, front_torque);
    }
    if (leg->rear_joint.motor != NULL)
    {
        DMMotorSetRef(leg->rear_joint.motor, rear_torque);
    }
}

/**
 * @brief 对一台 H6215 先做轮端力矩限幅、再做变化率限制并写入电机参考。
 *
 * Tw 的符号已在 MATLAB 导出的 K 中定义为该轮向前滚动；本处不再乘 direction，
 * 电机安装镜像只由本车 motor_reverse_flag 与 feedback_reverse_flag 成对配置处理。
 *
 * @param wheel 轮对象。
 * @param requested_torque LQR 的该轮 Tw，单位 N*m。
 * @param torque_limit 轮端力矩绝对限幅，单位 N*m。
 * @param torque_rate_limit 轮端力矩变化率上限，单位 N*m/s。
 * @param output_dt 当前输出周期，单位 s。
 * @param apply_output 当前周期是否允许真实出力。
 */
static void WheelLeggedSetWheelTorqueReference(WheelLeggedWheelInstance_t *wheel, float requested_torque,
                                               float torque_limit, float torque_rate_limit, float output_dt,
                                               uint8_t apply_output)
{
    float torque_command = 0.0f;
    float motor_torque_output = 0.0f;

    if (wheel == NULL)
    {
        return;
    }
    if (apply_output != 0u)
    {
        torque_command = WheelLeggedLimitTorque(requested_torque, torque_limit);
        motor_torque_output = WheelLeggedLimitTorqueRate(torque_command, wheel->previous_motor_torque,
                                                         torque_rate_limit, output_dt);
    }
    wheel->torque_command = torque_command;
    wheel->motor_torque_output = motor_torque_output;
    wheel->previous_motor_torque = motor_torque_output;
    if (wheel->motor != NULL)
    {
        DMMotorSetRef(wheel->motor, motor_torque_output);
    }
}

/**
 * @brief 限制一个有限力矩的绝对值。
 *
 * @param torque 待限幅力矩，单位 N*m。
 * @param torque_limit 绝对上限，单位 N*m。
 * @return 限幅后的力矩；异常输入返回 0。
 */
static float WheelLeggedLimitTorque(float torque, float torque_limit)
{
    if (!isfinite(torque) || !isfinite(torque_limit) || torque_limit <= 0.0f)
    {
        return 0.0f;
    }
    if (torque > torque_limit)
    {
        return torque_limit;
    }
    if (torque < -torque_limit)
    {
        return -torque_limit;
    }
    return torque;
}

/**
 * @brief 在有限周期内限制轮端力矩相对于上一帧的变化量。
 *
 * @param target_torque 限幅后的目标力矩，单位 N*m。
 * @param previous_torque 上一帧实际输出力矩，单位 N*m。
 * @param torque_rate_limit 最大变化率，单位 N*m/s。
 * @param output_dt 本周期时间间隔，单位 s。
 * @return 变化率受限后的电机轴力矩；异常输入返回 0。
 */
static float WheelLeggedLimitTorqueRate(float target_torque, float previous_torque, float torque_rate_limit,
                                        float output_dt)
{
    float maximum_delta;
    float torque_delta;

    if (!isfinite(target_torque) || !isfinite(previous_torque) || !isfinite(torque_rate_limit) ||
        !isfinite(output_dt) || torque_rate_limit <= 0.0f || output_dt <= 0.0f)
    {
        return 0.0f;
    }
    maximum_delta = torque_rate_limit * output_dt;
    torque_delta = target_torque - previous_torque;
    if (torque_delta > maximum_delta)
    {
        return previous_torque + maximum_delta;
    }
    if (torque_delta < -maximum_delta)
    {
        return previous_torque - maximum_delta;
    }
    return target_torque;
}
