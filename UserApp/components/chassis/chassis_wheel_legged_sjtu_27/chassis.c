/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   SJTU 上层流程到 27demo ACE 底盘的安全适配实现
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis.h"

#include <math.h>
#include <string.h>

#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
/* 未配置动力学模型时故意传入的无效输出配置，确保 core 仲裁始终 Stop。 */
#define WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE 0.0f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t WheelLeggedSjtu27IsModelReady(const WheelLeggedSjtu27ModelConfig_t *config);
static void WheelLeggedSjtu27SyncLegacyCommand(ChassisInstance *chassis);
static void WheelLeggedSjtu27SyncLegacyState(ChassisInstance *chassis);
static float WheelLeggedSjtu27ClampLength(float value, float minimum, float maximum);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化兼容外壳和 27demo ACE/J4310/H6215 底盘内核。
 *
 * 动力学参数或本车专属 K 未完成时，底层输出配置被置为非法值；即使上层误请求
 * CHASSIS_ON，唯一输出仲裁也会保持六台电机 Stop。
 *
 * @param init_config robot_config.h 中的完整初始化配置。
 * @return 初始化后的兼容对象；参数或内存无效时返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *init_config)
{
    ChassisInstance *chassis;
    WheelLeggedChassisInitConfig_t core_config;
    static const WheelLeggedChassisLqrOutputConfig_t disabled_output_config = {
        .supported_body_mass = WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE,
        .pitch_torque_limit = WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE,
        .wheel_torque_limit = WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE,
        .wheel_torque_rate_limit = WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE,
        .minimum_support_projection = WHEEL_LEGGED_SJTU27_DISABLED_OUTPUT_CONFIG_VALUE,
    };

    if (init_config == NULL)
    {
        return NULL;
    }

    chassis = (ChassisInstance *)zmalloc(sizeof(*chassis));
    if (chassis == NULL)
    {
        return NULL;
    }
    memset(chassis, 0, sizeof(*chassis));

    chassis->model_config = init_config->model_config;
    chassis->model_ready = WheelLeggedSjtu27IsModelReady(&chassis->model_config);
    chassis->jump_state = JUMP_STATE_IDLE;
    chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
    chassis->chassis_ctrl_cmd.leg_length = init_config->initial_leg_length;

    core_config = init_config->wheel_legged_init_config;
    if (chassis->model_ready == 0u)
    {
        core_config.lqr_output_config = &disabled_output_config;
    }
    WheelLeggedChassisInit(&chassis->core, &core_config);
    chassis->imu = chassis->core.imu;
    chassis->leg[0] = &chassis->core.right_leg;
    chassis->leg[1] = &chassis->core.left_leg;
    chassis->super_cap = (WheelLeggedSjtu27SuperCap_t *)zmalloc(sizeof(*chassis->super_cap));
    if (chassis->super_cap != NULL)
    {
        chassis->super_cap->super_cap_mode = SAFETY_MODE;
        chassis->super_cap->super_cap_ctrl_cmd = NORMAL;
    }

    WheelLeggedSjtu27SyncLegacyCommand(chassis);
    WheelLeggedSjtu27SyncLegacyState(chassis);
    return chassis;
}

/**
 * @brief 执行当前 27demo 唯一底盘链，并把结果同步给旧 SJTU 上层。
 *
 * 旧跳跃、卧倒、台阶和功率控制模式只保留命令流/界面流程，尚未把它们映射为本车
 * 机构的动力学控制。只有模型和 K 均配置完成且请求 CHASSIS_ON 时，才允许 core
 * 进入其完整安全仲裁。
 *
 * @param chassis 兼容底盘对象。
 */
void WheelLeggedSjtu27ChassisTask(ChassisInstance *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    WheelLeggedSjtu27SyncLegacyCommand(chassis);
    ChassisTask(&chassis->core);
    WheelLeggedSjtu27SyncLegacyState(chassis);
}

/**
 * @brief 检查本车动力学参数与专属 LQR 系数是否已明确配置。
 *
 * @param config robot_config.h 中的模型配置。
 * @return 所有必需参数为有限正数且两个许可标志均置位时返回 1。
 */
static uint8_t WheelLeggedSjtu27IsModelReady(const WheelLeggedSjtu27ModelConfig_t *config)
{
    return config != NULL && config->configured != 0u && config->lqr_coefficients_configured != 0u &&
           isfinite(config->body_mass) && config->body_mass > 0.0f && isfinite(config->leg_mass) &&
           config->leg_mass > 0.0f && isfinite(config->wheel_mass) && config->wheel_mass > 0.0f &&
           isfinite(config->body_pitch_inertia) && config->body_pitch_inertia > 0.0f &&
           isfinite(config->leg_inertia) && config->leg_inertia > 0.0f && isfinite(config->wheel_inertia) &&
           config->wheel_inertia > 0.0f && isfinite(config->yaw_inertia) && config->yaw_inertia > 0.0f &&
           isfinite(config->leg_com_position) && config->leg_com_position >= 0.0f &&
           isfinite(config->track_width) && config->track_width > 0.0f;
}

/**
 * @brief 将旧 SJTU 命令安全映射到当前 27demo 内核。
 *
 * 当前只映射共同腿长参考与 ON/OFF 请求；vx、yaw、跳跃和卧倒必须等本车模型和上层
 * LQR 参考接口完成后再接入，防止旧车的控制合同被误用于本车。
 *
 * @param chassis 兼容底盘对象。
 */
static void WheelLeggedSjtu27SyncLegacyCommand(ChassisInstance *chassis)
{
    const uint8_t allow_output = chassis != NULL && chassis->model_ready != 0u &&
                                 chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_ON;
    float requested_length;

    if (chassis == NULL)
    {
        return;
    }

    requested_length = chassis->chassis_ctrl_cmd.leg_length;
    chassis->core.left_leg.length_control.length_reference = WheelLeggedSjtu27ClampLength(
        requested_length, chassis->core.left_leg.length_control.minimum_length,
        chassis->core.left_leg.length_control.maximum_length);
    chassis->core.right_leg.length_control.length_reference = WheelLeggedSjtu27ClampLength(
        requested_length, chassis->core.right_leg.length_control.minimum_length,
        chassis->core.right_leg.length_control.maximum_length);
    chassis->core.chassis_ctrl_cmd.chassis_mode =
        allow_output != 0u ? (WheelLeggedChassisMode_e)1u : (WheelLeggedChassisMode_e)0u;
}

/**
 * @brief 用当前 27demo 十维状态刷新 SJTU 上层兼容状态。
 *
 * @param chassis 兼容底盘对象。
 */
static void WheelLeggedSjtu27SyncLegacyState(ChassisInstance *chassis)
{
    const WheelLeggedChassisState_t *state;

    if (chassis == NULL)
    {
        return;
    }

    chassis->last_state_var = chassis->state_var;
    state = &chassis->core.chassis_state;
    chassis->state_var.x_b = state->s;
    chassis->state_var.x_b_d = state->s_dot;
    chassis->state_var.phi = state->phi;
    chassis->state_var.phi_d = state->phi_dot;
    chassis->state_var.theta_l = state->theta_leg_left;
    chassis->state_var.theta_l_d = state->theta_leg_left_dot;
    chassis->state_var.theta_r = state->theta_leg_right;
    chassis->state_var.theta_r_d = state->theta_leg_right_dot;
    chassis->state_var.theta_b = state->theta_body;
    chassis->state_var.theta_b_d = state->theta_body_dot;
    chassis->lqr_valid = chassis->model_ready != 0u ? chassis->core.lqr.valid : 0u;
}

/**
 * @brief 将上层腿长命令夹在底层实际 PID 工作区内。
 *
 * @param value 上层请求腿长，单位 m。
 * @param minimum 下限，单位 m。
 * @param maximum 上限，单位 m。
 * @return 合法范围内的腿长；参数非法时返回 0。
 */
static float WheelLeggedSjtu27ClampLength(float value, float minimum, float maximum)
{
    if (!isfinite(value) || !isfinite(minimum) || !isfinite(maximum) || minimum <= 0.0f || maximum < minimum)
    {
        return 0.0f;
    }
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}
