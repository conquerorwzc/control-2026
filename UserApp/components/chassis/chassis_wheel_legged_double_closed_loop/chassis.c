/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   双闭环轮腿底盘的初始化与周期任务调度
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis.h"

#include <math.h>
#include <string.h>

#include "chassis_private.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedWheelInit(WheelLeggedWheelInstance_t *wheel, WheelLeggedWheelInitConfig_t *config);
static uint8_t WheelLeggedChassisHasCompleteState(const WheelLeggedChassisInstance_t *chassis);
static uint8_t WheelLeggedChassisCalculateGravityFeedforward(const WheelLeggedChassisInstance_t *chassis,
                                                             float *gravity_feedforward);
static float WheelLeggedChassisLimitPitchTorque(float torque, float torque_limit);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化底盘对象、左右腿、左右轮毂和底盘 IMU。
 *
 * 初始化配置由 robot 层传入，底盘 component 不依赖任何具体机器人目录的配置文件。
 *
 * @param chassis 待初始化的底盘对象。
 * @param init_config 本车完整的底盘初始化配置。
 */
void WheelLeggedChassisInit(WheelLeggedChassisInstance_t *chassis, WheelLeggedChassisInitConfig_t *init_config)
{
    if (chassis == NULL || init_config == NULL)
    {
        return;
    }

    memset(chassis, 0, sizeof(*chassis));
    chassis->chassis_ctrl_cmd.chassis_mode = CHASSIS_POWER_OFF;
    if (init_config->state_config != NULL)
    {
        chassis->state_config = *init_config->state_config;
    }
    if (init_config->lqr_output_config != NULL)
    {
        chassis->lqr_output_config = *init_config->lqr_output_config;
    }
    WheelLeggedChassisLqrInit(&chassis->lqr);
    WheelLeggedLegInit(&chassis->left_leg, init_config->left_leg_init_config);
    WheelLeggedLegInit(&chassis->right_leg, init_config->right_leg_init_config);
    WheelLeggedWheelInit(&chassis->left_wheel, init_config->left_wheel_init_config);
    WheelLeggedWheelInit(&chassis->right_wheel, init_config->right_wheel_init_config);
    if (init_config->imu_init_config != NULL)
    {
        chassis->imu = INS_Init(init_config->imu_init_config);
    }
}

/**
 * @brief 初始化一台轮毂电机和轮端运动学参数，并立即保持零力矩状态。
 *
 * 达妙初始化过程会注册 CAN 回调并发送使能模式命令；完成注册后立即 Stop，
 * 后续状态读取仅访问反馈，不在此模块向轮毂写入控制参考。
 *
 * @param wheel 待初始化的轮毂运行对象。
 * @param config 本轮的电机和轮端参数配置。
 */
static void WheelLeggedWheelInit(WheelLeggedWheelInstance_t *wheel, WheelLeggedWheelInitConfig_t *config)
{
    if (wheel == NULL || config == NULL)
    {
        return;
    }

    wheel->wheel_radius = config->wheel_radius;
    wheel->reduction_ratio = config->reduction_ratio;
    wheel->direction = config->direction;
    wheel->configured = config->configured;
    wheel->motor = DMMotorInit(&config->motor_config);
    if (wheel->motor != NULL)
    {
        DMMotorStop(wheel->motor);
    }
}

/**
 * @brief 执行一次底盘周期任务。
 *
 * 固定顺序为：更新关节反馈和正运动学、组装整车十维状态、计算四输入 LQR、计算带重力前馈的
 * 两条腿腿长 PID、计算两条腿 VMC 映射，最后由唯一仲裁器下发六台电机。
 *
 * @param chassis 底盘对象。
 */
void ChassisTask(WheelLeggedChassisInstance_t *chassis)
{
    uint8_t state_valid;
    uint8_t gravity_valid;
    uint8_t length_control_valid;
    float gravity_feedforward = 0.0f;

    if (chassis == NULL)
    {
        return;
    }

    WheelLeggedLegUpdate(&chassis->left_leg);
    WheelLeggedLegUpdate(&chassis->right_leg);
    WheelLeggedChassisStateUpdate(chassis);
    state_valid = WheelLeggedChassisHasCompleteState(chassis);
    WheelLeggedChassisLqrUpdate(
        &chassis->lqr, chassis->chassis_state.state_vector, state_valid,
        chassis->chassis_state.origin_captured, chassis->chassis_state.left_leg_length,
        chassis->chassis_state.right_leg_length);

    chassis->right_leg.vmc.pitch_torque_command =
        chassis->lqr.valid != 0u
            ? WheelLeggedChassisLimitPitchTorque(chassis->lqr.tp_right, chassis->lqr_output_config.pitch_torque_limit)
            : 0.0f;
    chassis->left_leg.vmc.pitch_torque_command =
        chassis->lqr.valid != 0u
            ? WheelLeggedChassisLimitPitchTorque(chassis->lqr.tp_left, chassis->lqr_output_config.pitch_torque_limit)
            : 0.0f;

    gravity_valid = WheelLeggedChassisCalculateGravityFeedforward(chassis, &gravity_feedforward);
    length_control_valid = chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_ON && gravity_valid != 0u;
    WheelLeggedLegLengthControlUpdate(&chassis->left_leg.length_control,
                                      length_control_valid != 0u &&
                                          (chassis->chassis_state.valid_mask & WHEEL_LEGGED_STATE_VALID_LEFT_LEG) != 0u,
                                      chassis->chassis_state.left_leg_length,
                                      chassis->chassis_state.left_leg_length_dot, gravity_feedforward);
    chassis->left_leg.vmc.force_command = chassis->left_leg.length_control.force_command;
    WheelLeggedLegLengthControlUpdate(&chassis->right_leg.length_control,
                                      length_control_valid != 0u &&
                                          (chassis->chassis_state.valid_mask & WHEEL_LEGGED_STATE_VALID_RIGHT_LEG) != 0u,
                                      chassis->chassis_state.right_leg_length,
                                      chassis->chassis_state.right_leg_length_dot, gravity_feedforward);
    chassis->right_leg.vmc.force_command = chassis->right_leg.length_control.force_command;
    WheelLeggedLegVmcUpdate(&chassis->left_leg);
    WheelLeggedLegVmcUpdate(&chassis->right_leg);
    WheelLeggedChassisApplyMotorOutput(chassis);
}

/**
 * @brief 判断十维状态是否具备四输入 LQR 所需的全部实时来源。
 *
 * @param chassis 底盘对象。
 * @return 左右轮、IMU、左右腿和髋点里程计均有效时返回 1。
 */
static uint8_t WheelLeggedChassisHasCompleteState(const WheelLeggedChassisInstance_t *chassis)
{
    const uint16_t required_mask = WHEEL_LEGGED_STATE_VALID_LEFT_WHEEL | WHEEL_LEGGED_STATE_VALID_RIGHT_WHEEL |
                                   WHEEL_LEGGED_STATE_VALID_IMU | WHEEL_LEGGED_STATE_VALID_LEFT_LEG |
                                   WHEEL_LEGGED_STATE_VALID_RIGHT_LEG | WHEEL_LEGGED_STATE_VALID_HIP_ODOMETRY;
    return chassis != NULL && (chassis->chassis_state.valid_mask & required_mask) == required_mask;
}

/**
 * @brief 按当前左右腿纵向投影计算每条腿相同的重力前馈。
 *
 * @param chassis 底盘对象。
 * @param gravity_feedforward 返回单条腿前馈，单位 N。
 * @return 状态、投影和配置均有效时返回 1。
 */
static uint8_t WheelLeggedChassisCalculateGravityFeedforward(const WheelLeggedChassisInstance_t *chassis,
                                                             float *gravity_feedforward)
{
    float support_projection;
    float force;

    if (gravity_feedforward == NULL)
    {
        return 0u;
    }
    *gravity_feedforward = 0.0f;
    if (chassis == NULL || WheelLeggedChassisHasCompleteState(chassis) == 0u ||
        !isfinite(chassis->lqr_output_config.supported_body_mass) ||
        !isfinite(chassis->lqr_output_config.minimum_support_projection) ||
        chassis->lqr_output_config.supported_body_mass <= 0.0f ||
        chassis->lqr_output_config.minimum_support_projection <= 0.0f ||
        !isfinite(chassis->chassis_state.theta_leg_left) || !isfinite(chassis->chassis_state.theta_leg_right))
    {
        return 0u;
    }

    support_projection = cosf(chassis->chassis_state.theta_leg_left) + cosf(chassis->chassis_state.theta_leg_right);
    if (!isfinite(support_projection) || support_projection < chassis->lqr_output_config.minimum_support_projection)
    {
        return 0u;
    }

    force = chassis->lqr_output_config.supported_body_mass * 9.81f / support_projection;
    if (!isfinite(force))
    {
        return 0u;
    }
    *gravity_feedforward = force;
    return 1u;
}

/**
 * @brief 将 LQR 的虚拟摆动力矩限制在 robot 配置指定的首次测试范围。
 *
 * @param torque 待限幅的 Tp，单位 N*m。
 * @param torque_limit 绝对力矩限幅，单位 N*m。
 * @return 有限且已限幅的 Tp；配置或输入异常时返回 0。
 */
static float WheelLeggedChassisLimitPitchTorque(float torque, float torque_limit)
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
