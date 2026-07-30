/**
 ******************************************************************************
 * @file    chassis_leg.c
 * @brief   双闭环轮腿的关节初始化、反馈读取与正运动学更新
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

#include <string.h>

#include "joint_transmission.h"

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t IsValidLegKinematicsInput(LegKinematicsInput_e input);
static JointTransmissionConfig_t WheelLeggedBuildJointTransmissionConfig(
    const WheelLeggedChainTransmissionConfig_t *chain_config);
static float WheelLeggedJointGetAngle(WheelLeggedJointInstance_t *joint);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化一条腿的前后主动关节和纯机构学实例。
 *
 * 关节初始化后立即置为零力矩；运行期不在读取反馈时反复 Stop 电机。
 *
 * @param leg 待初始化的腿对象。
 * @param config 该腿的关节电机与机构学配置。
 */
void WheelLeggedLegInit(WheelLeggedLegInstance_t *leg, WheelLeggedLegInitConfig_t *config)
{
    if (leg == NULL || config == NULL)
    {
        return;
    }

    memset(leg, 0, sizeof(*leg));
    if (config->geometry_config != NULL)
    {
        leg->kinematics_runtime_config.geometry = *config->geometry_config;
    }
    leg->front_joint_kinematics_input = config->front_joint_kinematics_input;
    leg->rear_joint_kinematics_input = config->rear_joint_kinematics_input;
    if (IsValidLegKinematicsInput(leg->front_joint_kinematics_input) &&
        IsValidLegKinematicsInput(leg->rear_joint_kinematics_input) &&
        leg->front_joint_kinematics_input != leg->rear_joint_kinematics_input)
    {
        leg->kinematics_runtime_config.transmission[leg->front_joint_kinematics_input] =
            WheelLeggedBuildJointTransmissionConfig(config->front_joint_chain_config);
        leg->kinematics_runtime_config.transmission[leg->rear_joint_kinematics_input] =
            WheelLeggedBuildJointTransmissionConfig(config->rear_joint_chain_config);
    }
    leg->front_joint.motor = DMMotorInit(&config->front_joint_motor_config);
    leg->rear_joint.motor = DMMotorInit(&config->rear_joint_motor_config);
    if (leg->front_joint.motor != NULL)
    {
        DMMotorStop(leg->front_joint.motor);
    }
    if (leg->rear_joint.motor != NULL)
    {
        DMMotorStop(leg->rear_joint.motor);
    }
    DoubleClosedLoopLegInit(&leg->kinematics, &leg->kinematics_runtime_config);
}

/**
 * @brief 更新一条腿的电机反馈与正运动学观测状态。
 *
 * @param leg 待更新的腿对象。
 */
void WheelLeggedLegUpdate(WheelLeggedLegInstance_t *leg)
{
    if (leg == NULL)
    {
        return;
    }

    leg->sequence++;
    const float front_actuator_angle = WheelLeggedJointGetAngle(&leg->front_joint);
    const float rear_actuator_angle = WheelLeggedJointGetAngle(&leg->rear_joint);
    float actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_COUNT] = {0.0f};
    if (IsValidLegKinematicsInput(leg->front_joint_kinematics_input) &&
        IsValidLegKinematicsInput(leg->rear_joint_kinematics_input) &&
        leg->front_joint_kinematics_input != leg->rear_joint_kinematics_input)
    {
        actuator_angle_by_kinematics_input[leg->front_joint_kinematics_input] = front_actuator_angle;
        actuator_angle_by_kinematics_input[leg->rear_joint_kinematics_input] = rear_actuator_angle;
    }
    DoubleClosedLoopLegUpdate(&leg->kinematics, actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_PHI1],
                              actuator_angle_by_kinematics_input[LEG_KINEMATICS_INPUT_PHI2]);
    leg->update_count++;
    leg->sequence++;
}

/**
 * @brief 读取主动关节累计转子角，并刷新反馈有效标志。
 *
 * 本函数只有读取行为，绝不修改电机 Stop、Enable 或控制参考。
 *
 * @param joint 主动关节对象。
 * @return 电机累计转子角；关节或电机对象无效时返回 0。
 */
static float WheelLeggedJointGetAngle(WheelLeggedJointInstance_t *joint)
{
    if (joint == NULL || joint->motor == NULL)
    {
        return 0.0f;
    }

    joint->feedback_ready = joint->motor->measure.state != 0u;
    return joint->motor->measure.total_angle;
}

/**
 * @brief 判断主动关节反馈对应的机构学输入槽位是否有效。
 *
 * @param input 待判断的 phi1 或 phi2 槽位。
 * @return 有效返回 1，无效返回 0。
 */
static uint8_t IsValidLegKinematicsInput(LegKinematicsInput_e input)
{
    return input < LEG_KINEMATICS_INPUT_COUNT;
}

/**
 * @brief 根据链轮齿数生成底层传动换算所需的角度比例配置。
 *
 * @param chain_config 电机侧到主动轴侧的链轮传动标定参数。
 * @return 供纯机构学模块使用的传动配置。
 */
static JointTransmissionConfig_t WheelLeggedBuildJointTransmissionConfig(
    const WheelLeggedChainTransmissionConfig_t *chain_config)
{
    JointTransmissionConfig_t transmission_config = {0};
    if (chain_config == NULL || chain_config->driving_sprocket_teeth == 0u ||
        chain_config->driven_sprocket_teeth == 0u)
    {
        return transmission_config;
    }

    transmission_config.configured = chain_config->configured != 0u;
    transmission_config.gain = (float)chain_config->driving_sprocket_teeth / (float)chain_config->driven_sprocket_teeth;
    transmission_config.direction = chain_config->direction;
    transmission_config.zero_offset =
        -transmission_config.direction * transmission_config.gain * chain_config->motor_zero_angle;
    return transmission_config;
}
