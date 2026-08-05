/**
 ******************************************************************************
 * @file    chassis_vmc.c
 * @brief   双闭环轮腿的 VMC 广义力到关节力矩映射
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_private.h"

#include <math.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t WheelLeggedLegHasValidFeedback(const WheelLeggedLegInstance_t *leg);
static void WheelLeggedLegClearVmcResult(WheelLeggedLegInstance_t *leg);
static float WheelLeggedJointTorqueFromActiveTorque(const JointTransmissionConfig_t *transmission,
                                                    float active_joint_torque);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 根据当前真实末端 J 的腿雅可比，把本腿 F、Tp 命令映射为主动轴和电机轴目标力矩。
 *
 * 虚拟广义力正方向由虚功定义：F 为正使腿长增加；Tp 为正使虚拟腿角增加，
 * 即轮轴 P 相对髋点 O 向后摆，等价于髋点 O 相对轮轴 P 向前运动。
 * 第一阶段只保存计算结果，禁止在本文件或调用链中向达妙电机写入任何力矩参考。
 *
 * @param leg 待计算 VMC 映射的一条腿。
 */
void WheelLeggedLegVmcUpdate(WheelLeggedLegInstance_t *leg)
{
    if (leg == NULL)
    {
        return;
    }

    WheelLeggedLegClearVmcResult(leg);
    leg->vmc.leg_jacobian_det = leg->kinematics.state.real_leg_jacobian_det;
    if (leg->kinematics.forward_kinematics_status != PARALLEL_LEG_OK ||
        leg->kinematics.state.real_leg_jacobian_valid == 0u || leg->kinematics.config == NULL ||
        leg->front_joint_kinematics_input >= LEG_KINEMATICS_INPUT_COUNT ||
        leg->rear_joint_kinematics_input >= LEG_KINEMATICS_INPUT_COUNT || !WheelLeggedLegHasValidFeedback(leg))
    {
        return;
    }

    const float(*jacobian)[2] = leg->kinematics.state.real_leg_jacobian;
    leg->vmc.phi1_torque = jacobian[0][0] * leg->vmc.force_command + jacobian[1][0] * leg->vmc.pitch_torque_command;
    leg->vmc.phi2_torque = jacobian[0][1] * leg->vmc.force_command + jacobian[1][1] * leg->vmc.pitch_torque_command;
    leg->vmc.front_motor_torque = WheelLeggedJointTorqueFromActiveTorque(
        &leg->kinematics.config->transmission[leg->front_joint_kinematics_input],
        leg->front_joint_kinematics_input == LEG_KINEMATICS_INPUT_PHI1 ? leg->vmc.phi1_torque : leg->vmc.phi2_torque);
    leg->vmc.rear_motor_torque = WheelLeggedJointTorqueFromActiveTorque(
        &leg->kinematics.config->transmission[leg->rear_joint_kinematics_input],
        leg->rear_joint_kinematics_input == LEG_KINEMATICS_INPUT_PHI1 ? leg->vmc.phi1_torque : leg->vmc.phi2_torque);
    leg->vmc.valid = isfinite(leg->vmc.phi1_torque) && isfinite(leg->vmc.phi2_torque) &&
                     isfinite(leg->vmc.front_motor_torque) && isfinite(leg->vmc.rear_motor_torque);
    if (leg->vmc.valid == 0u)
    {
        WheelLeggedLegClearVmcResult(leg);
    }
}

/**
 * @brief 判断一条腿的两个主动关节是否均已收到有效反馈。
 *
 * @param leg 待检查的一条腿。
 * @return 两个关节均有反馈时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedLegHasValidFeedback(const WheelLeggedLegInstance_t *leg)
{
    return leg != NULL && leg->front_joint.feedback_ready != 0u && leg->rear_joint.feedback_ready != 0u;
}

/**
 * @brief 清空本周期 VMC 的计算结果，但保留调试器手填的 F、Tp 命令。
 *
 * @param leg 待清空结果的一条腿。
 */
static void WheelLeggedLegClearVmcResult(WheelLeggedLegInstance_t *leg)
{
    if (leg == NULL)
    {
        return;
    }

    leg->vmc.phi1_torque = 0.0f;
    leg->vmc.phi2_torque = 0.0f;
    leg->vmc.front_motor_torque = 0.0f;
    leg->vmc.rear_motor_torque = 0.0f;
    leg->vmc.valid = 0u;
}

/**
 * @brief 按虚功守恒把主动轴力矩换算为电机轴目标力矩。
 *
 * 若 phi = direction * gain * motor_angle + offset，则 motor_torque = direction * gain * phi_torque。
 *
 * @param transmission 当前主动轴的链传动配置。
 * @param active_joint_torque 主动轴目标力矩，单位 N·m。
 * @return 对应电机轴目标力矩，单位 N·m；传动配置无效时返回 0。
 */
static float WheelLeggedJointTorqueFromActiveTorque(const JointTransmissionConfig_t *transmission,
                                                    float active_joint_torque)
{
    if (transmission == NULL || transmission->configured == 0u || !isfinite(transmission->gain) ||
        !isfinite(transmission->direction) || !isfinite(active_joint_torque))
    {
        return 0.0f;
    }
    return transmission->direction * transmission->gain * active_joint_torque;
}
