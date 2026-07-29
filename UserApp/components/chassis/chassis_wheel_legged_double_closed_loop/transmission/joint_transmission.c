/**
 ******************************************************************************
 * @file    joint_transmission.c
 * @brief   达妙反馈角到机构主动轴角的纯传动换算
 ******************************************************************************
 */
#include "joint_transmission.h"

#include <math.h>
#include <stddef.h>

JointTransmissionStatus_e JointTransmissionMotorToJoint(const JointTransmissionConfig_t *config, float motor_angle,
                                                        float *joint_angle)
{
    if (config == NULL || joint_angle == NULL)
    {
        return JOINT_TRANSMISSION_INVALID_ARGUMENT;
    }
    if (!config->configured)
    {
        return JOINT_TRANSMISSION_NOT_CONFIGURED;
    }
    if (!isfinite(motor_angle) || !isfinite(config->gain) || !isfinite(config->direction) ||
        !isfinite(config->zero_offset) || config->gain == 0.0f || config->direction == 0.0f)
    {
        return JOINT_TRANSMISSION_NUMERIC_ERROR;
    }

    *joint_angle = config->zero_offset + config->direction * config->gain * motor_angle;
    return isfinite(*joint_angle) ? JOINT_TRANSMISSION_OK : JOINT_TRANSMISSION_NUMERIC_ERROR;
}

/**
 * @brief 由机构主动轴角反算电机累计角。
 *
 * @param config 已标定的传动参数。
 * @param joint_angle 机构主动轴角，单位 rad。
 * @param motor_angle 反算得到的电机累计角，单位 rad。
 * @return 传动换算状态码。
 */
JointTransmissionStatus_e JointTransmissionJointToMotor(const JointTransmissionConfig_t *config, float joint_angle,
                                                        float *motor_angle)
{
    if (config == NULL || motor_angle == NULL)
    {
        return JOINT_TRANSMISSION_INVALID_ARGUMENT;
    }
    if (!config->configured)
    {
        return JOINT_TRANSMISSION_NOT_CONFIGURED;
    }
    if (!isfinite(joint_angle) || !isfinite(config->gain) || !isfinite(config->direction) ||
        !isfinite(config->zero_offset) || config->gain == 0.0f || config->direction == 0.0f)
    {
        return JOINT_TRANSMISSION_NUMERIC_ERROR;
    }

    *motor_angle = (joint_angle - config->zero_offset) / (config->direction * config->gain);
    return isfinite(*motor_angle) ? JOINT_TRANSMISSION_OK : JOINT_TRANSMISSION_NUMERIC_ERROR;
}
