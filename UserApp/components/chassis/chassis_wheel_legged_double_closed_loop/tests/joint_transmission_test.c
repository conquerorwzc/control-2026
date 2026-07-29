/**
 ******************************************************************************
 * @file    joint_transmission_test.c
 * @brief   传动换算模块主机端单测
 ******************************************************************************
 */
#include "joint_transmission_test.h"

#include <math.h>

#include "joint_transmission.h"

static void RecordCase(JointTransmissionSelfTestResult_t *result, uint32_t case_id, int passed)
{
    if (passed)
    {
        result->passed_count++;
    }
    else
    {
        result->failed_count++;
        result->last_failed_case = case_id;
    }
}

JointTransmissionSelfTestResult_t JointTransmissionRunSelfTest(void)
{
    JointTransmissionSelfTestResult_t result = {0};
    const JointTransmissionConfig_t configured = {
        .configured = true,
        .gain = 0.5f,
        .direction = -1.0f,
        .zero_offset = 0.2f,
    };
    float joint_angle = 0.0f;
    JointTransmissionStatus_e status = JointTransmissionMotorToJoint(&configured, 2.0f, &joint_angle);
    RecordCase(&result, 1u, status == JOINT_TRANSMISSION_OK && fabsf(joint_angle + 0.8f) < 1e-6f);

    JointTransmissionConfig_t unconfigured = configured;
    unconfigured.configured = false;
    status = JointTransmissionMotorToJoint(&unconfigured, 2.0f, &joint_angle);
    RecordCase(&result, 2u, status == JOINT_TRANSMISSION_NOT_CONFIGURED);

    JointTransmissionConfig_t invalid = configured;
    invalid.gain = 0.0f;
    status = JointTransmissionMotorToJoint(&invalid, 2.0f, &joint_angle);
    RecordCase(&result, 3u, status == JOINT_TRANSMISSION_NUMERIC_ERROR);

    return result;
}
