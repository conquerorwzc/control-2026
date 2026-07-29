/**
 ******************************************************************************
 * @file    joint_transmission_test.h
 * @brief   传动换算模块主机端单测
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t passed_count;     /* 已通过的测试用例数量。 */
    uint32_t failed_count;     /* 已失败的测试用例数量。 */
    uint32_t last_failed_case; /* 最近一个失败的测试用例编号，0 表示无失败。 */
} JointTransmissionSelfTestResult_t;

JointTransmissionSelfTestResult_t JointTransmissionRunSelfTest(void);
