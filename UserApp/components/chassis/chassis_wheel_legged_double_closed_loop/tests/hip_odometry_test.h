/**
 ******************************************************************************
 * @file    hip_odometry_test.h
 * @brief   髋点纵向里程计主机端单测接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t passed_count;    /* 已通过的断言数量。 */
    uint32_t failed_count;    /* 已失败的断言数量。 */
    uint32_t last_failed_case; /* 最近失败用例编号。 */
} WheelLeggedHipOdometryTestResult_t;

WheelLeggedHipOdometryTestResult_t WheelLeggedHipOdometryRunSelfTest(void);
