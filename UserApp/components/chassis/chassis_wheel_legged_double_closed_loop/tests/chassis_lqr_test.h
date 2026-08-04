/**
 ******************************************************************************
 * @file    chassis_lqr_test.h
 * @brief   十维影子 LQR 主机端回归测试接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

typedef struct
{
    uint32_t passed_count;      /* 已通过用例数。 */
    uint32_t failed_count;      /* 已失败用例数。 */
    uint32_t last_failed_case;  /* 最后一个失败用例编号。 */
} WheelLeggedChassisLqrSelfTestResult_t;

WheelLeggedChassisLqrSelfTestResult_t WheelLeggedChassisLqrRunSelfTest(void);
