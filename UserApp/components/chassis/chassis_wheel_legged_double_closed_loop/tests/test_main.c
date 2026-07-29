/**
 ******************************************************************************
 * @file    test_main.c
 * @brief   双闭环腿主机端单测入口，不参与单片机固件构建
 ******************************************************************************
 */
#include <stdio.h>

#include "double_closed_loop_leg_test.h"
#include "joint_transmission_test.h"

int main(void)
{
    const DoubleClosedLoopLegSelfTestResult_t geometry_result = DoubleClosedLoopLegRunSelfTest();
    const JointTransmissionSelfTestResult_t transmission_result = JointTransmissionRunSelfTest();

    printf("geometry: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)geometry_result.passed_count,
           (unsigned long)geometry_result.failed_count, (unsigned long)geometry_result.last_failed_case);
    printf("transmission: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)transmission_result.passed_count,
           (unsigned long)transmission_result.failed_count, (unsigned long)transmission_result.last_failed_case);

    return (geometry_result.failed_count == 0u && transmission_result.failed_count == 0u) ? 0 : 1;
}
