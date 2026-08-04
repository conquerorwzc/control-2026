/**
 ******************************************************************************
 * @file    test_main.c
 * @brief   同心五连杆主机端单测入口，不参与单片机固件构建
 ******************************************************************************
 */
#include <stdio.h>

#include "parallel_leg_test.h"
#include "hip_odometry_test.h"
#include "joint_transmission_test.h"

int main(void)
{
    const ParallelLegSelfTestResult_t geometry_result = ParallelLegRunSelfTest();
    const JointTransmissionSelfTestResult_t transmission_result = JointTransmissionRunSelfTest();
    const WheelLeggedHipOdometryTestResult_t hip_odometry_result = WheelLeggedHipOdometryRunSelfTest();

    printf("geometry: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)geometry_result.passed_count,
           (unsigned long)geometry_result.failed_count, (unsigned long)geometry_result.last_failed_case);
    printf("transmission: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)transmission_result.passed_count,
           (unsigned long)transmission_result.failed_count, (unsigned long)transmission_result.last_failed_case);
    printf("hip_odometry: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)hip_odometry_result.passed_count,
           (unsigned long)hip_odometry_result.failed_count, (unsigned long)hip_odometry_result.last_failed_case);

    return (geometry_result.failed_count == 0u && transmission_result.failed_count == 0u &&
            hip_odometry_result.failed_count == 0u)
               ? 0
               : 1;
}
