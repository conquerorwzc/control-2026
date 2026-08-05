/**
 ******************************************************************************
 * @file    test_main.c
 * @brief   同心五连杆主机端单测入口，不参与单片机固件构建
 ******************************************************************************
 */
#include <stdio.h>

#include "hip_odometry_test.h"
#include "wheel_odometry_test.h"
#include "joint_transmission_test.h"
#include "chassis_lqr_test.h"
#include "chassis_output_test.h"
#include "length_control_test.h"
#include "parallel_leg_test.h"

int main(void)
{
    const ParallelLegSelfTestResult_t geometry_result = ParallelLegRunSelfTest();
    const JointTransmissionSelfTestResult_t transmission_result = JointTransmissionRunSelfTest();
    const WheelLeggedHipOdometryTestResult_t hip_odometry_result = WheelLeggedHipOdometryRunSelfTest();
    const WheelLeggedWheelOdometryTestResult_t wheel_odometry_result = WheelLeggedWheelOdometryRunSelfTest();
    const WheelLeggedLengthControlSelfTestResult_t length_control_result = WheelLeggedLengthControlRunSelfTest();
    const WheelLeggedChassisLqrSelfTestResult_t lqr_result = WheelLeggedChassisLqrRunSelfTest();
    const WheelLeggedChassisOutputSelfTestResult_t output_result = WheelLeggedChassisOutputRunSelfTest();

    printf("geometry: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)geometry_result.passed_count,
           (unsigned long)geometry_result.failed_count, (unsigned long)geometry_result.last_failed_case);
    printf("transmission: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)transmission_result.passed_count,
           (unsigned long)transmission_result.failed_count, (unsigned long)transmission_result.last_failed_case);
    printf("hip_odometry: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)hip_odometry_result.passed_count,
           (unsigned long)hip_odometry_result.failed_count, (unsigned long)hip_odometry_result.last_failed_case);
    printf("wheel_odometry: passed=%lu failed=%lu last_failed=%lu\n",
           (unsigned long)wheel_odometry_result.passed_count, (unsigned long)wheel_odometry_result.failed_count,
           (unsigned long)wheel_odometry_result.last_failed_case);
    printf("length_control: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)length_control_result.passed_count,
           (unsigned long)length_control_result.failed_count, (unsigned long)length_control_result.last_failed_case);
    printf("lqr: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)lqr_result.passed_count,
           (unsigned long)lqr_result.failed_count, (unsigned long)lqr_result.last_failed_case);
    printf("output: passed=%lu failed=%lu last_failed=%lu\n", (unsigned long)output_result.passed_count,
           (unsigned long)output_result.failed_count, (unsigned long)output_result.last_failed_case);

    return (geometry_result.failed_count == 0u && transmission_result.failed_count == 0u &&
            hip_odometry_result.failed_count == 0u && length_control_result.failed_count == 0u &&
            wheel_odometry_result.failed_count == 0u &&
            lqr_result.failed_count == 0u && output_result.failed_count == 0u)
               ? 0
               : 1;
}
