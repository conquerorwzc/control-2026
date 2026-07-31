/**
 ******************************************************************************
 * @file    hip_odometry_test.c
 * @brief   髋点纵向里程计主机端单测
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "hip_odometry_test.h"

#include <math.h>
#include <stddef.h>

#include "hip_odometry.h"

/* Private define ------------------------------------------------------------*/
/* 浮点数主机端断言允许误差。 */
#define HIP_ODOMETRY_TEST_EPSILON 1e-6f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedHipOdometryExpectNear(WheelLeggedHipOdometryTestResult_t *result, uint32_t case_number,
                                             float actual, float expected);
static void WheelLeggedHipOdometryExpectTrue(WheelLeggedHipOdometryTestResult_t *result, uint32_t case_number,
                                             uint8_t condition);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 执行髋点纵向位置、速度计算的主机端单测。
 *
 * @return 全部用例的通过、失败数量和最近失败编号。
 */
WheelLeggedHipOdometryTestResult_t WheelLeggedHipOdometryRunSelfTest(void)
{
    WheelLeggedHipOdometryTestResult_t result = {0};
    WheelLeggedHipOdometryLegInput_t input = {
        .wheel_radius = 0.06f,
        .wheel_angle = 2.0f,
        .wheel_speed = 3.0f,
        .leg_length = 0.15f,
        .leg_length_dot = 0.0f,
        .leg_theta = 0.0f,
        .leg_theta_dot = 0.0f,
    };
    WheelLeggedHipOdometryLegOutput_t output = {0};

    WheelLeggedHipOdometryExpectTrue(&result, 1u, WheelLeggedHipOdometryCalculate(&input, &output));
    WheelLeggedHipOdometryExpectNear(&result, 2u, output.raw_position, 0.12f);
    WheelLeggedHipOdometryExpectNear(&result, 3u, output.velocity, 0.18f);

    input.wheel_angle = 0.0f;
    input.wheel_speed = 0.0f;
    input.leg_length = 0.20f;
    input.leg_length_dot = 0.0f;
    input.leg_theta = 0.0f;
    input.leg_theta_dot = 2.0f;
    WheelLeggedHipOdometryExpectTrue(&result, 4u, WheelLeggedHipOdometryCalculate(&input, &output));
    WheelLeggedHipOdometryExpectNear(&result, 5u, output.velocity, 0.40f);

    input.leg_length_dot = 0.30f;
    input.leg_theta = 0.5f * 3.14159265358979323846f;
    input.leg_theta_dot = 0.0f;
    WheelLeggedHipOdometryExpectTrue(&result, 6u, WheelLeggedHipOdometryCalculate(&input, &output));
    WheelLeggedHipOdometryExpectNear(&result, 7u, output.velocity, 0.30f);

    WheelLeggedHipOdometryLegOutput_t right_output = {0};
    WheelLeggedHipOdometryExpectTrue(&result, 8u, WheelLeggedHipOdometryCalculate(&input, &right_output));
    WheelLeggedHipOdometryExpectNear(&result, 9u, output.raw_position, right_output.raw_position);
    WheelLeggedHipOdometryExpectNear(&result, 10u, output.velocity, right_output.velocity);

    input.wheel_radius = 0.0f;
    WheelLeggedHipOdometryExpectTrue(&result, 11u, !WheelLeggedHipOdometryCalculate(&input, &output));
    return result;
}

/**
 * @brief 记录一个浮点数近似相等断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param actual 实际值。
 * @param expected 期望值。
 */
static void WheelLeggedHipOdometryExpectNear(WheelLeggedHipOdometryTestResult_t *result, uint32_t case_number,
                                             float actual, float expected)
{
    WheelLeggedHipOdometryExpectTrue(result, case_number,
                                     isfinite(actual) && isfinite(expected) && fabsf(actual - expected) <= HIP_ODOMETRY_TEST_EPSILON);
}

/**
 * @brief 记录一个布尔断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param condition 断言条件，非零代表通过。
 */
static void WheelLeggedHipOdometryExpectTrue(WheelLeggedHipOdometryTestResult_t *result, uint32_t case_number,
                                             uint8_t condition)
{
    if (result == NULL)
    {
        return;
    }
    if (condition != 0u)
    {
        result->passed_count++;
    }
    else
    {
        result->failed_count++;
        result->last_failed_case = case_number;
    }
}
