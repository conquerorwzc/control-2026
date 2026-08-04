/**
 ******************************************************************************
 * @file    chassis_lqr_test.c
 * @brief   十维影子 LQR 固定工作点与输出合同主机端回归测试
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_lqr_test.h"

#include <math.h>
#include <string.h>

#include "chassis_lqr.h"

/* Private define ------------------------------------------------------------*/
/* 浮点回归测试允许的绝对误差。 */
#define CHASSIS_LQR_TEST_EPSILON 1e-5f

/* 固定 0.160 m 工作点导出 K 的浮点比较允许误差。 */
#define CHASSIS_LQR_TEST_GAIN_TOLERANCE 1e-5f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedChassisLqrExpectTrue(WheelLeggedChassisLqrSelfTestResult_t *result, uint32_t case_number,
                                            uint8_t condition);
static void WheelLeggedChassisLqrExpectNear(WheelLeggedChassisLqrSelfTestResult_t *result, uint32_t case_number,
                                            float actual, float expected);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 执行影子 LQR 的调度、控制律和失效回退回归测试。
 *
 * @return 所有测试的通过与失败统计。
 */
WheelLeggedChassisLqrSelfTestResult_t WheelLeggedChassisLqrRunSelfTest(void)
{
    WheelLeggedChassisLqrSelfTestResult_t result = {0};
    WheelLeggedChassisLqr_t lqr = {0};
    float state[WHEEL_LEGGED_LQR_STATE_COUNT] = {0.0f};
    float reference[WHEEL_LEGGED_LQR_STATE_COUNT] = {0.0f};
    float expected_output[WHEEL_LEGGED_LQR_INPUT_COUNT] = {0.0f};
    uint32_t state_index;
    uint32_t input_index;

    WheelLeggedChassisLqrInit(&lqr);
    WheelLeggedChassisLqrUpdate(&lqr, state, 1u, 1u, 0.16f, 0.16f);
    WheelLeggedChassisLqrExpectTrue(&result, 1u, lqr.valid != 0u);
    WheelLeggedChassisLqrExpectTrue(&result, 2u,
                                    fabsf(lqr.gain[0][0] + 0.375709593f) <= CHASSIS_LQR_TEST_GAIN_TOLERANCE);
    WheelLeggedChassisLqrExpectNear(&result, 20u, lqr.input_sign[WHEEL_LEGGED_LQR_INPUT_TP_RIGHT], -1.0f);
    WheelLeggedChassisLqrExpectNear(&result, 21u, lqr.input_sign[WHEEL_LEGGED_LQR_INPUT_TP_LEFT], -1.0f);
    WheelLeggedChassisLqrExpectNear(&result, 22u, lqr.input_sign[WHEEL_LEGGED_LQR_INPUT_TW_RIGHT], 1.0f);
    WheelLeggedChassisLqrExpectNear(&result, 23u, lqr.input_sign[WHEEL_LEGGED_LQR_INPUT_TW_LEFT], 1.0f);
    memcpy(reference, lqr.state_reference, sizeof(reference));
    memcpy(expected_output, lqr.trim_input, sizeof(expected_output));

    WheelLeggedChassisLqrUpdate(&lqr, reference, 1u, 1u, 0.16f, 0.16f);
    WheelLeggedChassisLqrExpectTrue(&result, 3u, lqr.valid != 0u);
    for (input_index = 0u; input_index < WHEEL_LEGGED_LQR_INPUT_COUNT; input_index++)
    {
        WheelLeggedChassisLqrExpectNear(&result, 4u + input_index, lqr.output[input_index], expected_output[input_index]);
    }

    state[WHEEL_LEGGED_LQR_STATE_S] = 0.01f;
    for (state_index = 1u; state_index < WHEEL_LEGGED_LQR_STATE_COUNT; state_index++)
    {
        state[state_index] = reference[state_index];
    }
    state[WHEEL_LEGGED_LQR_STATE_S] = reference[WHEEL_LEGGED_LQR_STATE_S] + 0.01f;
    WheelLeggedChassisLqrUpdate(&lqr, state, 1u, 1u, 0.16f, 0.16f);
    for (input_index = 0u; input_index < WHEEL_LEGGED_LQR_INPUT_COUNT; input_index++)
    {
        WheelLeggedChassisLqrExpectNear(&result, 8u + input_index, lqr.output[input_index],
                                        lqr.trim_input[input_index] -
                                            lqr.gain[input_index][WHEEL_LEGGED_LQR_STATE_S] * 0.01f);
    }

    WheelLeggedChassisLqrUpdate(&lqr, reference, 0u, 1u, 0.16f, 0.16f);
    WheelLeggedChassisLqrExpectTrue(&result, 12u, lqr.valid == 0u);
    WheelLeggedChassisLqrExpectNear(&result, 13u, lqr.output[WHEEL_LEGGED_LQR_INPUT_TP_RIGHT], 0.0f);

    WheelLeggedChassisLqrUpdate(&lqr, reference, 1u, 1u, 0.187f, 0.187f);
    WheelLeggedChassisLqrExpectTrue(&result, 14u, lqr.valid != 0u);
    WheelLeggedChassisLqrExpectNear(&result, 15u, lqr.left_leg_length, 0.187f);

    WheelLeggedChassisLqrUpdate(&lqr, reference, 1u, 1u, NAN, 0.16f);
    WheelLeggedChassisLqrExpectTrue(&result, 16u, lqr.valid == 0u);
    WheelLeggedChassisLqrExpectNear(&result, 17u, lqr.output[WHEEL_LEGGED_LQR_INPUT_TW_LEFT], 0.0f);

    state[WHEEL_LEGGED_LQR_STATE_PHI] = NAN;
    WheelLeggedChassisLqrUpdate(&lqr, state, 1u, 1u, 0.16f, 0.16f);
    WheelLeggedChassisLqrExpectTrue(&result, 18u, lqr.valid == 0u);
    WheelLeggedChassisLqrExpectNear(&result, 19u, lqr.tp_left, 0.0f);
    return result;
}

/**
 * @brief 记录一个浮点近似相等断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param actual 实际值。
 * @param expected 期望值。
 */
static void WheelLeggedChassisLqrExpectNear(WheelLeggedChassisLqrSelfTestResult_t *result, uint32_t case_number,
                                            float actual, float expected)
{
    WheelLeggedChassisLqrExpectTrue(result, case_number,
                                    isfinite(actual) && isfinite(expected) &&
                                        fabsf(actual - expected) <= CHASSIS_LQR_TEST_EPSILON);
}

/**
 * @brief 记录一个布尔断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param condition 非零代表断言通过。
 */
static void WheelLeggedChassisLqrExpectTrue(WheelLeggedChassisLqrSelfTestResult_t *result, uint32_t case_number,
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
