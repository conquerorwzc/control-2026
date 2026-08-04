/**
 ******************************************************************************
 * @file    length_control_test.c
 * @brief   双闭环轮腿腿长 PID 主机端单测
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "length_control_test.h"

#include <math.h>
#include <stddef.h>

#include "chassis_length_control.h"

/* Private define ------------------------------------------------------------*/
/* 浮点数主机端断言允许误差。 */
#define LENGTH_CONTROL_TEST_EPSILON 1e-5f

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static WheelLeggedLegLengthControlInitConfig_t WheelLeggedLengthControlCreateDefaultConfig(void);
static WheelLeggedLegLengthControl_t WheelLeggedLengthControlCreateDefault(void);
static void WheelLeggedLengthControlExpectNear(WheelLeggedLengthControlSelfTestResult_t *result, uint32_t case_number,
                                               float actual, float expected);
static void WheelLeggedLengthControlExpectTrue(WheelLeggedLengthControlSelfTestResult_t *result, uint32_t case_number,
                                               uint8_t condition);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 执行通用 controller PID 与 Jacobian 腿长阻尼的主机端单测。
 *
 * @return 全部用例的通过、失败数量和最近失败编号。
 */
WheelLeggedLengthControlSelfTestResult_t WheelLeggedLengthControlRunSelfTest(void)
{
    WheelLeggedLengthControlSelfTestResult_t result = {0};
    WheelLeggedLegLengthControl_t control = WheelLeggedLengthControlCreateDefault();

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 1u, control.valid != 0u);
    WheelLeggedLengthControlExpectNear(&result, 2u, control.force_command, 0.0f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.13f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 3u, control.force_command > 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 4u, control.force_command, 10.0f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.17f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 5u, control.force_command < 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 6u, control.force_command, -10.0f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.2f, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 7u, control.force_command, -1.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, -0.2f, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 8u, control.force_command, 1.0f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, 8.25f);
    WheelLeggedLengthControlExpectNear(&result, 29u, control.force_pid_command, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 30u, control.gravity_feedforward, 8.25f);
    WheelLeggedLengthControlExpectNear(&result, 31u, control.force_command, 8.25f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, 30.0f);
    WheelLeggedLengthControlExpectNear(&result, 32u, control.force_command, 20.0f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.09f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 9u, control.force_command, 20.0f);
    WheelLeggedLengthControlExpectNear(&result, 10u, control.force_pid.Iout, 0.0f);

    control.force_pid_config.MaxOut = 10.0f;
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.09f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 11u, control.valid != 0u);
    WheelLeggedLengthControlExpectNear(&result, 12u, control.force_pid.MaxOut, 10.0f);
    WheelLeggedLengthControlExpectNear(&result, 13u, control.force_command, 10.0f);

    control.force_pid_config.MaxOut = 20.0f;
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.09f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 14u, control.valid != 0u);
    WheelLeggedLengthControlExpectNear(&result, 15u, control.force_pid.MaxOut, 20.0f);
    WheelLeggedLengthControlExpectNear(&result, 16u, control.force_command, 20.0f);

    WheelLeggedLegLengthControlInitConfig_t integral_config = WheelLeggedLengthControlCreateDefaultConfig();
    integral_config.force_pid_config.Ki = 100.0f;
    integral_config.force_pid_config.IntegralLimit = 0.005f;
    WheelLeggedLegLengthControlInit(&control, &integral_config);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.14f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 17u, control.force_pid.Iout, 0.005f);

    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.06f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 18u, control.valid == 0u);
    WheelLeggedLengthControlExpectNear(&result, 19u, control.force_command, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 20u, control.force_pid.Iout, 0.0f);
    WheelLeggedLengthControlExpectNear(&result, 21u, control.force_pid.Output, 0.0f);

    control = WheelLeggedLengthControlCreateDefault();
    WheelLeggedLegLengthControlUpdate(&control, 0u, 0.15f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 22u, control.valid == 0u);
    WheelLeggedLengthControlExpectNear(&result, 23u, control.force_command, 0.0f);

    WheelLeggedLegLengthControlInitConfig_t invalid_config = WheelLeggedLengthControlCreateDefaultConfig();
    invalid_config.force_pid_config.Kd = 1.0f;
    WheelLeggedLegLengthControlInit(&control, &invalid_config);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 24u, control.valid == 0u);
    WheelLeggedLengthControlExpectNear(&result, 25u, control.force_command, 0.0f);

    invalid_config = WheelLeggedLengthControlCreateDefaultConfig();
    invalid_config.force_pid_config.Improve |= PID_Derivative_On_Measurement;
    WheelLeggedLegLengthControlInit(&control, &invalid_config);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 26u, control.valid == 0u);

    control = WheelLeggedLengthControlCreateDefault();
    WheelLeggedLegLengthControlUpdate(&control, 1u, NAN, 0.0f, 0.0f);
    WheelLeggedLengthControlExpectTrue(&result, 27u, control.valid == 0u);
    WheelLeggedLengthControlExpectNear(&result, 28u, control.force_command, 0.0f);
    WheelLeggedLegLengthControlUpdate(&control, 1u, 0.15f, 0.0f, NAN);
    WheelLeggedLengthControlExpectTrue(&result, 33u, control.valid == 0u);
    WheelLeggedLengthControlExpectNear(&result, 34u, control.gravity_feedforward, 0.0f);
    return result;
}

/**
 * @brief 创建与实机初始配置一致的单条腿腿长 PID 初始化配置。
 *
 * @return 默认打开且参数有效的腿长 PID 初始化配置。
 */
static WheelLeggedLegLengthControlInitConfig_t WheelLeggedLengthControlCreateDefaultConfig(void)
{
    const WheelLeggedLegLengthControlInitConfig_t config = {
        .force_pid_config =
            {
                .Kp = 500.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 20.0f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 0.0f,
            },
        .velocity_damping_kd = 5.0f,
        .length_reference = 0.15f,
        .minimum_length = 0.07f,
        .maximum_length = 0.19f,
    };
    return config;
}

/**
 * @brief 创建已经完成通用 PID 初始化的单条腿运行对象。
 *
 * @return 默认打开且参数有效的腿长 PID 运行对象。
 */
static WheelLeggedLegLengthControl_t WheelLeggedLengthControlCreateDefault(void)
{
    WheelLeggedLegLengthControl_t control = {0};
    const WheelLeggedLegLengthControlInitConfig_t config = WheelLeggedLengthControlCreateDefaultConfig();
    WheelLeggedLegLengthControlInit(&control, &config);
    return control;
}

/**
 * @brief 记录一个浮点数近似相等断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param actual 实际值。
 * @param expected 期望值。
 */
static void WheelLeggedLengthControlExpectNear(WheelLeggedLengthControlSelfTestResult_t *result, uint32_t case_number,
                                               float actual, float expected)
{
    WheelLeggedLengthControlExpectTrue(result, case_number,
                                       isfinite(actual) && isfinite(expected) &&
                                           fabsf(actual - expected) <= LENGTH_CONTROL_TEST_EPSILON);
}

/**
 * @brief 记录一个布尔断言。
 *
 * @param result 当前测试统计结果。
 * @param case_number 用例编号。
 * @param condition 断言条件，非零代表通过。
 */
static void WheelLeggedLengthControlExpectTrue(WheelLeggedLengthControlSelfTestResult_t *result, uint32_t case_number,
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
