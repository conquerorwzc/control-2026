/**
 ******************************************************************************
 * @file    chassis_lqr.c
 * @brief   双闭环轮腿十维影子 LQR 的固定工作点纯计算
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "chassis_lqr.h"

#include <math.h>
#include <string.h>

#include "double_closed_loop_lqr_coefficients.h"

/* Private define ------------------------------------------------------------*/
/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void WheelLeggedChassisLqrClearResult(WheelLeggedChassisLqr_t *lqr);
static uint8_t WheelLeggedChassisLqrIsInputValid(const float state_vector[WHEEL_LEGGED_LQR_STATE_COUNT],
                                                 uint8_t state_valid, uint8_t origin_captured,
                                                 float left_leg_length, float right_leg_length);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 初始化 LQR 运行对象。
 *
 * MATLAB 已使用的输入符号、K、x_ref、u0 均固定来自 0.160 m 工作点导出常量，
 * MCU 不做腿长调度或拟合。
 *
 * @param lqr 待初始化的影子 LQR 对象。
 */
void WheelLeggedChassisLqrInit(WheelLeggedChassisLqr_t *lqr)
{
    uint32_t input_index;

    if (lqr == NULL)
    {
        return;
    }

    memset(lqr, 0, sizeof(*lqr));
    for (input_index = 0u; input_index < WHEEL_LEGGED_LQR_INPUT_COUNT; input_index++)
    {
        lqr->input_sign[input_index] = k_double_closed_loop_lqr_input_sign[input_index];
    }
}

/**
 * @brief 使用当前十维状态执行一次固定 0.160 m 工作点的 LQR 计算。
 *
 * 本函数只发布 u = u0 - K*(x-x_ref) 到 lqr 对象。MATLAB 在导出 K 前已经把
 * input_sign 合并到输入矩阵，因此此处严禁再次对 Tp 或 Tw 额外取反；本函数不
 * 包含、更不调用任何电机接口。
 *
 * @param lqr 影子 LQR 运行对象。
 * @param state_vector 固定顺序十维状态 x。
 * @param state_valid 十维状态来源本周期是否完整有效。
 * @param origin_captured 十维状态零点是否已采集。
 * @param left_leg_length 左腿当前长度，单位 m。
 * @param right_leg_length 右腿当前长度，单位 m。
 */
void WheelLeggedChassisLqrUpdate(WheelLeggedChassisLqr_t *lqr,
                                 const float state_vector[WHEEL_LEGGED_LQR_STATE_COUNT], uint8_t state_valid,
                                 uint8_t origin_captured, float left_leg_length, float right_leg_length)
{
    uint32_t input_index;
    uint32_t state_index;

    if (lqr == NULL)
    {
        return;
    }

    WheelLeggedChassisLqrClearResult(lqr);
    lqr->update_count++;
    if (WheelLeggedChassisLqrIsInputValid(state_vector, state_valid, origin_captured, left_leg_length,
                                          right_leg_length) == 0u)
    {
        return;
    }

    lqr->left_leg_length = left_leg_length;
    lqr->right_leg_length = right_leg_length;
    for (state_index = 0u; state_index < WHEEL_LEGGED_LQR_STATE_COUNT; state_index++)
    {
        lqr->state_reference[state_index] = k_double_closed_loop_lqr_x_ref[state_index];
        lqr->state_error[state_index] = state_vector[state_index] - lqr->state_reference[state_index];
        if (!isfinite(lqr->state_reference[state_index]) || !isfinite(lqr->state_error[state_index]))
        {
            WheelLeggedChassisLqrClearResult(lqr);
            return;
        }
    }
    for (input_index = 0u; input_index < WHEEL_LEGGED_LQR_INPUT_COUNT; input_index++)
    {
        float feedback = 0.0f;
        lqr->trim_input[input_index] = k_double_closed_loop_lqr_u0[input_index];
        for (state_index = 0u; state_index < WHEEL_LEGGED_LQR_STATE_COUNT; state_index++)
        {
            lqr->gain[input_index][state_index] = k_double_closed_loop_lqr_nominal[input_index][state_index];
            feedback += lqr->gain[input_index][state_index] * lqr->state_error[state_index];
        }
        lqr->output[input_index] = lqr->trim_input[input_index] - feedback;
        if (!isfinite(feedback) || !isfinite(lqr->trim_input[input_index]) || !isfinite(lqr->output[input_index]))
        {
            WheelLeggedChassisLqrClearResult(lqr);
            return;
        }
    }
    lqr->tp_right = lqr->output[WHEEL_LEGGED_LQR_INPUT_TP_RIGHT];
    lqr->tp_left = lqr->output[WHEEL_LEGGED_LQR_INPUT_TP_LEFT];
    lqr->tw_right = lqr->output[WHEEL_LEGGED_LQR_INPUT_TW_RIGHT];
    lqr->tw_left = lqr->output[WHEEL_LEGGED_LQR_INPUT_TW_LEFT];
    lqr->valid = 1u;
}

/**
 * @brief 清空本周期影子 LQR 的计算结果，杜绝无效帧沿用上帧输出。
 *
 * @param lqr 待清空的影子 LQR 对象。
 */
static void WheelLeggedChassisLqrClearResult(WheelLeggedChassisLqr_t *lqr)
{
    if (lqr == NULL)
    {
        return;
    }

    lqr->valid = 0u;
    lqr->left_leg_length = 0.0f;
    lqr->right_leg_length = 0.0f;
    memset(lqr->state_reference, 0, sizeof(lqr->state_reference));
    memset(lqr->state_error, 0, sizeof(lqr->state_error));
    memset(lqr->gain, 0, sizeof(lqr->gain));
    memset(lqr->trim_input, 0, sizeof(lqr->trim_input));
    memset(lqr->output, 0, sizeof(lqr->output));
    lqr->tp_right = 0.0f;
    lqr->tp_left = 0.0f;
    lqr->tw_right = 0.0f;
    lqr->tw_left = 0.0f;
}

/**
 * @brief 检查影子 LQR 的状态、零点和腿长数值是否有效。
 *
 * @param state_vector 固定顺序十维状态。
 * @param state_valid 十维状态来源是否完整有效。
 * @param origin_captured 状态零点是否已经采集。
 * @param left_leg_length 左腿当前长度，单位 m。
 * @param right_leg_length 右腿当前长度，单位 m。
 * @return 状态、零点和腿长数值均有效时返回 1，否则返回 0。
 */
static uint8_t WheelLeggedChassisLqrIsInputValid(const float state_vector[WHEEL_LEGGED_LQR_STATE_COUNT],
                                                 uint8_t state_valid, uint8_t origin_captured,
                                                 float left_leg_length, float right_leg_length)
{
    uint32_t state_index;

    if (state_vector == NULL || state_valid == 0u || origin_captured == 0u || !isfinite(left_leg_length) ||
        !isfinite(right_leg_length))
    {
        return 0u;
    }
    for (state_index = 0u; state_index < WHEEL_LEGGED_LQR_STATE_COUNT; state_index++)
    {
        if (!isfinite(state_vector[state_index]))
        {
            return 0u;
        }
    }
    return 1u;
}
