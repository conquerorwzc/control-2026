/**
 ******************************************************************************
 * @file    double_closed_loop_leg_test.c
 * @brief   双闭环腿纯几何模块主机端单测
 ******************************************************************************
 */
#include "double_closed_loop_leg_test.h"

#include <math.h>

#include "double_closed_loop_leg.h"

static void RecordCase(DoubleClosedLoopLegSelfTestResult_t *result, uint32_t case_id, int passed)
{
    if (passed)
    {
        result->passed_count++;
    }
    else
    {
        result->failed_count++;
        result->last_failed_case = case_id;
    }
}

/**
 * @brief 将角度差收敛到 [-pi, pi]，避免中心差分跨越角度边界时出现假大误差。
 *
 * @param angle 待收敛的角度差，单位 rad。
 * @return 收敛后的角度差，单位 rad。
 */
static float WrapAngleDifference(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

/**
 * @brief 用中心差分验证一组姿态下解析虚拟腿雅可比。
 *
 * @param geometry 双闭环机构几何。
 * @param input 待验证的 phi1、phi2。
 * @return 解析雅可比、局部速度关系和虚功关系均满足误差要求时返回 1。
 */
static int VerifyVirtualLegJacobian(const DoubleClosedLoopLegGeometry_t *geometry,
                                    const DoubleClosedLoopLegInput_t *input)
{
    const float epsilon = 1e-4f;
    const float jacobian_tolerance = 3e-3f;
    const float kinematics_tolerance = 2e-6f;
    const float virtual_work_tolerance = 2e-6f;
    const float force = 15.0f;
    const float pitch_torque = 0.35f;
    const float delta_phi1 = 2e-4f;
    const float delta_phi2 = -1.5e-4f;
    DoubleClosedLoopLegState_t state = {0};
    DoubleClosedLoopLegState_t plus_state = {0};
    DoubleClosedLoopLegState_t minus_state = {0};
    DoubleClosedLoopLegState_t displaced_state = {0};
    float numerical_jacobian[2][2] = {{0.0f}};

    if (DoubleClosedLoopLegForwardKinematics(geometry, input, &state) != DOUBLE_CLOSED_LOOP_LEG_OK ||
        state.virtual_leg_jacobian_valid == 0u)
    {
        return 0;
    }

    for (uint8_t input_index = 0u; input_index < 2u; input_index++)
    {
        DoubleClosedLoopLegInput_t plus_input = *input;
        DoubleClosedLoopLegInput_t minus_input = *input;
        if (input_index == 0u)
        {
            plus_input.phi1 += epsilon;
            minus_input.phi1 -= epsilon;
        }
        else
        {
            plus_input.phi2 += epsilon;
            minus_input.phi2 -= epsilon;
        }
        if (DoubleClosedLoopLegForwardKinematics(geometry, &plus_input, &plus_state) != DOUBLE_CLOSED_LOOP_LEG_OK ||
            DoubleClosedLoopLegForwardKinematics(geometry, &minus_input, &minus_state) != DOUBLE_CLOSED_LOOP_LEG_OK)
        {
            return 0;
        }
        numerical_jacobian[0][input_index] = (plus_state.length - minus_state.length) / (2.0f * epsilon);
        numerical_jacobian[1][input_index] =
            WrapAngleDifference(plus_state.virtual_leg_theta - minus_state.virtual_leg_theta) / (2.0f * epsilon);
        if (fabsf(numerical_jacobian[0][input_index] - state.virtual_leg_jacobian[0][input_index]) >
                jacobian_tolerance ||
            fabsf(numerical_jacobian[1][input_index] - state.virtual_leg_jacobian[1][input_index]) >
                jacobian_tolerance)
        {
            return 0;
        }
    }

    DoubleClosedLoopLegInput_t displaced_input = {
        .phi1 = input->phi1 + delta_phi1,
        .phi2 = input->phi2 + delta_phi2,
    };
    if (DoubleClosedLoopLegForwardKinematics(geometry, &displaced_input, &displaced_state) != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return 0;
    }

    const float predicted_length_change = state.virtual_leg_jacobian[0][0] * delta_phi1 +
                                          state.virtual_leg_jacobian[0][1] * delta_phi2;
    const float predicted_theta_change = state.virtual_leg_jacobian[1][0] * delta_phi1 +
                                         state.virtual_leg_jacobian[1][1] * delta_phi2;
    const float actual_length_change = displaced_state.length - state.length;
    const float actual_theta_change =
        WrapAngleDifference(displaced_state.virtual_leg_theta - state.virtual_leg_theta);
    if (fabsf(actual_length_change - predicted_length_change) > kinematics_tolerance ||
        fabsf(actual_theta_change - predicted_theta_change) > kinematics_tolerance)
    {
        return 0;
    }

    const float phi1_torque = state.virtual_leg_jacobian[0][0] * force +
                              state.virtual_leg_jacobian[1][0] * pitch_torque;
    const float phi2_torque = state.virtual_leg_jacobian[0][1] * force +
                              state.virtual_leg_jacobian[1][1] * pitch_torque;
    const float joint_virtual_work = phi1_torque * delta_phi1 + phi2_torque * delta_phi2;
    const float virtual_leg_work = force * predicted_length_change + pitch_torque * predicted_theta_change;
    return fabsf(joint_virtual_work - virtual_leg_work) <= virtual_work_tolerance;
}

DoubleClosedLoopLegSelfTestResult_t DoubleClosedLoopLegRunSelfTest(void)
{
    DoubleClosedLoopLegSelfTestResult_t result = {0};
    const DoubleClosedLoopLegGeometry_t geometry = {
        .configured = 1u,
        .oa_length = 0.100f,
        .oc_length = 0.100f,
        .ab_length = 0.100f,
        .bc_length = 0.100f,
        .bd_length = 0.050f,
        .cd_length = 0.132288f,
        .of_length = 0.200f,
        .cf_length = 0.100f,
        .de_length = 0.050f,
        .ef_length = 0.100f,
        .ep_length = 0.100f,
        .fp_length = 0.173205f,
        .first_loop_branch_sign = -1,
        .bcd_branch_sign = -1,
        .second_loop_branch_sign = -1,
        .efp_branch_sign = -1,
        .geometry_consistency_epsilon = 1e-5f,
        .singular_epsilon = 1e-5f,
    };
    DoubleClosedLoopLegInput_t input = {.phi1 = 1.57079632679489661923f, .phi2 = 0.0f};
    DoubleClosedLoopLegState_t state;

    DoubleClosedLoopLegStatus_e status = DoubleClosedLoopLegForwardKinematics(&geometry, &input, &state);
    RecordCase(&result, 1u,
               status == DOUBLE_CLOSED_LOOP_LEG_OK && state.first_loop_residual < 1e-5f &&
                   state.second_loop_residual < 1e-5f && isfinite(state.length) && isfinite(state.theta));

    input.phi1 += 0.02f;
    input.phi2 -= 0.02f;
    status = DoubleClosedLoopLegForwardKinematics(&geometry, &input, &state);
    RecordCase(&result, 2u,
               status == DOUBLE_CLOSED_LOOP_LEG_OK && state.first_loop_residual < 1e-5f &&
                   state.second_loop_residual < 1e-5f);

    DoubleClosedLoopLegGeometry_t unreachable_geometry = geometry;
    unreachable_geometry.ab_length = 0.020f;
    status = DoubleClosedLoopLegForwardKinematics(&unreachable_geometry, &input, &state);
    RecordCase(&result, 3u, status == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_UNREACHABLE);

    input.phi1 = 3.14159265358979323846f;
    input.phi2 = 0.0f;
    status = DoubleClosedLoopLegForwardKinematics(&geometry, &input, &state);
    RecordCase(&result, 4u, status == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR);

    DoubleClosedLoopLegGeometry_t unconfigured_geometry = geometry;
    unconfigured_geometry.configured = 0u;
    status = DoubleClosedLoopLegForwardKinematics(&unconfigured_geometry, &input, &state);
    RecordCase(&result, 5u, status == DOUBLE_CLOSED_LOOP_LEG_NOT_CONFIGURED);

    input.phi1 = 1.57079632679489661923f;
    input.phi2 = 0.0f;
    status = DoubleClosedLoopLegForwardKinematics(&geometry, &input, &state);
    DoubleClosedLoopLegInput_t ik_solution = {0};
    const DoubleClosedLoopLegInput_t previous_input = {.phi1 = input.phi1 + 0.02f, .phi2 = input.phi2 - 0.02f};
    if (status == DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        status = DoubleClosedLoopLegInverseKinematics(&geometry, &state.p, &previous_input, &ik_solution);
    }
    DoubleClosedLoopLegState_t ik_state = {0};
    if (status == DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        status = DoubleClosedLoopLegForwardKinematics(&geometry, &ik_solution, &ik_state);
    }
    RecordCase(&result, 6u,
               status == DOUBLE_CLOSED_LOOP_LEG_OK && hypotf(ik_state.p.x - state.p.x, ik_state.p.y - state.p.y) <
                                                           1e-5f);

    input.phi1 = 1.57079632679489661923f;
    input.phi2 = 0.0f;
    RecordCase(&result, 7u, VerifyVirtualLegJacobian(&geometry, &input));

    input.phi1 = 1.59079632679489661923f;
    input.phi2 = -0.020f;
    RecordCase(&result, 8u, VerifyVirtualLegJacobian(&geometry, &input));

    input.phi1 = 3.14159265358979323846f;
    input.phi2 = 0.0f;
    status = DoubleClosedLoopLegForwardKinematics(&geometry, &input, &state);
    RecordCase(&result, 9u,
               status == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR && state.virtual_leg_jacobian_valid == 0u);

    return result;
}
