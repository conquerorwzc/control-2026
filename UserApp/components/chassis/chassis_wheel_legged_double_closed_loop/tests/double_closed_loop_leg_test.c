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

    return result;
}
