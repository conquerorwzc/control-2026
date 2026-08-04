/**
 ******************************************************************************
 * @file    parallel_leg_test.c
 * @brief   同心五连杆纯几何模块主机端单测
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "parallel_leg_test.h"

#include <math.h>

#include "parallel_leg.h"

/* Private define ------------------------------------------------------------*/
/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/
static void ParallelLegRecordCase(ParallelLegSelfTestResult_t *result, uint32_t case_id, uint8_t passed);
static float ParallelLegWrapAngleDifference(float angle);
static uint8_t ParallelLegVerifyRealLegJacobian(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input);
static uint8_t ParallelLegVerifyInverseRoundTrip(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 记录单个测试用例的结果。
 *
 * @param result 汇总结果。
 * @param case_id 测试用例编号。
 * @param passed 用例通过时为 1。
 */
static void ParallelLegRecordCase(ParallelLegSelfTestResult_t *result, uint32_t case_id, uint8_t passed)
{
    if (passed != 0u)
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
 * @brief 将角度差收敛至 [-pi, pi]。
 *
 * @param angle 待收敛的角度差，单位 rad。
 * @return 收敛后的角度差，单位 rad。
 */
static float ParallelLegWrapAngleDifference(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

/**
 * @brief 用中心差分、微小位移和虚功恒等式验证真实末端 J 的解析雅可比。
 *
 * @param geometry 五连杆几何配置。
 * @param input 参与验证的一组主动角。
 * @return 全部误差在容差内时返回 1。
 */
static uint8_t ParallelLegVerifyRealLegJacobian(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input)
{
    const float epsilon = 1e-4f;
    const float jacobian_tolerance = 3e-3f;
    const float local_kinematics_tolerance = 3e-6f;
    const float virtual_work_tolerance = 3e-6f;
    const float force = 15.0f;
    const float pitch_torque = 0.35f;
    const float delta_phi1 = 2e-4f;
    const float delta_phi2 = -1.5e-4f;
    ParallelLegState_t state = {0};
    ParallelLegState_t plus_state = {0};
    ParallelLegState_t minus_state = {0};
    ParallelLegState_t displaced_state = {0};
    float numerical_jacobian[2][2] = {{0.0f}};

    if (ParallelLegForwardKinematics(geometry, input, &state) != PARALLEL_LEG_OK || state.real_leg_jacobian_valid == 0u)
    {
        return 0u;
    }
    for (uint8_t input_index = 0u; input_index < 2u; input_index++)
    {
        ParallelLegInput_t plus_input = *input;
        ParallelLegInput_t minus_input = *input;
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
        if (ParallelLegForwardKinematics(geometry, &plus_input, &plus_state) != PARALLEL_LEG_OK ||
            ParallelLegForwardKinematics(geometry, &minus_input, &minus_state) != PARALLEL_LEG_OK)
        {
            return 0u;
        }
        numerical_jacobian[0][input_index] = (plus_state.length - minus_state.length) / (2.0f * epsilon);
        numerical_jacobian[1][input_index] =
            ParallelLegWrapAngleDifference(plus_state.virtual_leg_theta - minus_state.virtual_leg_theta) /
            (2.0f * epsilon);
        if (fabsf(numerical_jacobian[0][input_index] - state.real_leg_jacobian[0][input_index]) > jacobian_tolerance ||
            fabsf(numerical_jacobian[1][input_index] - state.real_leg_jacobian[1][input_index]) > jacobian_tolerance)
        {
            return 0u;
        }
    }

    const ParallelLegInput_t displaced_input = {
        .phi1 = input->phi1 + delta_phi1,
        .phi2 = input->phi2 + delta_phi2,
    };
    if (ParallelLegForwardKinematics(geometry, &displaced_input, &displaced_state) != PARALLEL_LEG_OK)
    {
        return 0u;
    }
    const float predicted_length_change =
        state.real_leg_jacobian[0][0] * delta_phi1 + state.real_leg_jacobian[0][1] * delta_phi2;
    const float predicted_theta_change =
        state.real_leg_jacobian[1][0] * delta_phi1 + state.real_leg_jacobian[1][1] * delta_phi2;
    const float actual_length_change = displaced_state.length - state.length;
    const float actual_theta_change =
        ParallelLegWrapAngleDifference(displaced_state.virtual_leg_theta - state.virtual_leg_theta);
    if (fabsf(actual_length_change - predicted_length_change) > local_kinematics_tolerance ||
        fabsf(actual_theta_change - predicted_theta_change) > local_kinematics_tolerance)
    {
        return 0u;
    }

    const float phi1_torque = state.real_leg_jacobian[0][0] * force + state.real_leg_jacobian[1][0] * pitch_torque;
    const float phi2_torque = state.real_leg_jacobian[0][1] * force + state.real_leg_jacobian[1][1] * pitch_torque;
    const float joint_work = phi1_torque * delta_phi1 + phi2_torque * delta_phi2;
    const float virtual_work = force * predicted_length_change + pitch_torque * predicted_theta_change;
    return fabsf(joint_work - virtual_work) <= virtual_work_tolerance;
}

/**
 * @brief 验证指定几何和主动角的真实 J 点 FK -> IK -> FK 回代关系。
 *
 * @param geometry 待验证的 ACE 五连杆几何参数。
 * @param input 原始主动轴角 phi1、phi2。
 * @return 回代状态、主动角连续性和真实 J 位置误差均满足容差时返回 1。
 */
static uint8_t ParallelLegVerifyInverseRoundTrip(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input)
{
    const float phi_tolerance = 1e-4f;
    const float position_tolerance = 1e-5f;
    ParallelLegState_t forward_state = {0};
    ParallelLegInput_t solution = {0};
    ParallelLegState_t backward_state = {0};

    if (ParallelLegForwardKinematics(geometry, input, &forward_state) != PARALLEL_LEG_OK ||
        ParallelLegInverseKinematics(geometry, &forward_state.real_end_j, input, &solution) != PARALLEL_LEG_OK ||
        ParallelLegForwardKinematics(geometry, &solution, &backward_state) != PARALLEL_LEG_OK)
    {
        return 0u;
    }

    const float phi1_error = ParallelLegWrapAngleDifference(solution.phi1 - input->phi1);
    const float phi2_error = ParallelLegWrapAngleDifference(solution.phi2 - input->phi2);
    const float delta_x = backward_state.real_end_j.x - forward_state.real_end_j.x;
    const float delta_y = backward_state.real_end_j.y - forward_state.real_end_j.y;
    const float position_error = sqrtf(delta_x * delta_x + delta_y * delta_y);
    return fabsf(phi1_error) <= phi_tolerance && fabsf(phi2_error) <= phi_tolerance &&
           position_error <= position_tolerance;
}

/**
 * @brief 执行同心五连杆的全部主机端自测。
 *
 * @return 所有用例的汇总结果。
 */
ParallelLegSelfTestResult_t ParallelLegRunSelfTest(void)
{
    ParallelLegSelfTestResult_t result = {0};
    const ParallelLegGeometry_t geometry = {
        .configured = 1u,
        .l0 = 0.0f,
        .real_first_link_ah = 0.100f,
        .real_second_link_hj = 0.100f,
        .virtual_second_link_ce = 0.050f,
        .virtual_end_branch_sign = -1,
        .singular_epsilon = 1e-5f,
    };
    ParallelLegInput_t input = {.phi1 = 0.8f, .phi2 = 2.2f};
    ParallelLegState_t state = {0};
    ParallelLegStatus_e status = ParallelLegForwardKinematics(&geometry, &input, &state);
    const float scale_position_error = sqrtf((state.virtual_end_c.x - state.scale_k * state.real_end_j.x) *
                                                 (state.virtual_end_c.x - state.scale_k * state.real_end_j.x) +
                                             (state.virtual_end_c.y - state.scale_k * state.real_end_j.y) *
                                                 (state.virtual_end_c.y - state.scale_k * state.real_end_j.y));
    ParallelLegRecordCase(&result, 1u,
                          status == PARALLEL_LEG_OK && fabsf(state.scale_k - 0.5f) < 1e-6f &&
                              scale_position_error < 1e-6f && state.first_loop_residual < 1e-5f &&
                              state.second_loop_residual < 1e-5f && isfinite(state.length));
    ParallelLegRecordCase(&result, 2u, ParallelLegVerifyRealLegJacobian(&geometry, &input));

    ParallelLegInput_t ik_solution = {0};
    const ParallelLegInput_t previous_input = {.phi1 = input.phi1 + 0.02f, .phi2 = input.phi2 - 0.02f};
    ParallelLegState_t ik_state = {0};
    status = ParallelLegInverseKinematics(&geometry, &state.real_end_j, &previous_input, &ik_solution);
    if (status == PARALLEL_LEG_OK)
    {
        status = ParallelLegForwardKinematics(&geometry, &ik_solution, &ik_state);
    }
    const float ik_position_error =
        sqrtf((ik_state.real_end_j.x - state.real_end_j.x) * (ik_state.real_end_j.x - state.real_end_j.x) +
              (ik_state.real_end_j.y - state.real_end_j.y) * (ik_state.real_end_j.y - state.real_end_j.y));
    ParallelLegRecordCase(&result, 3u, status == PARALLEL_LEG_OK && ik_position_error < 1e-5f);

    ParallelLegGeometry_t unreachable_geometry = geometry;
    unreachable_geometry.real_first_link_ah = 0.500f;
    status = ParallelLegForwardKinematics(&unreachable_geometry, &input, &state);
    ParallelLegRecordCase(&result, 4u, status == PARALLEL_LEG_UNREACHABLE);

    input.phi2 = input.phi1;
    status = ParallelLegForwardKinematics(&geometry, &input, &state);
    ParallelLegRecordCase(&result, 5u, status == PARALLEL_LEG_SINGULAR);

    ParallelLegGeometry_t unconfigured_geometry = geometry;
    unconfigured_geometry.configured = 0u;
    status = ParallelLegForwardKinematics(&unconfigured_geometry, &input, &state);
    ParallelLegRecordCase(&result, 6u, status == PARALLEL_LEG_NOT_CONFIGURED);

    ParallelLegGeometry_t invalid_hj_geometry = geometry;
    invalid_hj_geometry.real_second_link_hj = 0.0f;
    status = ParallelLegForwardKinematics(&invalid_hj_geometry, &input, &state);
    ParallelLegRecordCase(&result, 7u, status == PARALLEL_LEG_INVALID_GEOMETRY);

    ParallelLegGeometry_t invalid_ce_geometry = geometry;
    invalid_ce_geometry.virtual_second_link_ce = 0.0f;
    status = ParallelLegForwardKinematics(&invalid_ce_geometry, &input, &state);
    ParallelLegRecordCase(&result, 8u, status == PARALLEL_LEG_INVALID_GEOMETRY);

    ParallelLegGeometry_t non_finite_scale_geometry = geometry;
    non_finite_scale_geometry.virtual_second_link_ce = INFINITY;
    status = ParallelLegForwardKinematics(&non_finite_scale_geometry, &input, &state);
    ParallelLegRecordCase(&result, 9u, status == PARALLEL_LEG_INVALID_GEOMETRY);

    ParallelLegGeometry_t tangent_geometry = geometry;
    tangent_geometry.singular_epsilon = 1e-4f; /* 吸收 float pi 在相切姿态下的舍入误差。 */
    const ParallelLegInput_t tangent_input = {.phi1 = 0.0f, .phi2 = 3.14159265358979323846f};
    status = ParallelLegForwardKinematics(&tangent_geometry, &tangent_input, &state);
    ParallelLegRecordCase(&result, 10u, status == PARALLEL_LEG_SINGULAR);

    /* 当前 robot_config.h 的 ACE 实物几何；几何标定变更时同步更新本测试。 */
    const ParallelLegGeometry_t current_robot_geometry = {
        .configured = 1u,
        .l0 = 0.0f,
        .real_first_link_ah = 0.105f,
        .real_second_link_hj = 0.125f,
        .virtual_second_link_ce = 0.0625f,
        .virtual_end_branch_sign = -1,
        .singular_epsilon = 1e-5f,
    };
    const ParallelLegInput_t current_robot_inputs[] = {
        {.phi1 = 0.80f, .phi2 = 2.20f}, /* 代表性非奇异姿态一。 */
        {.phi1 = 1.05f, .phi2 = 2.00f}, /* 代表性非奇异姿态二。 */
        {.phi1 = 0.55f, .phi2 = 2.40f}, /* 代表性非奇异姿态三。 */
    };
    ParallelLegRecordCase(&result, 11u,
                          ParallelLegVerifyRealLegJacobian(&current_robot_geometry, &current_robot_inputs[0]));
    ParallelLegRecordCase(&result, 12u,
                          ParallelLegVerifyInverseRoundTrip(&current_robot_geometry, &current_robot_inputs[0]));
    ParallelLegRecordCase(&result, 13u,
                          ParallelLegVerifyInverseRoundTrip(&current_robot_geometry, &current_robot_inputs[1]));
    ParallelLegRecordCase(&result, 14u,
                          ParallelLegVerifyInverseRoundTrip(&current_robot_geometry, &current_robot_inputs[2]));
    return result;
}
