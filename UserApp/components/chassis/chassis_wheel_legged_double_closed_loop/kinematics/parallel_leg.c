/**
 ******************************************************************************
 * @file    parallel_leg.c
 * @brief   平面同心五连杆纯运动学
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "parallel_leg.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/
typedef struct
{
    ParallelLegVec2_t point[2]; /* 两圆的两个交点候选。 */
    uint8_t count;              /* 有效候选点数量。 */
} ParallelLegCircleCandidates_t;

/* Private function prototypes -----------------------------------------------*/
static ParallelLegVec2_t ParallelLegVec2Subtract(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs);
static float ParallelLegVec2Norm(ParallelLegVec2_t vector);
static float ParallelLegVec2Cross(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs);
static float ParallelLegVec2Dot(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs);
static uint8_t ParallelLegIsFiniteVec2(ParallelLegVec2_t vector);
static uint8_t ParallelLegIsGeometryValid(const ParallelLegGeometry_t *geometry);
static ParallelLegStatus_e ParallelLegFindCircleCandidates(ParallelLegVec2_t center0, float radius0,
                                                           ParallelLegVec2_t center1, float radius1, float epsilon,
                                                           ParallelLegCircleCandidates_t *candidates);
static ParallelLegStatus_e ParallelLegSelectForwardBranch(const ParallelLegGeometry_t *geometry,
                                                          ParallelLegVec2_t first_end, ParallelLegVec2_t second_end,
                                                          const ParallelLegCircleCandidates_t *candidates,
                                                          ParallelLegVec2_t *end_effector);
static void ParallelLegCalculateRealLegJacobian(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input,
                                                ParallelLegState_t *state);
static float ParallelLegWrapAngleDifference(float angle);
static float ParallelLegInputDistanceSquared(const ParallelLegInput_t *lhs, const ParallelLegInput_t *rhs);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 计算两个二维点的差向量。
 *
 * @param lhs 被减点。
 * @param rhs 减点。
 * @return lhs-rhs。
 */
static ParallelLegVec2_t ParallelLegVec2Subtract(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs)
{
    const ParallelLegVec2_t result = {.x = lhs.x - rhs.x, .y = lhs.y - rhs.y};
    return result;
}

/**
 * @brief 计算二维向量模长。
 *
 * @param vector 待计算向量。
 * @return 向量模长。
 */
static float ParallelLegVec2Norm(ParallelLegVec2_t vector)
{
    return sqrtf(vector.x * vector.x + vector.y * vector.y);
}

/**
 * @brief 计算二维向量叉积。
 *
 * @param lhs 第一向量。
 * @param rhs 第二向量。
 * @return lhs×rhs 的 z 分量。
 */
static float ParallelLegVec2Cross(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs)
{
    return lhs.x * rhs.y - lhs.y * rhs.x;
}

/**
 * @brief 计算二维向量点积。
 *
 * @param lhs 第一向量。
 * @param rhs 第二向量。
 * @return lhs·rhs。
 */
static float ParallelLegVec2Dot(ParallelLegVec2_t lhs, ParallelLegVec2_t rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

/**
 * @brief 判断二维坐标是否均为有限数。
 *
 * @param vector 待检查坐标。
 * @return 有限返回 1，否则返回 0。
 */
static uint8_t ParallelLegIsFiniteVec2(ParallelLegVec2_t vector)
{
    return isfinite(vector.x) && isfinite(vector.y);
}

/**
 * @brief 检查同心五连杆配置是否可以参与运动学计算。
 *
 * @param geometry 待检查几何配置。
 * @return 有效返回 1，否则返回 0。
 */
static uint8_t ParallelLegIsGeometryValid(const ParallelLegGeometry_t *geometry)
{
    if (geometry == NULL || geometry->configured == 0u || !isfinite(geometry->l0) ||
        !isfinite(geometry->real_first_link_ah) || !isfinite(geometry->real_second_link_hj) ||
        !isfinite(geometry->virtual_second_link_ce) || !isfinite(geometry->singular_epsilon) ||
        geometry->real_first_link_ah <= 0.0f || geometry->real_second_link_hj <= 0.0f ||
        geometry->virtual_second_link_ce <= 0.0f || geometry->singular_epsilon <= 0.0f ||
        fabsf(geometry->l0) > geometry->singular_epsilon ||
        (geometry->virtual_end_branch_sign != 1 && geometry->virtual_end_branch_sign != -1))
    {
        return 0u;
    }

    const float scale_k = geometry->virtual_second_link_ce / geometry->real_second_link_hj;
    const float virtual_active_link_length = scale_k * geometry->real_first_link_ah;
    return isfinite(scale_k) && scale_k > 0.0f && isfinite(virtual_active_link_length) &&
           virtual_active_link_length > geometry->singular_epsilon;
}

/**
 * @brief 求两个圆的交点候选，不在此函数选择机构装配支路。
 *
 * @param center0 第一圆圆心。
 * @param radius0 第一圆半径，单位 m。
 * @param center1 第二圆圆心。
 * @param radius1 第二圆半径，单位 m。
 * @param epsilon 奇异判据，单位 m。
 * @param candidates 返回的交点候选。
 * @return 可达时返回成功；圆心重合或相切返回奇异。
 */
static ParallelLegStatus_e ParallelLegFindCircleCandidates(ParallelLegVec2_t center0, float radius0,
                                                           ParallelLegVec2_t center1, float radius1, float epsilon,
                                                           ParallelLegCircleCandidates_t *candidates)
{
    if (candidates == NULL || !ParallelLegIsFiniteVec2(center0) || !ParallelLegIsFiniteVec2(center1) ||
        !isfinite(radius0) || !isfinite(radius1) || !isfinite(epsilon) || radius0 <= 0.0f || radius1 <= 0.0f ||
        epsilon <= 0.0f)
    {
        return PARALLEL_LEG_INVALID_ARGUMENT;
    }

    memset(candidates, 0, sizeof(*candidates));
    const ParallelLegVec2_t delta = ParallelLegVec2Subtract(center1, center0);
    const float distance = ParallelLegVec2Norm(delta);
    if (!isfinite(distance))
    {
        return PARALLEL_LEG_NUMERIC_ERROR;
    }
    if (distance <= epsilon)
    {
        return PARALLEL_LEG_SINGULAR;
    }
    if (distance > radius0 + radius1 + epsilon || distance < fabsf(radius0 - radius1) - epsilon)
    {
        return PARALLEL_LEG_UNREACHABLE;
    }

    const float projection = (radius0 * radius0 - radius1 * radius1 + distance * distance) / (2.0f * distance);
    float height_squared = radius0 * radius0 - projection * projection;
    if (height_squared < -epsilon * epsilon)
    {
        return PARALLEL_LEG_UNREACHABLE;
    }
    if (height_squared <= epsilon * epsilon)
    {
        return PARALLEL_LEG_SINGULAR;
    }
    if (height_squared < 0.0f)
    {
        height_squared = 0.0f;
    }

    const float inverse_distance = 1.0f / distance;
    const ParallelLegVec2_t base = {.x = center0.x + projection * delta.x * inverse_distance,
                                    .y = center0.y + projection * delta.y * inverse_distance};
    const float height = sqrtf(height_squared);
    const ParallelLegVec2_t perpendicular = {.x = -delta.y * inverse_distance, .y = delta.x * inverse_distance};
    candidates->point[0] =
        (ParallelLegVec2_t){.x = base.x + height * perpendicular.x, .y = base.y + height * perpendicular.y};
    candidates->point[1] =
        (ParallelLegVec2_t){.x = base.x - height * perpendicular.x, .y = base.y - height * perpendicular.y};
    candidates->count = 2u;
    return ParallelLegIsFiniteVec2(candidates->point[0]) && ParallelLegIsFiniteVec2(candidates->point[1])
               ? PARALLEL_LEG_OK
               : PARALLEL_LEG_NUMERIC_ERROR;
}

/**
 * @brief 从两个圆交候选中按配置叉积符号选择虚拟末端 C 的装配支路。
 *
 * @param geometry 五连杆几何与支路配置。
 * @param first_end phi1 对应虚拟主动杆末端 D。
 * @param second_end phi2 对应虚拟主动杆末端 E。
 * @param candidates 两个圆交候选。
 * @param end_effector 返回选中的虚拟末端 C。
 * @return 支路匹配时返回成功。
 */
static ParallelLegStatus_e ParallelLegSelectForwardBranch(const ParallelLegGeometry_t *geometry,
                                                          ParallelLegVec2_t first_end, ParallelLegVec2_t second_end,
                                                          const ParallelLegCircleCandidates_t *candidates,
                                                          ParallelLegVec2_t *end_effector)
{
    if (geometry == NULL || candidates == NULL || end_effector == NULL || candidates->count != 2u)
    {
        return PARALLEL_LEG_INVALID_ARGUMENT;
    }

    const ParallelLegVec2_t first_to_second = ParallelLegVec2Subtract(second_end, first_end);
    const float cross_epsilon = geometry->singular_epsilon * geometry->singular_epsilon;
    for (uint8_t index = 0u; index < candidates->count; index++)
    {
        const ParallelLegVec2_t first_to_candidate = ParallelLegVec2Subtract(candidates->point[index], first_end);
        const float cross = ParallelLegVec2Cross(first_to_second, first_to_candidate);
        if ((cross > cross_epsilon && geometry->virtual_end_branch_sign > 0) ||
            (cross < -cross_epsilon && geometry->virtual_end_branch_sign < 0))
        {
            *end_effector = candidates->point[index];
            return PARALLEL_LEG_OK;
        }
    }
    return PARALLEL_LEG_BRANCH_MISMATCH;
}

/**
 * @brief 通过虚拟五连杆闭环约束微分计算真实末端 J 的雅可比。
 *
 * @param geometry 五连杆几何参数。
 * @param input 当前 phi1、phi2。
 * @param state 已完成 FK 的机构状态；函数回填雅可比字段。
 */
static void ParallelLegCalculateRealLegJacobian(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input,
                                                ParallelLegState_t *state)
{
    if (geometry == NULL || input == NULL || state == NULL || state->length <= geometry->singular_epsilon ||
        !isfinite(state->scale_k) || state->scale_k <= 0.0f)
    {
        return;
    }

    const ParallelLegVec2_t c_minus_d = ParallelLegVec2Subtract(state->virtual_end_c, state->virtual_first_end_d);
    const ParallelLegVec2_t c_minus_e = ParallelLegVec2Subtract(state->virtual_end_c, state->virtual_second_end_e);
    const float constraint_det = ParallelLegVec2Cross(c_minus_d, c_minus_e);
    const float constraint_det_epsilon = geometry->singular_epsilon * geometry->singular_epsilon;
    if (!isfinite(constraint_det) || fabsf(constraint_det) <= constraint_det_epsilon)
    {
        return;
    }

    const float virtual_active_link_length = state->scale_k * geometry->real_first_link_ah; /* AD=AE。 */
    const ParallelLegVec2_t d_derivative = {
        .x = -virtual_active_link_length * sinf(input->phi1),
        .y = virtual_active_link_length * cosf(input->phi1),
    };
    const ParallelLegVec2_t e_derivative = {
        .x = -virtual_active_link_length * sinf(input->phi2),
        .y = virtual_active_link_length * cosf(input->phi2),
    };
    const float rhs[2][2] = {
        {ParallelLegVec2Dot(c_minus_d, d_derivative), 0.0f},
        {0.0f, ParallelLegVec2Dot(c_minus_e, e_derivative)},
    };
    const float inverse_constraint_det = 1.0f / constraint_det;
    for (uint8_t input_index = 0u; input_index < 2u; input_index++)
    {
        const ParallelLegVec2_t c_derivative = {
            .x = (rhs[0][input_index] * c_minus_e.y - c_minus_d.y * rhs[1][input_index]) * inverse_constraint_det,
            .y = (c_minus_d.x * rhs[1][input_index] - rhs[0][input_index] * c_minus_e.x) * inverse_constraint_det,
        };
        const ParallelLegVec2_t j_derivative = {
            .x = c_derivative.x / state->scale_k,
            .y = c_derivative.y / state->scale_k,
        };
        state->real_leg_jacobian[0][input_index] =
            (state->real_end_j.x * j_derivative.x + state->real_end_j.y * j_derivative.y) / state->length;
        state->real_leg_jacobian[1][input_index] =
            (-state->real_end_j.y * j_derivative.x + state->real_end_j.x * j_derivative.y) /
            (state->length * state->length);
    }
    state->real_leg_jacobian_det = state->real_leg_jacobian[0][0] * state->real_leg_jacobian[1][1] -
                                   state->real_leg_jacobian[0][1] * state->real_leg_jacobian[1][0];
    state->real_leg_jacobian_valid = isfinite(state->real_leg_jacobian[0][0]) &&
                                     isfinite(state->real_leg_jacobian[0][1]) &&
                                     isfinite(state->real_leg_jacobian[1][0]) &&
                                     isfinite(state->real_leg_jacobian[1][1]) && isfinite(state->real_leg_jacobian_det);
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
 * @brief 计算两组主动角的周期等价距离平方。
 *
 * @param lhs 第一组主动角。
 * @param rhs 第二组主动角。
 * @return 考虑 2pi 周期后的距离平方。
 */
static float ParallelLegInputDistanceSquared(const ParallelLegInput_t *lhs, const ParallelLegInput_t *rhs)
{
    const float delta_phi1 = ParallelLegWrapAngleDifference(lhs->phi1 - rhs->phi1);
    const float delta_phi2 = ParallelLegWrapAngleDifference(lhs->phi2 - rhs->phi2);
    return delta_phi1 * delta_phi1 + delta_phi2 * delta_phi2;
}

/**
 * @brief 根据两个主动角计算 ACE 虚拟 C、真实末端 J、腿长和摆角。
 *
 * @param geometry 同心五连杆几何配置。
 * @param input 主动轴角 phi1、phi2。
 * @param state 返回机构状态。
 * @return 本次 FK 状态。
 */
ParallelLegStatus_e ParallelLegForwardKinematics(const ParallelLegGeometry_t *geometry, const ParallelLegInput_t *input,
                                                 ParallelLegState_t *state)
{
    if (input == NULL || state == NULL || !isfinite(input->phi1) || !isfinite(input->phi2))
    {
        return PARALLEL_LEG_INVALID_ARGUMENT;
    }
    if (!ParallelLegIsGeometryValid(geometry))
    {
        return geometry != NULL && geometry->configured == 0u ? PARALLEL_LEG_NOT_CONFIGURED
                                                              : PARALLEL_LEG_INVALID_GEOMETRY;
    }

    memset(state, 0, sizeof(*state));
    state->o1 = (ParallelLegVec2_t){0.0f, 0.0f};
    state->o2 = (ParallelLegVec2_t){.x = geometry->l0, .y = 0.0f};
    state->scale_k = geometry->virtual_second_link_ce / geometry->real_second_link_hj;
    const float virtual_active_link_length = state->scale_k * geometry->real_first_link_ah; /* AD=AE。 */
    state->virtual_first_end_d = (ParallelLegVec2_t){.x = virtual_active_link_length * cosf(input->phi1),
                                                     .y = virtual_active_link_length * sinf(input->phi1)};
    state->virtual_second_end_e = (ParallelLegVec2_t){.x = state->o2.x + virtual_active_link_length * cosf(input->phi2),
                                                      .y = virtual_active_link_length * sinf(input->phi2)};
    state->real_second_end_h = (ParallelLegVec2_t){.x = state->o2.x + geometry->real_first_link_ah * cosf(input->phi2),
                                                   .y = geometry->real_first_link_ah * sinf(input->phi2)};

    ParallelLegCircleCandidates_t candidates = {0};
    ParallelLegStatus_e status = ParallelLegFindCircleCandidates(
        state->virtual_first_end_d, geometry->virtual_second_link_ce, state->virtual_second_end_e,
        geometry->virtual_second_link_ce, geometry->singular_epsilon, &candidates);
    if (status != PARALLEL_LEG_OK)
    {
        return status;
    }
    status = ParallelLegSelectForwardBranch(geometry, state->virtual_first_end_d, state->virtual_second_end_e,
                                            &candidates, &state->virtual_end_c);
    if (status != PARALLEL_LEG_OK)
    {
        return status;
    }

    state->real_end_j =
        (ParallelLegVec2_t){.x = state->virtual_end_c.x / state->scale_k, .y = state->virtual_end_c.y / state->scale_k};
    state->length = ParallelLegVec2Norm(state->real_end_j);
    if (!isfinite(state->length) || state->length <= geometry->singular_epsilon)
    {
        return PARALLEL_LEG_SINGULAR;
    }
    state->theta = atan2f(state->real_end_j.x, state->real_end_j.y);
    state->virtual_leg_theta = atan2f(-state->real_end_j.x, state->real_end_j.y);
    state->first_loop_residual =
        fabsf(ParallelLegVec2Norm(ParallelLegVec2Subtract(state->virtual_end_c, state->virtual_first_end_d)) -
              geometry->virtual_second_link_ce);
    state->second_loop_residual =
        fabsf(ParallelLegVec2Norm(ParallelLegVec2Subtract(state->virtual_end_c, state->virtual_second_end_e)) -
              geometry->virtual_second_link_ce);
    ParallelLegCalculateRealLegJacobian(geometry, input, state);
    return ParallelLegIsFiniteVec2(state->virtual_end_c) && ParallelLegIsFiniteVec2(state->real_second_end_h) &&
                   ParallelLegIsFiniteVec2(state->real_end_j) && isfinite(state->scale_k) && isfinite(state->theta) &&
                   isfinite(state->virtual_leg_theta) && isfinite(state->first_loop_residual) &&
                   isfinite(state->second_loop_residual)
               ? PARALLEL_LEG_OK
               : PARALLEL_LEG_NUMERIC_ERROR;
}

/**
 * @brief 根据真实末端 J 反解两主动轴角，并按上一帧选择连续支路。
 *
 * @param geometry 同心五连杆几何配置。
 * @param target_j 目标真实末端 J 位置。
 * @param previous_input 上一帧主动角，用于选择连续解。
 * @param solution 返回 phi1、phi2。
 * @return 本次 IK 状态。
 */
ParallelLegStatus_e ParallelLegInverseKinematics(const ParallelLegGeometry_t *geometry,
                                                 const ParallelLegVec2_t *target_j,
                                                 const ParallelLegInput_t *previous_input, ParallelLegInput_t *solution)
{
    if (target_j == NULL || previous_input == NULL || solution == NULL || !ParallelLegIsFiniteVec2(*target_j) ||
        !isfinite(previous_input->phi1) || !isfinite(previous_input->phi2))
    {
        return PARALLEL_LEG_INVALID_ARGUMENT;
    }
    if (!ParallelLegIsGeometryValid(geometry))
    {
        return geometry != NULL && geometry->configured == 0u ? PARALLEL_LEG_NOT_CONFIGURED
                                                              : PARALLEL_LEG_INVALID_GEOMETRY;
    }

    const float scale_k = geometry->virtual_second_link_ce / geometry->real_second_link_hj;
    const float virtual_active_link_length = scale_k * geometry->real_first_link_ah; /* AD=AE。 */
    const ParallelLegVec2_t target_c = {.x = scale_k * target_j->x, .y = scale_k * target_j->y};
    ParallelLegCircleCandidates_t first_candidates = {0};
    ParallelLegCircleCandidates_t second_candidates = {0};
    ParallelLegStatus_e status = ParallelLegFindCircleCandidates(
        (ParallelLegVec2_t){0.0f, 0.0f}, virtual_active_link_length, target_c, geometry->virtual_second_link_ce,
        geometry->singular_epsilon, &first_candidates);
    if (status != PARALLEL_LEG_OK)
    {
        return status;
    }
    status = ParallelLegFindCircleCandidates((ParallelLegVec2_t){.x = geometry->l0, .y = 0.0f},
                                             virtual_active_link_length, target_c, geometry->virtual_second_link_ce,
                                             geometry->singular_epsilon, &second_candidates);
    if (status != PARALLEL_LEG_OK)
    {
        return status;
    }

    float best_cost = INFINITY;
    uint8_t found_solution = 0u;
    for (uint8_t first_index = 0u; first_index < first_candidates.count; first_index++)
    {
        for (uint8_t second_index = 0u; second_index < second_candidates.count; second_index++)
        {
            const ParallelLegInput_t candidate = {
                .phi1 = atan2f(first_candidates.point[first_index].y, first_candidates.point[first_index].x),
                .phi2 = atan2f(second_candidates.point[second_index].y,
                               second_candidates.point[second_index].x - geometry->l0),
            };
            ParallelLegState_t candidate_state = {0};
            if (ParallelLegForwardKinematics(geometry, &candidate, &candidate_state) != PARALLEL_LEG_OK ||
                ParallelLegVec2Norm(ParallelLegVec2Subtract(candidate_state.real_end_j, *target_j)) >
                    geometry->singular_epsilon * 10.0f)
            {
                continue;
            }
            const float cost = ParallelLegInputDistanceSquared(&candidate, previous_input);
            if (cost < best_cost)
            {
                best_cost = cost;
                *solution = candidate;
                found_solution = 1u;
            }
        }
    }
    return found_solution != 0u ? PARALLEL_LEG_OK : PARALLEL_LEG_BRANCH_MISMATCH;
}

/**
 * @brief 绑定 ACE 显式 k 缩放同心五连杆配置并清空运行状态。
 *
 * @param instance 待初始化的机构学实例。
 * @param config 只读运行配置。
 */
void ParallelLegInit(ParallelLegInstance_t *instance, const ParallelLegConfig_t *config)
{
    if (instance == NULL)
    {
        return;
    }
    memset(instance, 0, sizeof(*instance));
    instance->config = config;
    instance->transmission_status[0] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
    instance->transmission_status[1] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
    instance->forward_kinematics_status = PARALLEL_LEG_INVALID_ARGUMENT;
}

/**
 * @brief 完成电机角到主动轴角换算，并更新 ACE 显式 k 缩放五连杆 FK。
 *
 * @param instance 机构学实例。
 * @param actuator_angle_0 phi1 对应电机的累计角，单位 rad。
 * @param actuator_angle_1 phi2 对应电机的累计角，单位 rad。
 */
void ParallelLegUpdate(ParallelLegInstance_t *instance, float actuator_angle_0, float actuator_angle_1)
{
    if (instance == NULL || instance->config == NULL)
    {
        return;
    }
    instance->actuator_angle[0] = actuator_angle_0;
    instance->actuator_angle[1] = actuator_angle_1;
    instance->transmission_status[0] =
        JointTransmissionMotorToJoint(&instance->config->transmission[0], actuator_angle_0, &instance->phi[0]);
    instance->transmission_status[1] =
        JointTransmissionMotorToJoint(&instance->config->transmission[1], actuator_angle_1, &instance->phi[1]);
    memset(&instance->state, 0, sizeof(instance->state));
    if (instance->transmission_status[0] != JOINT_TRANSMISSION_OK ||
        instance->transmission_status[1] != JOINT_TRANSMISSION_OK)
    {
        instance->forward_kinematics_status = PARALLEL_LEG_NOT_CONFIGURED;
        return;
    }
    const ParallelLegInput_t input = {.phi1 = instance->phi[0], .phi2 = instance->phi[1]};
    instance->forward_kinematics_status =
        ParallelLegForwardKinematics(&instance->config->geometry, &input, &instance->state);
}
