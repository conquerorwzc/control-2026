/**
 ******************************************************************************
 * @file    double_closed_loop_leg.c
 * @brief   平面双闭环腿纯正运动学
 ******************************************************************************
 */
#include "double_closed_loop_leg.h"

#include <math.h>
#include <string.h>

static float Vec2Norm(DoubleClosedLoopLegVec2_t vector)
{
    return sqrtf(vector.x * vector.x + vector.y * vector.y);
}

static DoubleClosedLoopLegVec2_t Vec2Subtract(DoubleClosedLoopLegVec2_t lhs, DoubleClosedLoopLegVec2_t rhs)
{
    DoubleClosedLoopLegVec2_t result = {.x = lhs.x - rhs.x, .y = lhs.y - rhs.y};
    return result;
}

static DoubleClosedLoopLegVec2_t Vec2Rotate(DoubleClosedLoopLegVec2_t vector, float angle)
{
    const float cosine = cosf(angle);
    const float sine = sinf(angle);
    const DoubleClosedLoopLegVec2_t result = {
        .x = cosine * vector.x - sine * vector.y,
        .y = sine * vector.x + cosine * vector.y,
    };
    return result;
}

static float Vec2Cross(DoubleClosedLoopLegVec2_t lhs, DoubleClosedLoopLegVec2_t rhs)
{
    return lhs.x * rhs.y - lhs.y * rhs.x;
}

static int IsFiniteVec2(DoubleClosedLoopLegVec2_t vector)
{
    return isfinite(vector.x) && isfinite(vector.y);
}

static int IsValidBranchSign(int8_t sign)
{
    return sign == 1 || sign == -1;
}

static int IsValidGeometry(const DoubleClosedLoopLegGeometry_t *geometry)
{
    if (geometry == NULL || !geometry->configured || geometry->oa_length <= 0.0f || geometry->oc_length <= 0.0f ||
        geometry->ab_length <= 0.0f || geometry->bc_length <= 0.0f || geometry->bd_length <= 0.0f ||
        geometry->cd_length <= 0.0f || geometry->of_length <= 0.0f || geometry->cf_length <= 0.0f ||
        geometry->de_length <= 0.0f || geometry->ef_length <= 0.0f || geometry->ep_length <= 0.0f ||
        geometry->fp_length <= 0.0f || geometry->geometry_consistency_epsilon <= 0.0f ||
        geometry->singular_epsilon <= 0.0f || !IsValidBranchSign(geometry->first_loop_branch_sign) ||
        !IsValidBranchSign(geometry->bcd_branch_sign) || !IsValidBranchSign(geometry->second_loop_branch_sign) ||
        !IsValidBranchSign(geometry->efp_branch_sign))
    {
        return 0;
    }

    return isfinite(geometry->oa_length) && isfinite(geometry->oc_length) && isfinite(geometry->ab_length) &&
           isfinite(geometry->bc_length) && isfinite(geometry->bd_length) && isfinite(geometry->cd_length) &&
           isfinite(geometry->of_length) && isfinite(geometry->cf_length) && isfinite(geometry->de_length) &&
           isfinite(geometry->ef_length) && isfinite(geometry->ep_length) && isfinite(geometry->fp_length) &&
           isfinite(geometry->geometry_consistency_epsilon) && isfinite(geometry->singular_epsilon) &&
           fabsf(geometry->of_length - geometry->oc_length - geometry->cf_length) <=
               geometry->geometry_consistency_epsilon;
}

static float WrapAngle(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static float UnwrapAngleNear(float angle, float reference)
{
    return reference + WrapAngle(angle - reference);
}

static int CrossSign(DoubleClosedLoopLegVec2_t lhs, DoubleClosedLoopLegVec2_t rhs, float epsilon)
{
    const float cross = Vec2Cross(lhs, rhs);
    if (cross > epsilon)
    {
        return 1;
    }
    if (cross < -epsilon)
    {
        return -1;
    }
    return 0;
}

static DoubleClosedLoopLegStatus_e RotationDistanceSolutions(DoubleClosedLoopLegVec2_t target,
                                                             DoubleClosedLoopLegVec2_t local_vector, float distance,
                                                             float epsilon, float solutions[2])
{
    const float target_length = Vec2Norm(target);
    const float local_length = Vec2Norm(local_vector);
    if (!IsFiniteVec2(target) || !IsFiniteVec2(local_vector) || !isfinite(distance) || distance <= 0.0f ||
        target_length <= epsilon || local_length <= epsilon)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVERSE_SINGULAR;
    }

    float cosine = (target_length * target_length + local_length * local_length - distance * distance) /
                   (2.0f * target_length * local_length);
    if (cosine > 1.0f + epsilon || cosine < -1.0f - epsilon)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVERSE_UNREACHABLE;
    }
    cosine = fminf(1.0f, fmaxf(-1.0f, cosine));
    const float delta = acosf(cosine);
    if (delta <= epsilon || fabsf(delta - (float)M_PI) <= epsilon)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVERSE_SINGULAR;
    }

    const float base_angle = atan2f(target.y, target.x) - atan2f(local_vector.y, local_vector.x);
    solutions[0] = base_angle + delta;
    solutions[1] = base_angle - delta;
    return DOUBLE_CLOSED_LOOP_LEG_OK;
}

static int IsExpectedInverseBranch(const DoubleClosedLoopLegGeometry_t *geometry,
                                   const DoubleClosedLoopLegState_t *state)
{
    return CrossSign(Vec2Subtract(state->a, state->c), Vec2Subtract(state->b, state->c),
                     geometry->singular_epsilon) == geometry->first_loop_branch_sign &&
           CrossSign(Vec2Subtract(state->b, state->c), Vec2Subtract(state->d, state->c),
                     geometry->singular_epsilon) == geometry->bcd_branch_sign &&
           CrossSign(Vec2Subtract(state->d, state->f), Vec2Subtract(state->e, state->f),
                     geometry->singular_epsilon) == geometry->second_loop_branch_sign &&
           CrossSign(Vec2Subtract(state->e, state->f), Vec2Subtract(state->p, state->f),
                     geometry->singular_epsilon) == geometry->efp_branch_sign;
}

static DoubleClosedLoopLegStatus_e CircleIntersection(const DoubleClosedLoopLegVec2_t center0, float radius0,
                                                      const DoubleClosedLoopLegVec2_t center1, float radius1,
                                                      float singular_epsilon, int8_t branch_sign,
                                                      DoubleClosedLoopLegVec2_t *intersection)
{
    const DoubleClosedLoopLegVec2_t delta = Vec2Subtract(center1, center0);
    const float distance = Vec2Norm(delta);
    const float tolerance = singular_epsilon;
    const float radius_sum = radius0 + radius1;
    const float radius_difference = fabsf(radius0 - radius1);

    if (!isfinite(distance) || distance > radius_sum + tolerance || distance < radius_difference - tolerance)
    {
        return DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_UNREACHABLE;
    }
    if (distance <= tolerance || fabsf(distance - radius_sum) <= tolerance ||
        fabsf(distance - radius_difference) <= tolerance)
    {
        return DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR;
    }

    const float projection = (radius0 * radius0 - radius1 * radius1 + distance * distance) / (2.0f * distance);
    const float height_squared = radius0 * radius0 - projection * projection;
    if (height_squared <= tolerance * tolerance)
    {
        return DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR;
    }
    if (height_squared < 0.0f)
    {
        return DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_UNREACHABLE;
    }

    const DoubleClosedLoopLegVec2_t base = {
        .x = center0.x + projection * delta.x / distance,
        .y = center0.y + projection * delta.y / distance,
    };
    const float height = sqrtf(height_squared);
    const DoubleClosedLoopLegVec2_t perpendicular = {
        .x = -delta.y / distance,
        .y = delta.x / distance,
    };
    const DoubleClosedLoopLegVec2_t candidate_plus = {
        .x = base.x + height * perpendicular.x,
        .y = base.y + height * perpendicular.y,
    };
    const DoubleClosedLoopLegVec2_t candidate_minus = {
        .x = base.x - height * perpendicular.x,
        .y = base.y - height * perpendicular.y,
    };

    const DoubleClosedLoopLegVec2_t from_center0_to_center1 = Vec2Subtract(center1, center0);
    const float plus_cross = Vec2Cross(from_center0_to_center1, Vec2Subtract(candidate_plus, center0));
    const float minus_cross = Vec2Cross(from_center0_to_center1, Vec2Subtract(candidate_minus, center0));

    if ((branch_sign > 0 && plus_cross > 0.0f) || (branch_sign < 0 && plus_cross < 0.0f))
    {
        *intersection = candidate_plus;
    }
    else if ((branch_sign > 0 && minus_cross > 0.0f) || (branch_sign < 0 && minus_cross < 0.0f))
    {
        *intersection = candidate_minus;
    }
    else
    {
        return DOUBLE_CLOSED_LOOP_LEG_BRANCH_MISMATCH;
    }
    return DOUBLE_CLOSED_LOOP_LEG_OK;
}

DoubleClosedLoopLegStatus_e DoubleClosedLoopLegForwardKinematics(const DoubleClosedLoopLegGeometry_t *geometry,
                                                                 const DoubleClosedLoopLegInput_t *input,
                                                                 DoubleClosedLoopLegState_t *state)
{
    if (geometry == NULL || input == NULL || state == NULL)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
    }
    memset(state, 0, sizeof(*state));
    if (!geometry->configured)
    {
        return DOUBLE_CLOSED_LOOP_LEG_NOT_CONFIGURED;
    }
    if (!IsValidGeometry(geometry) || !isfinite(input->phi1) || !isfinite(input->phi2))
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVALID_GEOMETRY;
    }

    state->a.x = geometry->oa_length * cosf(input->phi1);
    state->a.y = geometry->oa_length * sinf(input->phi1);
    state->c.x = geometry->oc_length * cosf(input->phi2);
    state->c.y = geometry->oc_length * sinf(input->phi2);
    state->f.x = geometry->of_length * cosf(input->phi2);
    state->f.y = geometry->of_length * sinf(input->phi2);
    DoubleClosedLoopLegStatus_e status =
        CircleIntersection(state->c, geometry->bc_length, state->a, geometry->ab_length,
                           geometry->singular_epsilon, geometry->first_loop_branch_sign, &state->b);
    if (status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return status;
    }

    status = CircleIntersection(state->c, geometry->cd_length, state->b, geometry->bd_length,
                                geometry->singular_epsilon, geometry->bcd_branch_sign, &state->d);
    if (status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return status;
    }
    const DoubleClosedLoopLegVec2_t c_to_d = Vec2Subtract(state->d, state->c);
    state->beta = atan2f(c_to_d.y, c_to_d.x);

    status = CircleIntersection(state->f, geometry->ef_length, state->d, geometry->de_length,
                                geometry->singular_epsilon, geometry->second_loop_branch_sign, &state->e);
    if (status == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_UNREACHABLE)
    {
        return DOUBLE_CLOSED_LOOP_LEG_SECOND_LOOP_UNREACHABLE;
    }
    if (status == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR)
    {
        return DOUBLE_CLOSED_LOOP_LEG_SECOND_LOOP_SINGULAR;
    }
    if (status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return status;
    }

    status = CircleIntersection(state->f, geometry->fp_length, state->e, geometry->ep_length,
                                geometry->singular_epsilon, geometry->efp_branch_sign, &state->p);
    if (status != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return status;
    }
    const DoubleClosedLoopLegVec2_t f_to_p = Vec2Subtract(state->p, state->f);
    state->epsilon = atan2f(f_to_p.y, f_to_p.x);

    state->length = Vec2Norm(state->p);
    state->theta = atan2f(state->p.x, state->p.y);
    state->first_loop_residual = fabsf(Vec2Norm(Vec2Subtract(state->b, state->a)) - geometry->ab_length);
    state->second_loop_residual = fabsf(Vec2Norm(Vec2Subtract(state->e, state->d)) - geometry->de_length);

    return IsFiniteVec2(state->p) && isfinite(state->beta) && isfinite(state->epsilon) && isfinite(state->length) &&
                   isfinite(state->theta)
               ? DOUBLE_CLOSED_LOOP_LEG_OK
               : DOUBLE_CLOSED_LOOP_LEG_NUMERIC_ERROR;
}

/**
 * @brief 按实物装配支路解析反算双闭环腿的两个主动轴角。
 *
 * @param geometry 已标定的机构几何及装配支路。
 * @param target_p 足端 P 的目标坐标，单位 m。
 * @param previous_input 当前或上一次的主动轴角，用来选择连续解。
 * @param solution 成功时输出 phi1、phi2，单位 rad。
 * @return 逆运动学状态码。
 */
DoubleClosedLoopLegStatus_e DoubleClosedLoopLegInverseKinematics(const DoubleClosedLoopLegGeometry_t *geometry,
                                                                 const DoubleClosedLoopLegVec2_t *target_p,
                                                                 const DoubleClosedLoopLegInput_t *previous_input,
                                                                 DoubleClosedLoopLegInput_t *solution)
{
    DoubleClosedLoopLegInput_t best_solution = {0};
    float best_cost = INFINITY;
    int found = 0;
    const DoubleClosedLoopLegVec2_t origin = {0};
    DoubleClosedLoopLegVec2_t f_candidates[2];
    const DoubleClosedLoopLegVec2_t b_local = {.x = geometry != NULL ? geometry->bc_length : 0.0f, .y = 0.0f};

    if (target_p == NULL || previous_input == NULL || solution == NULL)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
    }
    if (!IsValidGeometry(geometry))
    {
        return geometry != NULL && !geometry->configured ? DOUBLE_CLOSED_LOOP_LEG_NOT_CONFIGURED
                                                          : DOUBLE_CLOSED_LOOP_LEG_INVALID_GEOMETRY;
    }
    if (!IsFiniteVec2(*target_p) || !isfinite(previous_input->phi1) || !isfinite(previous_input->phi2))
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
    }

    const DoubleClosedLoopLegStatus_e f_status_positive =
        CircleIntersection(origin, geometry->of_length, *target_p, geometry->fp_length, geometry->singular_epsilon,
                           1, &f_candidates[0]);
    const DoubleClosedLoopLegStatus_e f_status_negative =
        CircleIntersection(origin, geometry->of_length, *target_p, geometry->fp_length, geometry->singular_epsilon,
                           -1, &f_candidates[1]);
    if (f_status_positive != DOUBLE_CLOSED_LOOP_LEG_OK && f_status_negative != DOUBLE_CLOSED_LOOP_LEG_OK)
    {
        return f_status_positive == DOUBLE_CLOSED_LOOP_LEG_FIRST_LOOP_SINGULAR
                   ? DOUBLE_CLOSED_LOOP_LEG_INVERSE_SINGULAR
                   : DOUBLE_CLOSED_LOOP_LEG_INVERSE_UNREACHABLE;
    }

    const float d_projection = (geometry->bc_length * geometry->bc_length + geometry->cd_length * geometry->cd_length -
                                geometry->bd_length * geometry->bd_length) /
                               (2.0f * geometry->bc_length);
    const float d_height_squared = geometry->cd_length * geometry->cd_length - d_projection * d_projection;
    const float efp_cosine = (geometry->ef_length * geometry->ef_length + geometry->fp_length * geometry->fp_length -
                              geometry->ep_length * geometry->ep_length) /
                             (2.0f * geometry->ef_length * geometry->fp_length);
    if (d_height_squared <= geometry->singular_epsilon * geometry->singular_epsilon || efp_cosine < -1.0f ||
        efp_cosine > 1.0f)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVERSE_SINGULAR;
    }
    const DoubleClosedLoopLegVec2_t d_local = {
        .x = d_projection,
        .y = (float)geometry->bcd_branch_sign * sqrtf(d_height_squared),
    };
    const float efp_delta = (float)geometry->efp_branch_sign * acosf(efp_cosine);

    for (uint8_t f_index = 0u; f_index < 2u; f_index++)
    {
        if ((f_index == 0u && f_status_positive != DOUBLE_CLOSED_LOOP_LEG_OK) ||
            (f_index == 1u && f_status_negative != DOUBLE_CLOSED_LOOP_LEG_OK))
        {
            continue;
        }
        const DoubleClosedLoopLegVec2_t f = f_candidates[f_index];
        const float phi2 = atan2f(f.y, f.x);
        const DoubleClosedLoopLegVec2_t c = {
            .x = geometry->oc_length * cosf(phi2),
            .y = geometry->oc_length * sinf(phi2),
        };
        const DoubleClosedLoopLegVec2_t f_to_p = Vec2Subtract(*target_p, f);
        const DoubleClosedLoopLegVec2_t e = Vec2Rotate(
            (DoubleClosedLoopLegVec2_t){.x = geometry->ef_length * f_to_p.x / geometry->fp_length,
                                        .y = geometry->ef_length * f_to_p.y / geometry->fp_length},
            -efp_delta);
        const DoubleClosedLoopLegVec2_t e_absolute = {.x = f.x + e.x, .y = f.y + e.y};
        float beta_candidates[2];
        if (RotationDistanceSolutions(Vec2Subtract(e_absolute, c), d_local, geometry->de_length,
                                      geometry->singular_epsilon, beta_candidates) != DOUBLE_CLOSED_LOOP_LEG_OK)
        {
            continue;
        }
        for (uint8_t beta_index = 0u; beta_index < 2u; beta_index++)
        {
            const DoubleClosedLoopLegVec2_t b_relative = Vec2Rotate(b_local, beta_candidates[beta_index]);
            const DoubleClosedLoopLegVec2_t b = {.x = c.x + b_relative.x, .y = c.y + b_relative.y};
            float phi1_candidates[2];
            if (RotationDistanceSolutions(b, (DoubleClosedLoopLegVec2_t){.x = geometry->oa_length, .y = 0.0f},
                                          geometry->ab_length, geometry->singular_epsilon, phi1_candidates) !=
                DOUBLE_CLOSED_LOOP_LEG_OK)
            {
                continue;
            }
            for (uint8_t phi1_index = 0u; phi1_index < 2u; phi1_index++)
            {
                const DoubleClosedLoopLegInput_t candidate = {
                    .phi1 = UnwrapAngleNear(phi1_candidates[phi1_index], previous_input->phi1),
                    .phi2 = UnwrapAngleNear(phi2, previous_input->phi2),
                };
                DoubleClosedLoopLegState_t state = {0};
                if (DoubleClosedLoopLegForwardKinematics(geometry, &candidate, &state) != DOUBLE_CLOSED_LOOP_LEG_OK ||
                    !IsExpectedInverseBranch(geometry, &state))
                {
                    continue;
                }
                const float target_error = Vec2Norm(Vec2Subtract(state.p, *target_p));
                const float phi1_error = WrapAngle(candidate.phi1 - previous_input->phi1);
                const float phi2_error = WrapAngle(candidate.phi2 - previous_input->phi2);
                const float cost = target_error * target_error + phi1_error * phi1_error + phi2_error * phi2_error;
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_solution = candidate;
                    found = 1;
                }
            }
        }
    }

    if (!found)
    {
        return DOUBLE_CLOSED_LOOP_LEG_INVERSE_UNREACHABLE;
    }
    *solution = best_solution;
    return DOUBLE_CLOSED_LOOP_LEG_OK;
}

void DoubleClosedLoopLegInit(DoubleClosedLoopLegInstance_t *instance, const DoubleClosedLoopLegConfig_t *config)
{
    if (instance == NULL)
    {
        return;
    }

    memset(instance, 0, sizeof(*instance));
    instance->config = config;
    instance->transmission_status[0] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
    instance->transmission_status[1] = JOINT_TRANSMISSION_INVALID_ARGUMENT;
    instance->forward_kinematics_status = DOUBLE_CLOSED_LOOP_LEG_INVALID_ARGUMENT;
}

void DoubleClosedLoopLegUpdate(DoubleClosedLoopLegInstance_t *instance, float actuator_angle_0, float actuator_angle_1)
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
        instance->forward_kinematics_status = DOUBLE_CLOSED_LOOP_LEG_NOT_CONFIGURED;
        return;
    }

    const DoubleClosedLoopLegInput_t input = {.phi1 = instance->phi[0], .phi2 = instance->phi[1]};
    instance->forward_kinematics_status =
        DoubleClosedLoopLegForwardKinematics(&instance->config->geometry, &input, &instance->state);
}
