#include "sysid_hip.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && sizeof(float) == sizeof(uint32_t),
               "SysId hip requires binary32 float");

static bool FloatIsFinite(float value)
{
    // 不让固件 -Ofast 的有限数假设消除 NaN/Inf 检查。
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static bool InRange(float value, float min, float max)
{
    return FloatIsFinite(value) && value >= min && value <= max;
}

static bool WithinMagnitude(float value, float limit)
{
    return InRange(value, -limit, limit);
}

static bool Positive(float value)
{
    return FloatIsFinite(value) && value > 0.0f;
}

static bool NonNegative(float value)
{
    return FloatIsFinite(value) && value >= 0.0f;
}

static bool JointConfigValid(const SysIdHipJointConfig *joint)
{
    if (!FloatIsFinite(joint->q_min_rad) || !FloatIsFinite(joint->q_max_rad) || !FloatIsFinite(joint->center_rad) ||
        !(joint->q_min_rad < joint->center_rad && joint->center_rad < joint->q_max_rad) ||
        !Positive(joint->kp_nm_per_rad) || !NonNegative(joint->kd_nm_per_rad_s) ||
        !Positive(joint->max_abs_speed_rad_s) || !Positive(joint->max_abs_feedback_torque_nm) ||
        !Positive(joint->max_abs_command_torque_nm) || !FloatIsFinite(joint->max_temperature_c) ||
        !NonNegative(joint->start_max_error_rad) || !NonNegative(joint->start_max_abs_speed_rad_s) ||
        joint->start_max_abs_speed_rad_s > joint->max_abs_speed_rad_s ||
        !NonNegative(joint->start_max_abs_torque_nm) ||
        joint->start_max_abs_torque_nm > joint->max_abs_command_torque_nm)
        return false;

    return InRange(joint->center_rad - joint->start_max_error_rad, joint->q_min_rad, joint->q_max_rad) &&
           InRange(joint->center_rad + joint->start_max_error_rad, joint->q_min_rad, joint->q_max_rad);
}

bool SysIdHipConfigValid(const SysIdHip_Init_Config_s *config)
{
    if (config == NULL)
        return false;
    const unsigned mask = config->measured_mask;
    if (mask != SYSID_HIP_MASK_LEFT && mask != SYSID_HIP_MASK_RIGHT && mask != SYSID_HIP_MASK_ALL)
        return false;
    for (size_t i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if ((mask & (1u << i)) != 0 && !JointConfigValid(&config->joint[i]))
            return false;
    }
    return true;
}

bool SysIdHipInit(SysIdHipInstance *hip, const SysIdHip_Init_Config_s *config)
{
    if (hip == NULL)
        return false;
    *hip = (SysIdHipInstance){.status = SYSID_HIP_FAULT};
    if (!SysIdHipConfigValid(config))
        return false;
    *hip = (SysIdHipInstance){.config = config};
    return true;
}

static SysIdHipOutput Quiet(SysIdHipInstance *hip, SysIdHipStatus status)
{
    if (hip != NULL)
    {
        hip->status = status;
        if (status != SYSID_HIP_ARMED)
            hip->side = SYSID_HIP_SIDE_NONE;
    }
    return (SysIdHipOutput){.status = status};
}

static bool JointFeedbackInRange(const SysIdHipFeedback *fb, const SysIdHipJointConfig *joint)
{
    return fb->valid && InRange(fb->q_rad, joint->q_min_rad, joint->q_max_rad) &&
           WithinMagnitude(fb->qd_rad_s, joint->max_abs_speed_rad_s) &&
           WithinMagnitude(fb->tau_nm, joint->max_abs_feedback_torque_nm) && FloatIsFinite(fb->temperature_c) &&
           fb->temperature_c <= joint->max_temperature_c;
}

static bool FeedbackInRange(const SysIdHip_Init_Config_s *config, const SysIdHipFeedback feedback[SYSID_HIP_COUNT])
{
    if (feedback == NULL)
        return false;
    for (size_t i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if ((config->measured_mask & (1u << i)) != 0 && !JointFeedbackInRange(&feedback[i], &config->joint[i]))
            return false;
    }
    return true;
}

bool SysIdHipFeedbackValid(const SysIdHip_Init_Config_s *config, const SysIdHipFeedback feedback[SYSID_HIP_COUNT])
{
    return SysIdHipConfigValid(config) && FeedbackInRange(config, feedback);
}

static bool NearCenter(const SysIdHipFeedback *fb, float q_ref, const SysIdHipJointConfig *joint)
{
    return WithinMagnitude(fb->q_rad - joint->center_rad, joint->start_max_error_rad) &&
           WithinMagnitude(q_ref - joint->center_rad, joint->start_max_error_rad) &&
           WithinMagnitude(fb->qd_rad_s, joint->start_max_abs_speed_rad_s);
}

static float ClampTorque(float torque, float limit)
{
    if (torque > limit)
        return limit;
    if (torque < -limit)
        return -limit;
    return torque;
}

SysIdHipOutput SysIdHipStep(SysIdHipInstance *hip, const SysIdHipInput *input)
{
    if (hip == NULL || input == NULL || !SysIdHipConfigValid(hip->config))
        return Quiet(hip, SYSID_HIP_FAULT);

    if (input->state == SYSID_STATE_ESTOP)
        return Quiet(hip, SYSID_HIP_DISABLED);
    if (hip->status == SYSID_HIP_FAULT)
        return Quiet(hip, SYSID_HIP_FAULT);

    const bool starting = input->state == SYSID_STATE_STARTING;
    switch (input->state)
    {
    case SYSID_STATE_NEED_ESTOP:
    case SYSID_STATE_READY:
    case SYSID_STATE_FINISHED:
    case SYSID_STATE_DRY_RUNNING:
        return Quiet(hip, SYSID_HIP_DISABLED);
    case SYSID_STATE_STARTING:
    case SYSID_STATE_RUNNING:
        break;
    default:
        return Quiet(hip, SYSID_HIP_FAULT);
    }

    if ((input->side != SYSID_HIP_SIDE_LEFT && input->side != SYSID_HIP_SIDE_RIGHT) ||
        (hip->status != SYSID_HIP_DISABLED && hip->side != input->side) ||
        (starting && hip->status == SYSID_HIP_ACTIVE) || (!starting && hip->status == SYSID_HIP_DISABLED))
        return Quiet(hip, SYSID_HIP_FAULT);

    const bool left = input->side == SYSID_HIP_SIDE_LEFT;
    const unsigned side_mask = left ? SYSID_HIP_MASK_LEFT : SYSID_HIP_MASK_RIGHT;
    if ((hip->config->measured_mask & side_mask) != side_mask || !FeedbackInRange(hip->config, input->feedback))
        return Quiet(hip, SYSID_HIP_FAULT);

    const bool first_run = !starting && hip->status == SYSID_HIP_ARMED;
    const size_t first = left ? SYSID_HIP_LEFT_FRONT : SYSID_HIP_RIGHT_FRONT;
    SysIdHipOutput out = {.status = starting ? SYSID_HIP_ARMED : SYSID_HIP_ACTIVE};
    for (size_t i = 0; i < SYSID_LEG_COUNT; ++i)
    {
        const size_t idx = first + i;
        const SysIdHipJointConfig *joint = &hip->config->joint[idx];
        const SysIdHipFeedback *fb = &input->feedback[idx];
        const float q_ref = input->q_ref_rad[i];
        if (!InRange(q_ref, joint->q_min_rad, joint->q_max_rad) ||
            ((starting || first_run) && !NearCenter(fb, q_ref, joint)))
            return Quiet(hip, SYSID_HIP_FAULT);
        if (starting)
            continue;

        const float torque = joint->kp_nm_per_rad * (q_ref - fb->q_rad) - joint->kd_nm_per_rad_s * fb->qd_rad_s;
        if (!FloatIsFinite(torque) || (first_run && !WithinMagnitude(torque, joint->start_max_abs_torque_nm)))
            return Quiet(hip, SYSID_HIP_FAULT);
        out.torque_nm[idx] = ClampTorque(torque, joint->max_abs_command_torque_nm);
        out.clipped |= (out.torque_nm[idx] != torque);
    }

    hip->side = input->side;
    hip->status = out.status;
    return out;
}
