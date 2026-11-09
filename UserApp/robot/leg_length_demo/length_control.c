#include "length_control.h"
#include <math.h>

// 提取自 control-2026@170306b7 wheel_legged_SHU/chassis_control.c 的 LegCalculate。
// 仅支持源 demo 的同心主动轴 ACE 等效机构；不是新车未知尺寸的通用五连杆。
bool LegLengthGeometry(const LegLengthConfig *config, const float q[2], const float qd[2], LegLengthState *state)
{
    if (!isfinite(q[0]) || !isfinite(q[1]) || !isfinite(qd[0]) || !isfinite(qd[1]))
        return false;
    const float scale = config->virtual_second_m / config->second_m;
    const float active = scale * config->first_m;
    const float half_difference = 0.5f * (q[0] - q[1]);
    const float sine = sinf(half_difference), cosine = cosf(half_difference);
    const float discriminant = config->virtual_second_m * config->virtual_second_m - active * active * sine * sine;
    if (!(discriminant > config->epsilon * config->epsilon))
        return false;
    const float root = sqrtf(discriminant);
    float branch = 1.0f;
    float radial = active * cosine + root;
    if (radial <= config->epsilon)
    {
        branch = -1.0f;
        radial = active * cosine - root;
    }
    if (!(radial > config->epsilon))
        return false;
    state->length_m = radial / scale;
    state->jacobian[0] = 0.5f * (-active * sine - branch * active * active * sine * cosine / root) / scale;
    state->jacobian[1] = -state->jacobian[0];
    state->speed_m_s = state->jacobian[0] * qd[0] + state->jacobian[1] * qd[1];
    return isfinite(state->length_m) && isfinite(state->speed_m_s) && fabsf(state->jacobian[0]) > config->epsilon;
}

bool LegLengthStep(const LegLengthConfig *config, float target_m, LegLengthState *state)
{
    if (!isfinite(target_m) || target_m < config->minimum_m || target_m > config->maximum_m)
        return false;
    // 源 SHU 使用通用 PI + 雅可比腿速阻尼，避免对拨轮目标求微分。
    const float force = PIDCalculate(&state->pid, state->length_m, target_m) - config->damping * state->speed_m_s;
    if (!isfinite(force))
        return false;
    state->force_n = fminf(fmaxf(force, -state->pid.MaxOut), state->pid.MaxOut);
    for (unsigned i = 0; i < 2; ++i)
    {
        const float torque = state->jacobian[i] * state->force_n;
        if (!isfinite(torque))
            return false;
        state->torque_nm[i] = fminf(fmaxf(torque, -config->torque_limit_nm), config->torque_limit_nm);
    }
    return true;
}

float LegLengthTarget(float dial, float full_scale, const LegLengthConfig *config)
{
    const float ratio = 0.5f * (fminf(fmaxf(dial / full_scale, -1.0f), 1.0f) + 1.0f);
    return config->minimum_m + ratio * (config->maximum_m - config->minimum_m);
}

bool LegLengthRequested(bool online, unsigned left_switch, unsigned right_switch)
{
    // DT7: UP=1 DOWN=2 MID=3；任一下位或非法编码停止。原 UP 的平衡功能不移植。
    return online && (left_switch == 1u || left_switch == 3u) && (right_switch == 1u || right_switch == 3u);
}
