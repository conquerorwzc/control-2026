#include "sysid_runner.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && sizeof(float) == sizeof(uint32_t),
               "SysId runner requires binary32 float");

static bool FloatIsFinite(float value)
{
    // 直接检查指数位，避免固件 -Ofast 将浮点有限性检查优化掉。
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static SysIdMode SelectMode(SysIdSwitch right)
{
    switch (right)
    {
    case SYSID_SWITCH_UP:
        return SYSID_MODE_SLOW;
    case SYSID_SWITCH_MIDDLE:
        return SYSID_MODE_CHIRP;
    case SYSID_SWITCH_DOWN:
        return SYSID_MODE_HOLD;
    default:
        return SYSID_MODE_NONE;
    }
}

static SysIdSignalConfig MakeSignal(const SysIdRunner_Init_Config_s *config, SysIdMode mode)
{
    const SysIdTrialConfig *trial = &config->hold;
    SysIdSignalType type = SYSID_SIGNAL_HOLD;
    if (mode == SYSID_MODE_SLOW)
    {
        trial = &config->slow;
        type = SYSID_SIGNAL_SINE;
    }
    else if (mode == SYSID_MODE_CHIRP)
    {
        trial = &config->chirp;
        type = SYSID_SIGNAL_CHIRP;
    }

    return (SysIdSignalConfig){
        .type = type,
        .offset = 0.0f,
        .amplitude = trial->amplitude_rad,
        .frequency_start_hz = mode == SYSID_MODE_HOLD ? 0.0f : trial->frequency_start_hz,
        .frequency_end_hz = mode == SYSID_MODE_CHIRP ? trial->frequency_end_hz :
                            mode == SYSID_MODE_SLOW  ? trial->frequency_start_hz : 0.0f,
        .duration_s = trial->duration_s,
        .ramp_s = trial->ramp_s,
        .sample_period_s = SYSID_RUNNER_PERIOD_US * 1.0e-6f,
    };
}

bool SysIdRunnerInit(SysIdRunnerInstance *runner, const SysIdRunner_Init_Config_s *config)
{
    *runner = (SysIdRunnerInstance){.state = SYSID_STATE_FAULT};
    if (config == NULL)
        return false;

    for (size_t leg = 0; leg < SYSID_LEG_COUNT; ++leg)
    {
        if (!FloatIsFinite(config->min_angle_rad[leg]) || !FloatIsFinite(config->max_angle_rad[leg]) ||
            config->min_angle_rad[leg] >= config->max_angle_rad[leg])
            return false;
    }

    for (SysIdMode mode = SYSID_MODE_SLOW; mode <= SYSID_MODE_HOLD; ++mode)
    {
        const SysIdSignalConfig signal = MakeSignal(config, mode);
        if (!SysIdSignalValid(&signal) || signal.amplitude <= 0.0f ||
            (mode == SYSID_MODE_CHIRP && signal.frequency_start_hz == signal.frequency_end_hz) ||
            (mode == SYSID_MODE_HOLD && 2.0f * signal.ramp_s >= signal.duration_s))
            return false;
    }

    runner->config = config;
    runner->state = SYSID_STATE_NEED_ESTOP;
    return true;
}

static SysIdRunnerOutput SetOutput(SysIdRunnerInstance *runner, SysIdState state, SysIdMode mode, float excitation)
{
    runner->state = state;
    runner->mode = mode;
    return (SysIdRunnerOutput){.mode = mode, .state = state, .excitation_rad = excitation};
}

static bool CaptureTrial(SysIdRunnerInstance *runner, const SysIdRunnerInput *input)
{
    const SysIdSignalConfig signal = MakeSignal(runner->config, runner->mode);
    for (size_t leg = 0; leg < SYSID_LEG_COUNT; ++leg)
    {
        const float center = input->center_rad[leg];
        const int8_t direction = input->direction[leg];
        if (!FloatIsFinite(center) || (direction != 1 && direction != -1))
            return false;

        // 往返/扫频是双向的；平台仅从中心沿 direction 偏移。
        const float offset = direction * signal.amplitude;
        const float first = runner->mode == SYSID_MODE_HOLD ? center : center - offset;
        const float last = center + offset;
        if (!FloatIsFinite(first) || !FloatIsFinite(last) || first < runner->config->min_angle_rad[leg] ||
            first > runner->config->max_angle_rad[leg] || last < runner->config->min_angle_rad[leg] ||
            last > runner->config->max_angle_rad[leg])
            return false;
    }
    memcpy(runner->center_rad, input->center_rad, sizeof(runner->center_rad));
    memcpy(runner->direction, input->direction, sizeof(runner->direction));
    runner->active_signal = signal;
    runner->start_us = input->now_us;
    return true;
}

static bool ReferencesUnchanged(const SysIdRunnerInstance *runner, const SysIdRunnerInput *input)
{
    for (size_t leg = 0; leg < SYSID_LEG_COUNT; ++leg)
    {
        if (!FloatIsFinite(input->center_rad[leg]) || input->center_rad[leg] != runner->center_rad[leg] ||
            input->direction[leg] != runner->direction[leg])
            return false;
    }
    return true;
}

SysIdRunnerOutput SysIdRunnerStep(SysIdRunnerInstance *runner, const SysIdRunnerInput *input)
{
    const SysIdMode selected = SelectMode(input->right);
    const bool time_valid = !runner->have_previous_time || input->now_us > runner->previous_us;
    runner->have_previous_time = true;
    runner->previous_us = input->now_us;

    if (runner->config == NULL || !time_valid || !input->remote_online || selected == SYSID_MODE_NONE ||
        (input->left != SYSID_SWITCH_DOWN && input->left != SYSID_SWITCH_MIDDLE && input->left != SYSID_SWITCH_UP) ||
        !(input->feedback_valid || (runner->config->dry_run && input->synthetic_input_valid)))
        return SetOutput(runner, SYSID_STATE_FAULT, SYSID_MODE_NONE, 0.0f);

    // 故障当拍的左下不算复位，只有此处实际采到的健康左下才算。
    if (input->left == SYSID_SWITCH_DOWN)
        return SetOutput(runner, SYSID_STATE_ESTOP, SYSID_MODE_NONE, 0.0f);

    switch (runner->state)
    {
    case SYSID_STATE_ESTOP:
        if (input->left == SYSID_SWITCH_MIDDLE)
            return SetOutput(runner, SYSID_STATE_READY, selected, 0.0f);
        break;
    case SYSID_STATE_READY:
        if (input->left == SYSID_SWITCH_MIDDLE)
            return SetOutput(runner, SYSID_STATE_READY, selected, 0.0f);
        if (selected != runner->mode)
            break;
        if (!CaptureTrial(runner, input))
            return SetOutput(runner, SYSID_STATE_FAULT, SYSID_MODE_NONE, 0.0f);
        return SetOutput(runner, SYSID_STATE_STARTING, selected, 0.0f);
    case SYSID_STATE_STARTING:
    case SYSID_STATE_RUNNING:
    case SYSID_STATE_DRY_RUNNING: {
        if (input->left != SYSID_SWITCH_UP || selected != runner->mode)
            break;
        if (!ReferencesUnchanged(runner, input))
            return SetOutput(runner, SYSID_STATE_FAULT, SYSID_MODE_NONE, 0.0f);

        const float elapsed_s = (float)(input->now_us - runner->start_us) * 1.0e-6f;
        if (elapsed_s >= runner->active_signal.duration_s)
            return SetOutput(runner, SYSID_STATE_FINISHED, selected, 0.0f);

        const float excitation = SysIdSignalValue(&runner->active_signal, elapsed_s);
        if (!FloatIsFinite(excitation))
            return SetOutput(runner, SYSID_STATE_FAULT, SYSID_MODE_NONE, 0.0f);
        const SysIdState state = runner->config->dry_run ? SYSID_STATE_DRY_RUNNING : SYSID_STATE_RUNNING;
        return SetOutput(runner, state, selected, excitation);
    }
    case SYSID_STATE_FINISHED:
    case SYSID_STATE_FAULT:
    case SYSID_STATE_NEED_ESTOP:
        return SetOutput(runner, runner->state, runner->mode, 0.0f);
    }

    // 跳过准备、边启动边换档、运行中改档或退出左上，均需重新拨过左下。
    return SetOutput(runner, SYSID_STATE_NEED_ESTOP, SYSID_MODE_NONE, 0.0f);
}
