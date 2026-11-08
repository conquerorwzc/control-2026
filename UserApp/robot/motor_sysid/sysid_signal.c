#include "sysid_signal.h"
#include <math.h>

static const float kPi = 3.14159265358979323846f;

bool SysIdSignalValid(const SysIdSignalConfig *config)
{
    if (!isfinite(config->offset) || !isfinite(config->amplitude) || !isfinite(config->frequency_start_hz) ||
        !isfinite(config->frequency_end_hz) || !isfinite(config->duration_s) || !isfinite(config->ramp_s) ||
        !isfinite(config->sample_period_s))
        return false;
    if (config->duration_s <= 0.0f || config->sample_period_s <= 0.0f || config->amplitude < 0.0f ||
        config->ramp_s <= 0.0f || 2.0f * config->ramp_s > config->duration_s)
        return false;
    if (config->type < SYSID_SIGNAL_HOLD || config->type > SYSID_SIGNAL_CHIRP)
        return false;
    if (config->type == SYSID_SIGNAL_HOLD)
        return true;
    const float nyquist_hz = 0.5f / config->sample_period_s;
    if (config->frequency_start_hz <= 0.0f || config->frequency_start_hz >= nyquist_hz ||
        config->frequency_end_hz <= 0.0f || config->frequency_end_hz >= nyquist_hz)
        return false;
    return config->type != SYSID_SIGNAL_SINE || config->frequency_start_hz == config->frequency_end_hz;
}

float SysIdSignalValue(const SysIdSignalConfig *config, float elapsed_s)
{
    if (elapsed_s <= 0.0f || elapsed_s >= config->duration_s)
        return config->offset;
    float envelope = 1.0f;
    if (elapsed_s < config->ramp_s)
        envelope = 0.5f * (1.0f - cosf(kPi * elapsed_s / config->ramp_s));
    else if (elapsed_s > config->duration_s - config->ramp_s)
        envelope = 0.5f * (1.0f - cosf(kPi * (config->duration_s - elapsed_s) / config->ramp_s));
    float carrier = 1.0f;
    if (config->type != SYSID_SIGNAL_HOLD)
    {
        // 线性扫频相位是频率积分，不能直接用 sin(2*pi*f(t)*t)。
        const float slope = (config->frequency_end_hz - config->frequency_start_hz) / config->duration_s;
        const float phase =
            2.0f * kPi * (config->frequency_start_hz * elapsed_s + 0.5f * slope * elapsed_s * elapsed_s);
        carrier = sinf(phase);
    }
    return config->offset + config->amplitude * envelope * carrier;
}
