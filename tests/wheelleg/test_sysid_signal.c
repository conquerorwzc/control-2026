#include "sysid_signal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    // 合成测试值，不是实车激励或安全限值。
    SysIdSignalConfig config = {SYSID_SIGNAL_SINE, 2.0f, 0.25f, 1.0f, 1.0f, 2.0f, 0.1f, 0.002f};
    assert(SysIdSignalValid(&config));
    assert(SysIdSignalValue(&config, -1.0f) == 2.0f);
    assert(SysIdSignalValue(&config, 0.0f) == 2.0f);
    assert(fabsf(SysIdSignalValue(&config, 0.25f) - 2.25f) < 1e-6f);
    assert(fabsf(SysIdSignalValue(&config, 0.75f) - 1.75f) < 1e-6f);
    assert(SysIdSignalValue(&config, 2.0f) == 2.0f);
    config.type = SYSID_SIGNAL_CHIRP;
    config.frequency_end_hz = 3.0f;
    assert(SysIdSignalValid(&config));
    const float expected = 2.0f + 0.25f * sinf(2.0f * 3.14159265358979323846f * 0.625f);
    assert(fabsf(SysIdSignalValue(&config, 0.5f) - expected) < 1e-6f);
    for (int i = 0; i <= 1000; ++i)
        assert(fabsf(SysIdSignalValue(&config, 0.002f * (float)i) - 2.0f) <= 0.25001f);
    config.type = SYSID_SIGNAL_HOLD;
    assert(fabsf(SysIdSignalValue(&config, 0.05f) - 2.125f) < 1e-6f);
    assert(fabsf(SysIdSignalValue(&config, 1.95f) - 2.125f) < 1e-6f);
    config.type = SYSID_SIGNAL_CHIRP;
    config.frequency_end_hz = 250.0f;
    assert(!SysIdSignalValid(&config));
    config.frequency_end_hz = 3.0f;
    config.amplitude = NAN;
    assert(!SysIdSignalValid(&config));
    config.amplitude = 0.25f;
    config.ramp_s = 0.0f;
    assert(!SysIdSignalValid(&config));
    puts("sysid signal: sine, chirp phase, bounds, envelope, NaN and Nyquist checks passed");
    return 0;
}
