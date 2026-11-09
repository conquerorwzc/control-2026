#include "length_control.h"
#include <assert.h>
#include <stdio.h>

// 仅替换硬件计时；PIDInit/PIDCalculate 链接队内真实 controller.c。
float DWT_GetDeltaT(uint32_t *count)
{
    *count += 2000u;
    return 0.002f;
}

int main(void)
{
    LegLengthConfig config = {
        .first_m = 0.105f,
        .second_m = 0.125f,
        .virtual_second_m = 0.0625f,
        .epsilon = 1e-5f,
        .minimum_m = 0.05619f,
        .maximum_m = 0.18192f,
        .damping = 5.0f,
        .torque_limit_nm = 2.0f,
        .pid = {.Kp = 800.0f, .MaxOut = 40.0f, .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral}};
    LegLengthState state = {0};
    const float q[2] = {0.841850453f, 2.299742201f};
    const float qd[2] = {0.0f, 0.0f};
    assert(LegLengthGeometry(&config, q, qd, &state));
    assert(fabsf(state.length_m - 0.18192f) < 2e-5f);
    for (unsigned j = 0; j < 2; ++j)
    {
        float plus[2] = {q[0], q[1]}, minus[2] = {q[0], q[1]};
        plus[j] += 0.001f;
        minus[j] -= 0.001f;
        LegLengthState a = {0}, b = {0};
        assert(LegLengthGeometry(&config, plus, qd, &a));
        assert(LegLengthGeometry(&config, minus, qd, &b));
        const float derivative = (a.length_m - b.length_m) / 0.002f;
        assert(fabsf(state.jacobian[j] - derivative) < 1e-5f);
    }
    PIDInit(&state.pid, &config.pid);
    state.length_m = 0.12f;
    state.speed_m_s = 0.1f;
    assert(LegLengthStep(&config, 0.13f, &state));
    assert(fabsf(state.force_n - 7.5f) < 1e-4f);
    assert(state.torque_nm[0] > 0.0f && state.torque_nm[1] < 0.0f);
    assert(fabsf(state.torque_nm[0] + state.torque_nm[1]) < 1e-6f);
    state.speed_m_s = 0.0f;
    assert(LegLengthStep(&config, config.maximum_m, &state));
    assert(state.force_n == 40.0f);
    assert(fabsf(state.torque_nm[0]) <= 2.0f && fabsf(state.torque_nm[1]) <= 2.0f);
    assert(!LegLengthStep(&config, 0.20f, &state));
    assert(!LegLengthStep(&config, NAN, &state));
    float singular[2] = {1.0f, 1.0f};
    assert(!LegLengthGeometry(&config, singular, qd, &state));
    singular[0] = NAN;
    assert(!LegLengthGeometry(&config, singular, qd, &state));
    assert(LegLengthTarget(-660.0f, 660.0f, &config) == config.minimum_m);
    assert(fabsf(LegLengthTarget(660.0f, 660.0f, &config) - config.maximum_m) < 1e-6f);
    assert(fabsf(LegLengthTarget(0.0f, 660.0f, &config) - 0.119055f) < 1e-6f);
    for (unsigned l = 0; l <= 4; ++l)
        for (unsigned r = 0; r <= 4; ++r)
        {
            assert(!LegLengthRequested(false, l, r));
            const bool expected = (l == 1 || l == 3) && (r == 1 || r == 3);
            assert(LegLengthRequested(true, l, r) == expected);
        }
    puts("PASS: SHU pose, finite-difference Jacobian, real PID + damping, torque limits, invalid inputs, all switch "
         "states");
    return 0;
}
