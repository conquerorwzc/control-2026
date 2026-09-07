/**
 * @file pid_tuning.c
 * @author Codex
 * @brief  PID 参数整定工具实现
 * @version V1.0.0
 * @date 2026-08-01
 */
#include "pid_tuning.h"
#include <stddef.h>

#define PIDT_PI 3.14159265358979f

static PID_TuningResult_s PIDT_Make(float Kp, float Ti, float Td)
{
    PID_TuningResult_s r;
    r.Kp = Kp;
    r.Ti = (Ti > 0.0f) ? Ti : 0.0f;
    r.Td = (Td > 0.0f) ? Td : 0.0f;
    r.Ki = (r.Ti > 0.0f) ? (Kp / r.Ti) : 0.0f;
    r.Kd = (r.Td > 0.0f) ? (Kp * r.Td) : 0.0f;
    return r;
}

/* Ziegler-Nichols 临界比例度法:由临界增益 Ku 与临界周期 Tu */
PID_TuningResult_s PIDTuneZN_Oscillation(float Ku, float Tu)
{
    /* 经典 Z-N PID 公式:Kp=0.6Ku, Ti=0.5Tu, Td=0.125Tu */
    return PIDT_Make(0.6f * Ku, 0.5f * Tu, 0.125f * Tu);
}

/* Ziegler-Nichols 响应曲线法:由一阶惯性+纯滞后模型 K/L/T */
PID_TuningResult_s PIDTuneZN_ReactionCurve(float K, float L, float T)
{
    /* Kp=1.2*T/(K*L), Ti=2L, Td=0.5L */
    if (K <= 0.0f || L <= 0.0f)
        return PIDT_Make(0.0f, 0.0f, 0.0f);
    return PIDT_Make(1.2f * T / (K * L), 2.0f * L, 0.5f * L);
}

/* Cohen-Coon 反应曲线法 */
PID_TuningResult_s PIDTuneCohenCoon(float K, float L, float T)
{
    if (K <= 0.0f || L <= 0.0f || T <= 0.0f)
        return PIDT_Make(0.0f, 0.0f, 0.0f);
    float x = L / T;
    float Kp = (1.0f / K) * (T / L) * (0.9f + x / 12.0f);
    float Ti = L * (30.0f + 3.0f * x) / (9.0f + 20.0f * x);
    float Td = L * (4.0f * x) / (11.0f + 2.0f * x);
    return PIDT_Make(Kp, Ti, Td);
}

/* Lambda / SIMC (Skogestad) */
PID_TuningResult_s PIDTuneSIMC(float K, float L, float T, float Tc)
{
    if (K <= 0.0f || L <= 0.0f || Tc <= 0.0f)
        return PIDT_Make(0.0f, 0.0f, 0.0f);
    float Kp = T / (K * (Tc + L));
    float Ti = (T < 4.0f * (Tc + L)) ? T : (4.0f * (Tc + L));
    return PIDT_Make(Kp, Ti, 0.0f); /* 一阶对象无需微分 */
}

/* 4:1 衰减曲线法 */
PID_TuningResult_s PIDTuneDecayCurve(float Kp_s, float Tu_s)
{
    /* 常用经验式:Kp=Kp_s, Ti=Tu_s/1.2, Td=Tu_s/8 */
    return PIDT_Make(Kp_s, Tu_s / 1.2f, Tu_s / 8.0f);
}

/* ---------------------- 继电反馈自整定 ---------------------- */

int PIDRelayAutoTuneInit(PID_RelayAutoTune_s *rt, float h, float mu, uint32_t cycles)
{
    if (rt == NULL || h <= 0.0f || mu < 0.0f)
        return -1;
    rt->h = h;
    rt->mu = mu;
    rt->cycles_needed = (cycles >= 2) ? cycles : 3;
    rt->setpoint = 0.0f;
    rt->out = 0.0f;
    rt->dt = 0.0f;
    rt->t = 0.0f;
    rt->peak_max = 0.0f;
    rt->peak_min = 0.0f;
    rt->a = 0.0f;
    rt->a_accum = 0.0f;
    rt->period_accum = 0.0f;
    rt->Tu = 0.0f;
    rt->Ku = 0.0f;
    rt->t_period_start = 0.0f;
    rt->half_cycles = 0;
    rt->full_cycles = 0;
    rt->initialized = 0;
    rt->done = 0;
    return 0;
}

float PIDRelayAutoTuneStep(PID_RelayAutoTune_s *rt, float measure, float setpoint, float dt)
{
    rt->setpoint = setpoint;
    rt->dt = dt;
    if (rt->done)
        return 0.0f;

    if (!rt->initialized)
    {
        rt->initialized = 1;
        rt->out = rt->h; /* 初始正向激励:对象向上穿越 +mu 后切入负反馈 bang-bang */
        rt->t = 0.0f;
        rt->peak_max = measure;
        rt->peak_min = measure;
        return rt->out;
    }

    rt->t += dt;
    if (measure > rt->peak_max)
        rt->peak_max = measure;
    if (measure < rt->peak_min)
        rt->peak_min = measure;

    float err = measure - setpoint;
    /* 滞回负反馈 bang-bang:误差过高输出 -h、过低输出 +h,把对象驱动到设定值附近振荡 */
    float out_old = rt->out;
    if (err > rt->mu)
        rt->out = -rt->h;
    else if (err < -rt->mu)
        rt->out = rt->h;
    int switched = (rt->out != out_old) ? 1 : 0;

    if (switched)
    {
        rt->half_cycles++;
        float a_half = (rt->peak_max - rt->peak_min) * 0.5f; /* 本半波幅值 */
        if ((rt->half_cycles & 1u) == 0u)
        {
            /* 完成一个完整周期:幅值/周期取平均 */
            float period = rt->t - rt->t_period_start;
            rt->period_accum += period;
            rt->a_accum += a_half;
            rt->full_cycles++;
            rt->Tu = rt->period_accum / (float)rt->full_cycles;
            rt->a = rt->a_accum / (float)rt->full_cycles;
            rt->t_period_start = rt->t;

            if (rt->full_cycles >= rt->cycles_needed)
            {
                /* Ku = 4h/(pi*a) */
                if (rt->a > 1e-6f)
                    rt->Ku = 4.0f * rt->h / (PIDT_PI * rt->a);
                rt->done = 1;
                rt->out = 0.0f;
            }
        }
        else
        {
            if (rt->half_cycles == 1)
                rt->t_period_start = rt->t;
        }
        /* 重置半波极值,开始新的半波观测 */
        rt->peak_max = measure;
        rt->peak_min = measure;
    }
    return rt->out;
}

int PIDRelayAutoTuneIsDone(const PID_RelayAutoTune_s *rt)
{
    return (rt->done) ? 1 : 0;
}

void PIDRelayAutoTuneGetKuTu(const PID_RelayAutoTune_s *rt, float *Ku, float *Tu)
{
    if (Ku)
        *Ku = rt->Ku;
    if (Tu)
        *Tu = rt->Tu;
}

PID_TuningResult_s PIDRelayAutoTuneTune(const PID_RelayAutoTune_s *rt)
{
    if (!rt->done || rt->Ku <= 0.0f || rt->Tu <= 0.0f)
        return PIDT_Make(0.0f, 0.0f, 0.0f);
    return PIDTuneZN_Oscillation(rt->Ku, rt->Tu);
}



