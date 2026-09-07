/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @author modified by Codex (V2.0.0 全量优化落地)
 * @brief  PID 控制器实现:位置式/增量式 + 全套工程优化
 * @version V2.0.0
 * @date 2026-08-01
 *
 * 覆盖章节(与 README.md 一一对应):
 *  一、积分项:梯形积分 / 积分分离(滞回) / 条件积分 / 反计算抗饱和 / 变速积分 / 积分先行 / Kahan 求和
 *  二、微分项:微分先行 / 不完全微分 / 四点差分 / 微分增益限幅 / 测量预处理(均值/中值/LPF)
 *  三、控制结构:位置式/增量式 / 2-DOF / 前馈 / 串级 / 史密斯预估 / 增益调度
 *  四、工程细节:dt 钳位 / 输出限幅+软限幅 / 死区 / 无扰切换 / 设定值斜坡与滤波 / 坏值检测与故障安全
 */
#include "controller.h"
#include "memory.h"
#include <string.h>

#define PID_CLAMP(x, lo, hi) (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/* 一阶低通:out = alpha*new + (1-alpha)*last, alpha = dt/(RC+dt) */
static float PID_FilterLPF(float new_val, float last_val, float rc, float dt)
{
    float alpha = dt / (rc + dt);
    return alpha * new_val + (1.0f - alpha) * last_val;
}

/* ---------------------------- 积分项优化 ---------------------------- */

/* 变速积分:误差大时积分慢、误差小时积分快(也天然包含积分分离) */
static float f_ChangingIntegrationRate(PIDInstance *pid, float dI)
{
    if (pid->CoefA <= 0.0f)
        return dI;
    if (pid->Err * pid->Iout > 0.0f) /* 积分呈累积趋势时才调速 */
    {
        float ae = fabsf(pid->Err);
        if (ae <= pid->CoefB)
            return dI; /* 小误差,全速积分 */
        if (ae <= (pid->CoefA + pid->CoefB))
            return dI * ((pid->CoefA - ae + pid->CoefB) / pid->CoefA); /* 中误差,线性降速 */
        return 0.0f; /* 大误差,停止积分 */
    }
    return dI;
}

/* 积分分离(带滞回):返回 1 表示当前应暂停积分 */
static int f_IntegralSeparation(PIDInstance *pid)
{
    float ae = fabsf(pid->Err);
    if (!pid->SeparationActive)
    {
        if (ae > pid->IntegralSeparationEnter)
            pid->SeparationActive = 1; /* 进入:大偏差暂停积分 */
    }
    else
    {
        if (ae < pid->IntegralSeparationExit)
            pid->SeparationActive = 0; /* 退出:回到稳态附近恢复积分(滞回避免临界抖动) */
    }
    return pid->SeparationActive;
}

/* 条件积分/饱和冻结:输出饱和且误差方向仍加深饱和时,本拍不积分 */
static int f_IntegrationBlocked(PIDInstance *pid, float u_without_i)
{
    if (pid->Improve & PID_Conditional_Integral)
    {
        if (pid->SatFlag && pid->Err * pid->Output >= 0.0f)
            return 1;
    }
    /* 兼容旧行为:PID_Integral_Limit 标志下也做饱和冻结(原 f_Integral_Limit 逻辑) */
    if (pid->Improve & PID_Integral_Limit)
    {
        if (fabsf(u_without_i) > pid->MaxOut && pid->Err * pid->Iout > 0.0f)
            return 1;
    }
    return 0;
}

/* 反计算抗饱和:饱和时按 Kb 把超出的输出量折算回积分项,退出饱和干脆利落 */
static void f_BackCalculation(PIDInstance *pid, float u_unclamped, float u_clamped)
{
    if (u_unclamped == u_clamped)
        return;
    float kb = (pid->BackCalcGain > 0.0f) ? pid->BackCalcGain
                                          : ((pid->Ki > 0.0f) ? pid->Ki : pid->Kp);
    pid->Iout += kb * (u_clamped - u_unclamped);
    if (pid->Improve & PID_Integral_Limit)
        pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);
}

/* ---------------------------- 微分项优化 ---------------------------- */

/*
 * 计算微分分量(返回 Kd*derivative)
 * - PID_Derivative_On_Measurement / 2-DOF c 加权:只对(或部分对)测量值微分,避免设定值阶跃冲击
 * - PID_FourPointDiff:四点中心差分,噪声小于两点差分(隐式低通)
 * - PID_DerivativeFilter:不完全微分(一阶惯性),滤除高频噪声
 * - PID_DerivativeLimit:微分增益限幅,防噪声尖峰打满输出
 */
static float f_ComputeDerivative(PIDInstance *pid, float dt)
{
    float c;
    if (pid->Improve & PID_2DOF)
        c = pid->SetpointWeightD;
    else if (pid->Improve & PID_Derivative_On_Measurement)
        c = 0.0f; /* 微分先行:仅对测量值微分 */
    else
        c = 1.0f;

    float ed0 = c * pid->Ref_Filtered - pid->Measure_Filtered;
    float ed1 = c * pid->Last_Ref_Filtered - pid->Last_Measure_Filtered;
    float d;
    if (pid->Improve & PID_FourPointDiff)
    {
        float ed2 = c * pid->Prev2_Ref_Filtered - pid->Prev2_Measure_Filtered;
        float ed3 = c * pid->Prev3_Ref_Filtered - pid->Prev3_Measure_Filtered;
        d = (ed0 + 3.0f * ed1 - 3.0f * ed2 - ed3) / (6.0f * dt);
    }
    else
    {
        d = (ed0 - ed1) / dt; /* 两点差分 */
    }
    d *= pid->Kd;

    if (pid->Improve & PID_DerivativeFilter)
        d = PID_FilterLPF(d, pid->Last_Dout, pid->Derivative_LPF_RC, dt);

    if ((pid->Improve & PID_DerivativeLimit) && pid->DerivativeLimit > 0.0f)
        d = PID_CLAMP(d, -pid->DerivativeLimit, pid->DerivativeLimit);

    return d;
}

/* ---------------------------- 测量预处理 ---------------------------- */

/* 移动平均滤波 */
static float f_MeasureMovingAverage(PIDInstance *pid, float raw)
{
    uint8_t w = pid->MeasureFilterWindow;
    if (w > PID_MEASURE_FILTER_MAX)
        w = PID_MEASURE_FILTER_MAX;
    pid->MeasureBuf[pid->MeasureBufIdx] = raw;
    pid->MeasureBufIdx = (uint8_t)((pid->MeasureBufIdx + 1) % w);
    if (pid->MeasureBufCount < w)
        pid->MeasureBufCount++;
    else if (pid->MeasureBufCount > w) /* 窗口被调小时钳位,防读到陈旧数据 */
        pid->MeasureBufCount = w;
    float sum = 0.0f;
    for (uint8_t i = 0; i < pid->MeasureBufCount; i++)
        sum += pid->MeasureBuf[i];
    return sum / (float)pid->MeasureBufCount;
}

/* 中值滤波(插入排序,窗口通常 3/5) */
static float f_MeasureMedianFilter(PIDInstance *pid, float raw)
{
    uint8_t w = pid->MeasureFilterWindow;
    if (w > PID_MEASURE_FILTER_MAX)
        w = PID_MEASURE_FILTER_MAX;
    pid->MeasureBuf[pid->MeasureBufIdx] = raw;
    pid->MeasureBufIdx = (uint8_t)((pid->MeasureBufIdx + 1) % w);
    if (pid->MeasureBufCount < w)
        pid->MeasureBufCount++;
    else if (pid->MeasureBufCount > w)
        pid->MeasureBufCount = w;
    uint8_t n = pid->MeasureBufCount;
    float tmp[PID_MEASURE_FILTER_MAX];
    for (uint8_t i = 0; i < n; i++)
        tmp[i] = pid->MeasureBuf[i];
    for (uint8_t i = 1; i < n; i++)
    {
        float key = tmp[i];
        int j = (int)i - 1;
        while (j >= 0 && tmp[j] > key)
        {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    if (n & 1u)
        return tmp[n / 2];
    return (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

/* ---------------------------- 堵转检测 ---------------------------- */

/* 电机堵转检测(原有功能,保留) */
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;
    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }
    if (pid->ERRORHandler.ERRORCount > 200)
    {
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
        pid->ERRORHandler.ERRORCount = 0;
    }
}

/* ---------------------------- 核心计算 ---------------------------- */

static float PID_CalculateCore(PIDInstance *pid, float measure, float ref, float feedforward)
{
    /* 堵转检测(使用上一拍数据,保持原行为) */
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    /* 1. 采样周期:实测 dt + 可选钳位(防调度抖动/异常间隔) */
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
    if (pid->DtMax > 0.0f)
    {
        float dmin = (pid->DtMin > 0.0f) ? pid->DtMin : 1e-6f;
        pid->dt = PID_CLAMP(pid->dt, dmin, pid->DtMax);
    }
    float dt = pid->dt;

    /* 2. 测量坏值检测:超范围/跳变过大 -> 保持上一有效值;连续超阈值 -> 故障安全输出 */
    float prev_raw = pid->Last_Measure;
    float raw = measure;
    int bad = 0;
    if (pid->Improve & PID_MeasureCheck)
    {
        if (raw < pid->MeasureMin || raw > pid->MeasureMax)
            bad = 1;
        else if (pid->MeasureReady && pid->MeasureJumpLimit > 0.0f &&
                 fabsf(raw - prev_raw) > pid->MeasureJumpLimit)
            bad = 1;
        if (bad)
        {
            pid->MeasureErrorCount++;
            raw = prev_raw; /* 保持上一有效值 */
            if (pid->MeasureErrorCount >= pid->MeasureErrorThreshold)
            {
                pid->ERRORHandler.ERRORType = PID_MEASURE_FAULT_ERROR;
                pid->Measure = raw;
                pid->Last_Measure = raw;
                pid->Output = pid->FaultSafeOutput;
                pid->Last_Output = pid->Output;
                return pid->Output;
            }
        }
        else
        {
            pid->MeasureErrorCount = 0;
            pid->MeasureReady = 1;
        }
    }
    pid->Last_Measure = raw;
    pid->Measure = raw;

    /* 3. 测量滤波历史移位 + 滤波 */
    pid->Prev3_Measure_Filtered = pid->Prev2_Measure_Filtered;
    pid->Prev2_Measure_Filtered = pid->Last_Measure_Filtered;
    pid->Last_Measure_Filtered = pid->Measure_Filtered;

    float m_f = raw;
    if (pid->Improve & PID_MeasurementFilter)
    {
        if (pid->MeasureFilterMode == 1 && pid->MeasureFilterWindow >= 2)
            m_f = f_MeasureMovingAverage(pid, raw);
        else if (pid->MeasureFilterMode == 2 && pid->MeasureFilterWindow >= 3)
            m_f = f_MeasureMedianFilter(pid, raw);
        if (pid->MeasureFilterRC > 0.0f)
            m_f = PID_FilterLPF(m_f, pid->Measure_Filtered, pid->MeasureFilterRC, dt);
    }
    pid->Measure_Filtered = m_f;

    /* 4. 设定值斜坡/滤波历史移位 + 处理 */
    pid->Prev3_Ref_Filtered = pid->Prev2_Ref_Filtered;
    pid->Prev2_Ref_Filtered = pid->Last_Ref_Filtered;
    pid->Last_Ref_Filtered = pid->Ref_Filtered;

    pid->Ref = ref;
    float r = ref;
    if ((pid->Improve & PID_SetpointRamp) && pid->SetpointRampRate > 0.0f)
    {
        float max_step = pid->SetpointRampRate * dt;
        float delta = PID_CLAMP(ref - pid->Ref_Filtered, -max_step, max_step);
        r = pid->Ref_Filtered + delta; /* 设定值斜坡:按斜率逼近目标 */
    }
    if (pid->Improve & PID_SetpointFilter)
        r = PID_FilterLPF(r, pid->Ref_Filtered, pid->SetpointFilterRC, dt); /* 设定值平滑 */
    pid->Ref_Filtered = r;

    /* 5. 误差(使用滤波后测量与处理后的设定值) */
    pid->Err = pid->Ref_Filtered - pid->Measure_Filtered;
    float e = pid->Err;

    /* 6. 死区 */
    if (fabsf(e) <= pid->DeadBand)
    {
        pid->ITerm = 0.0f;
        if (pid->Improve & PID_Incremental)
        {
            /* 增量式:输出为累积量,进入死区保持(冻结) */
        }
        else
        {
            pid->Output = 0.0f; /* 位置式:兼容原行为,输出清零 */
        }
        goto epilogue;
    }

    /* 比例项设定值加权 b(2-DOF / 比例先行) */
    float b;
    if (pid->Improve & PID_2DOF)
        b = pid->SetpointWeightP;
    else if (pid->Improve & PID_Proportional_On_Measurement)
        b = 0.0f;
    else
        b = 1.0f;

    if (pid->Improve & PID_Incremental)
    {
        /* ================= 增量式 PID =================
         * u(k) = u(k-1) + dP + dI + dD + dFF
         * 输出为累积量并被限幅,天然无积分饱和、无扰切换简单 */
        float m0 = pid->Measure_Filtered;
        float m1 = pid->Last_Measure_Filtered;
        float r0 = pid->Ref_Filtered;
        float r1 = pid->Last_Ref_Filtered;

        float dP = pid->Kp * ((b * r0 - m0) - (b * r1 - m1)); /* 比例增量 */

        float dI = pid->Ki * e * dt;
        if (pid->Improve & PID_Trapezoid_Intergral)
            dI = pid->Ki * (e + pid->Last_Err) * 0.5f * dt; /* 梯形积分 */
        if ((pid->Improve & PID_Integral_Separation) && f_IntegralSeparation(pid))
            dI = 0.0f; /* 积分分离 */
        if (pid->Improve & PID_ChangingIntegrationRate)
            dI = f_ChangingIntegrationRate(pid, dI); /* 变速积分 */
        if ((pid->Improve & PID_Conditional_Integral) && pid->SatFlag &&
            e * pid->Output >= 0.0f)
            dI = 0.0f; /* 条件积分 */

        float D_now = f_ComputeDerivative(pid, dt);
        float dD = D_now - pid->Last_Dout; /* 微分增量 */
        pid->Last_Dout = D_now;

        float dFF = feedforward - pid->Last_Feedforward; /* 前馈增量 */
        pid->Last_Feedforward = feedforward;
        pid->Feedforward = feedforward;

        float du = dP + dI + dD + dFF;
        pid->Pout = dP;
        pid->Dout = D_now;
        pid->ITerm = dI;
        pid->Iout += dI; /* 仅作观测,不参与控制(天然无 windup) */
        if (pid->Improve & PID_Integral_Limit)
            pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);

        float u = pid->Output + du;

        /* 输出滤波 */
        if (pid->Improve & PID_OutputFilter)
            u = PID_FilterLPF(u, pid->Last_Output, pid->Output_LPF_RC, dt);
        /* 软限幅(变化率) */
        if ((pid->Improve & PID_SlewRateLimit) && pid->SlewRateLimit > 0.0f)
        {
            float maxd = pid->SlewRateLimit * dt;
            u = pid->Last_Output + PID_CLAMP(u - pid->Last_Output, -maxd, maxd);
        }
        /* 硬限幅 */
        u = PID_CLAMP(u, -pid->MaxOut, pid->MaxOut);
        pid->SatFlag = (pid->MaxOut > 0.0f && fabsf(u) >= pid->MaxOut) ? 1u : 0u;
        pid->Output = u;
    }
    else
    {
        /* ================= 位置式 PID ================= */
        float Pout = pid->Kp * (b * pid->Ref_Filtered - pid->Measure_Filtered);
        float Dout = f_ComputeDerivative(pid, dt);

        float dI = pid->Ki * e * dt;
        if (pid->Improve & PID_Trapezoid_Intergral)
            dI = pid->Ki * (e + pid->Last_Err) * 0.5f * dt;
        if ((pid->Improve & PID_Integral_Separation) && f_IntegralSeparation(pid))
            dI = 0.0f;
        if (pid->Improve & PID_ChangingIntegrationRate)
            dI = f_ChangingIntegrationRate(pid, dI);

        /* 条件积分/饱和冻结(用旧 Iout 判断,兼容原实现) */
        float u_without_i = Pout + pid->Iout + Dout + feedforward;
        if (f_IntegrationBlocked(pid, u_without_i))
            dI = 0.0f;
        pid->ITerm = dI;

        /* 积分累加(Kahan 补偿求和,避免大数吃小量) */
        if (pid->Improve & PID_Kahan)
        {
            float y = dI - pid->I_Comp;
            float t = pid->Iout + y;
            pid->I_Comp = (t - pid->Iout) - y;
            pid->Iout = t;
        }
        else
        {
            pid->Iout += dI;
        }
        if (pid->Improve & PID_Integral_Limit)
            pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);

        pid->Pout = Pout;
        pid->Dout = Dout;
        pid->Feedforward = feedforward;

        float u = Pout + pid->Iout + Dout + feedforward;

        /* 反计算抗饱和 */
        if (pid->Improve & PID_BackCalculation)
        {
            float u_clamped = PID_CLAMP(u, -pid->MaxOut, pid->MaxOut);
            f_BackCalculation(pid, u, u_clamped);
            u = u_clamped;
        }

        /* 输出滤波 */
        if (pid->Improve & PID_OutputFilter)
            u = PID_FilterLPF(u, pid->Last_Output, pid->Output_LPF_RC, dt);
        /* 软限幅(变化率) */
        if ((pid->Improve & PID_SlewRateLimit) && pid->SlewRateLimit > 0.0f)
        {
            float maxd = pid->SlewRateLimit * dt;
            u = pid->Last_Output + PID_CLAMP(u - pid->Last_Output, -maxd, maxd);
        }
        /* 硬限幅 */
        u = PID_CLAMP(u, -pid->MaxOut, pid->MaxOut);
        pid->SatFlag = (pid->MaxOut > 0.0f && fabsf(u) >= pid->MaxOut) ? 1u : 0u;
        pid->Output = u;
    }

epilogue:
    /* 保存当前数据,用于下次计算 */
    pid->Prev3_Err = pid->Prev2_Err;
    pid->Prev2_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_ITerm = pid->ITerm;
    if (!(pid->Improve & PID_Incremental))
        pid->Last_Dout = pid->Dout;

    return pid->Output;
}

/* ---------------------------- 外部算法接口 ---------------------------- */

/* 将 PIDInstance 配置块快照到 PID_Init_Config_s(供 PIDReset 复用) */
static void PID_InstanceToConfig(const PIDInstance *pid, PID_Init_Config_s *cfg)
{
    cfg->Kp = pid->Kp;
    cfg->Ki = pid->Ki;
    cfg->Kd = pid->Kd;
    cfg->MaxOut = pid->MaxOut;
    cfg->DeadBand = pid->DeadBand;
    cfg->Improve = (PID_Improvement_e)pid->Improve;
    cfg->IntegralLimit = pid->IntegralLimit;
    cfg->CoefA = pid->CoefA;
    cfg->CoefB = pid->CoefB;
    cfg->IntegralSeparationEnter = pid->IntegralSeparationEnter;
    cfg->IntegralSeparationExit = pid->IntegralSeparationExit;
    cfg->BackCalcGain = pid->BackCalcGain;
    cfg->IntegralPreload = pid->IntegralPreload;
    cfg->Derivative_LPF_RC = pid->Derivative_LPF_RC;
    cfg->DerivativeLimit = pid->DerivativeLimit;
    cfg->SetpointWeightP = pid->SetpointWeightP;
    cfg->SetpointWeightD = pid->SetpointWeightD;
    cfg->SetpointRampRate = pid->SetpointRampRate;
    cfg->SetpointFilterRC = pid->SetpointFilterRC;
    cfg->Output_LPF_RC = pid->Output_LPF_RC;
    cfg->SlewRateLimit = pid->SlewRateLimit;
    cfg->MeasureMin = pid->MeasureMin;
    cfg->MeasureMax = pid->MeasureMax;
    cfg->MeasureJumpLimit = pid->MeasureJumpLimit;
    cfg->FaultSafeOutput = pid->FaultSafeOutput;
    cfg->MeasureErrorThreshold = pid->MeasureErrorThreshold;
    cfg->MeasureFilterMode = pid->MeasureFilterMode;
    cfg->MeasureFilterWindow = pid->MeasureFilterWindow;
    cfg->MeasureFilterRC = pid->MeasureFilterRC;
    cfg->DtMin = pid->DtMin;
    cfg->DtMax = pid->DtMax;
}

/* 将配置应用到 PIDInstance(公共部分) */
static void PID_ConfigToInstance(PIDInstance *pid, const PID_Init_Config_s *cfg)
{
    pid->Kp = cfg->Kp;
    pid->Ki = cfg->Ki;
    pid->Kd = cfg->Kd;
    pid->MaxOut = cfg->MaxOut;
    pid->DeadBand = cfg->DeadBand;
    pid->Improve = (uint32_t)cfg->Improve;
    pid->IntegralLimit = cfg->IntegralLimit;
    pid->CoefA = cfg->CoefA;
    pid->CoefB = cfg->CoefB;
    pid->IntegralSeparationEnter = cfg->IntegralSeparationEnter;
    pid->IntegralSeparationExit = cfg->IntegralSeparationExit;
    pid->BackCalcGain = cfg->BackCalcGain;
    pid->IntegralPreload = cfg->IntegralPreload;
    pid->Derivative_LPF_RC = cfg->Derivative_LPF_RC;
    pid->DerivativeLimit = cfg->DerivativeLimit;
    pid->SetpointWeightP = cfg->SetpointWeightP;
    pid->SetpointWeightD = cfg->SetpointWeightD;
    pid->SetpointRampRate = cfg->SetpointRampRate;
    pid->SetpointFilterRC = cfg->SetpointFilterRC;
    pid->Output_LPF_RC = cfg->Output_LPF_RC;
    pid->SlewRateLimit = cfg->SlewRateLimit;
    pid->MeasureMin = cfg->MeasureMin;
    pid->MeasureMax = cfg->MeasureMax;
    pid->MeasureJumpLimit = cfg->MeasureJumpLimit;
    pid->FaultSafeOutput = cfg->FaultSafeOutput;
    pid->MeasureErrorThreshold = cfg->MeasureErrorThreshold;
    pid->MeasureFilterMode = cfg->MeasureFilterMode;
    pid->MeasureFilterWindow = cfg->MeasureFilterWindow;
    pid->MeasureFilterRC = cfg->MeasureFilterRC;
    pid->DtMin = cfg->DtMin;
    pid->DtMax = cfg->DtMax;

    /* 归一化默认值 */
    if (pid->MeasureErrorThreshold == 0)
        pid->MeasureErrorThreshold = 10;
    if (pid->IntegralSeparationExit <= 0.0f)
        pid->IntegralSeparationExit = pid->IntegralSeparationEnter;
}

/**
 * @brief 初始化 PID,设置参数和启用的优化环节,将其他数据置零
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    memset(pid, 0, sizeof(PIDInstance));
    PID_ConfigToInstance(pid, config);
    pid->Iout = pid->IntegralPreload; /* 积分先行:启动时预置积分项 */
    if (pid->Improve & PID_Integral_Limit) /* 预置值同样受积分限幅约束 */
        pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);
    DWT_GetDeltaT(&pid->DWT_CNT);     /* 建立 DWT 时间基准 */
}

/**
 * @brief 复位运行状态(保留配置)
 */
void PIDReset(PIDInstance *pid)
{
    PID_Init_Config_s cfg;
    PID_InstanceToConfig(pid, &cfg);
    PIDInit(pid, &cfg);
}

/**
 * @brief PID 计算
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    float ff = (pid->Improve & PID_Feedforward) ? pid->Feedforward : 0.0f;
    return PID_CalculateCore(pid, measure, ref, ff);
}

/**
 * @brief 带前馈的 PID 计算
 */
float PIDCalculateEx(PIDInstance *pid, float measure, float ref, float feedforward)
{
    pid->Feedforward = feedforward;
    return PID_CalculateCore(pid, measure, ref, feedforward);
}

/**
 * @brief 在线修改 PID 增益(积分项保持不变 -> 无扰)
 */
void PIDSetGains(PIDInstance *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * @brief 预置积分项(积分先行/切换前预装载)
 */
void PIDSetIntegral(PIDInstance *pid, float value)
{
    pid->Iout = value;
    if (pid->Improve & PID_Integral_Limit)
        pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);
}

/**
 * @brief 无扰切换/跟踪:强制输出指定值并反解积分项
 */
void PIDSetOutput(PIDInstance *pid, float value)
{
    float v = PID_CLAMP(value, -pid->MaxOut, pid->MaxOut);
    if (pid->Improve & PID_Incremental)
    {
        /* 增量式:输出即累积状态,直接覆盖 */
        pid->Output = v;
        pid->Last_Output = v;
        return;
    }
    /* 位置式:反解积分项,保证切换后输出连续 */
    float i = v - pid->Pout - pid->Dout;
    pid->Iout = i;
    if (pid->Improve & PID_Integral_Limit)
        pid->Iout = PID_CLAMP(pid->Iout, -pid->IntegralLimit, pid->IntegralLimit);
    pid->Output = v;
    pid->Last_Output = v;
}

/**
 * @brief 立即设定设定值(跳过斜坡过渡)
 */
void PIDSetSetpoint(PIDInstance *pid, float ref)
{
    pid->Ref = ref;
    pid->Ref_Filtered = ref;
    pid->Last_Ref_Filtered = ref;
}

/* ---------------------------- 增益调度 ---------------------------- */

/**
 * @brief 增益调度:按工况量 x 线性插值切换 Kp/Ki/Kd(积分项保持不变,无扰)
 */
void PIDGainScheduleApply(PIDInstance *pid, PID_GainSchedule_s *sched, float x)
{
    uint8_t n = sched->count;
    if (n == 0)
        return;
    float kp, ki, kd;
    if (x <= sched->breakpoints[0] || n == 1)
    {
        kp = sched->gains[0].Kp;
        ki = sched->gains[0].Ki;
        kd = sched->gains[0].Kd;
    }
    else if (x >= sched->breakpoints[n - 1])
    {
        kp = sched->gains[n - 1].Kp;
        ki = sched->gains[n - 1].Ki;
        kd = sched->gains[n - 1].Kd;
    }
    else
    {
        uint8_t i = 0;
        while (i < n - 1 && x > sched->breakpoints[i + 1])
            i++;
        float x0 = sched->breakpoints[i];
        float x1 = sched->breakpoints[i + 1];
        float w = (x - x0) / (x1 - x0);
        kp = sched->gains[i].Kp + w * (sched->gains[i + 1].Kp - sched->gains[i].Kp);
        ki = sched->gains[i].Ki + w * (sched->gains[i + 1].Ki - sched->gains[i].Ki);
        kd = sched->gains[i].Kd + w * (sched->gains[i + 1].Kd - sched->gains[i].Kd);
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    sched->Kp = kp;
    sched->Ki = ki;
    sched->Kd = kd;
}

/* ---------------------------- 串级 PID ---------------------------- */

void PIDCascadeInit(PID_Cascade_s *c, PID_Init_Config_s *outer_cfg, PID_Init_Config_s *inner_cfg)
{
    PIDInit(&c->outer, outer_cfg);
    PIDInit(&c->inner, inner_cfg);
}

/**
 * @brief 串级 PID 计算:外环输出(已按 outer.MaxOut 限幅)作为内环设定值
 */
float PIDCascadeCalculate(PID_Cascade_s *c, float measure_outer, float ref_outer, float measure_inner)
{
    float inner_ref = PIDCalculate(&c->outer, measure_outer, ref_outer);
    return PIDCalculate(&c->inner, measure_inner, inner_ref);
}

/* ---------------------------- 史密斯预估器 ---------------------------- */

void PIDSmithInit(PID_SmithPredictor_s *sp, PID_Init_Config_s *pid_cfg, float K, float T,
                  float delay_time, float dt)
{
    PIDInit(&sp->pid, pid_cfg);
    sp->ModelGain = K;
    sp->ModelTau = (T > 0.0f) ? T : 0.001f;
    sp->DelayTime = delay_time;
    sp->dt = (dt > 0.0f) ? dt : 0.001f;
    sp->delay_steps = (uint16_t)(delay_time / sp->dt + 0.5f);
    if (sp->delay_steps > PID_SMITH_MAX_DELAY)
        sp->delay_steps = PID_SMITH_MAX_DELAY;
    sp->head = 0;
    sp->y_model = 0.0f;
    sp->y_delayed = 0.0f;
    sp->y_pred = 0.0f;
    memset(sp->u_delay_buf, 0, sizeof(sp->u_delay_buf));
    sp->pid.Improve |= PID_SmithPredictor; /* 标记该实例运行史密斯预估 */
}

/**
 * @brief 史密斯预估 PID:用对象模型预估无滞后输出,提前对误差做出反应
 *        标准结构: y_pred = y_model(无滞后模型) + (y_meas - y_delayed(带滞后模型))
 *        即:测量值中的滞后部分由带滞后模型替换为无滞后模型输出,
 *        控制器看到的是"去掉纯滞后"的对象,从而允许更高增益而不振荡。
 */
float PIDSmithCalculate(PID_SmithPredictor_s *sp, float measure, float ref)
{
    float u_prev = sp->pid.Output; /* 上一拍控制量 */

    /* 控制量延迟线:写入当前,读取 L 拍前(L=delay_steps) */
    sp->u_delay_buf[sp->head] = u_prev;
    sp->head = (uint16_t)((sp->head + 1) % (uint16_t)(sp->delay_steps + 1));
    float u_delayed = sp->u_delay_buf[sp->head];

    float k = sp->dt / sp->ModelTau;
    /* 无滞后模型:输入为当前控制量 */
    sp->y_model += k * (sp->ModelGain * u_prev - sp->y_model);
    /* 带滞后模型:输入为 L 拍前的控制量 */
    sp->y_delayed += k * (sp->ModelGain * u_delayed - sp->y_delayed);
    /* 预估输出 = 无滞后模型输出 + (实测 - 带滞后模型输出)  [修正:原实现写反] */
    sp->y_pred = sp->y_model + (measure - sp->y_delayed);

    return PIDCalculate(&sp->pid, sp->y_pred, ref);
}









