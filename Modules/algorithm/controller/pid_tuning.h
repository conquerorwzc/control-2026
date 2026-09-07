/**
 ******************************************************************************
 * @file     pid_tuning.h
 * @author   Codex
 * @version  V1.0.0
 * @date     2026-08-01
 * @brief    PID 参数整定工具:经验公式整定 + 继电反馈在线自整定
 *           覆盖方法:
 *           - Ziegler-Nichols 临界比例度法(振荡法,由 Ku/Tu)
 *           - Ziegler-Nichols 响应曲线法(由一阶惯性+纯滞后模型 K/L/T)
 *           - Cohen-Coon 反应曲线法
 *           - Lambda / SIMC(Skogestad)法
 *           - 4:1 衰减曲线法
 *           - Åström-Hägglund 继电反馈自整定(在线辨识 Ku/Tu,可配合 ZN 使用)
 ******************************************************************************
 */
#ifndef PID_TUNING_H
#define PID_TUNING_H

#include <stdint.h>

/* 整定结果:既给 Kp/Ki/Kd,也给 Ti/Td(Ki=Kp/Ti, Kd=Kp*Td) */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Ti;
    float Td;
} PID_TuningResult_s;

/**
 * @brief Ziegler-Nichols 临界比例度法(振荡法)
 * @param Ku 临界增益(系统等幅振荡时的比例增益)
 * @param Tu 临界周期(s)
 */
PID_TuningResult_s PIDTuneZN_Oscillation(float Ku, float Tu);

/**
 * @brief Ziegler-Nichols 响应曲线法
 * @param K 对象稳态增益(一阶惯性+纯滞后模型)
 * @param L 纯滞后时间(s)
 * @param T 时间常数(s)
 */
PID_TuningResult_s PIDTuneZN_ReactionCurve(float K, float L, float T);

/**
 * @brief Cohen-Coon 反应曲线法(比 Z-N 温和,适合自衡对象)
 */
PID_TuningResult_s PIDTuneCohenCoon(float K, float L, float T);

/**
 * @brief Lambda / SIMC(Skogestad)法:用期望闭环时间常数 Tc 直接设计,鲁棒性好
 * @param Tc 期望闭环时间常数,建议 Tc >= L
 */
PID_TuningResult_s PIDTuneSIMC(float K, float L, float T, float Tc);

/**
 * @brief 4:1 衰减曲线法
 * @param Kp_s 4:1 衰减振荡时的比例增益
 * @param Tu_s 4:1 衰减振荡周期(s)
 */
PID_TuningResult_s PIDTuneDecayCurve(float Kp_s, float Tu_s);

/* ---------------------- 继电反馈自整定 (Åström-Hägglund) ---------------------- */

typedef struct
{
    /* 配置 */
    float h;              /* 继电输出幅值(建议取满幅的 5%~10%) */
    float mu;             /* 切换滞回/死区(抗噪,建议取噪声幅值) */
    uint32_t cycles_needed; /* 需要观测的完整振荡周期数(>=2,0=默认3) */

    /* 运行状态 */
    float setpoint;       /* 设定值 */
    float out;            /* 当前继电输出 */
    float dt;             /* 采样周期(s) */
    float t;              /* 累计时间(s) */
    float peak_max;       /* 当前半波极大值 */
    float peak_min;       /* 当前半波极小值 */
    float a;              /* 振荡幅值估计 */
    float a_accum;        /* 幅值累加 */
    float period_accum;   /* 周期累加 */
    float Tu;             /* 临界周期估计 */
    float Ku;             /* 临界增益 Ku = 4h/(pi*a) */
    float t_period_start; /* 当前周期计时起点 */
    uint32_t half_cycles; /* 半波(切换)计数 */
    uint32_t full_cycles; /* 完整周期计数 */
    uint8_t initialized;
    uint8_t done;
} PID_RelayAutoTune_s;

/**
 * @brief 初始化继电反馈自整定器
 * @param h     继电输出幅值
 * @param mu    切换滞回(0 表示无滞回)
 * @param cycles 需要观测的周期数(0=默认3)
 * @return 0=成功
 */
int PIDRelayAutoTuneInit(PID_RelayAutoTune_s *rt, float h, float mu, uint32_t cycles);

/**
 * @brief 自整定单步:把返回值接入被控对象,替代 PID 输出
 * @param rt       自整定器
 * @param measure  对象当前测量值
 * @param setpoint 设定值(对象将被激励到该值附近振荡)
 * @param dt       本拍采样周期(s)
 * @return 继电输出(整定完成后返回 0)
 */
float PIDRelayAutoTuneStep(PID_RelayAutoTune_s *rt, float measure, float setpoint, float dt);

/**
 * @brief 整定是否完成
 */
int PIDRelayAutoTuneIsDone(const PID_RelayAutoTune_s *rt);

/**
 * @brief 获取辨识结果
 * @param Ku 临界增益输出
 * @param Tu 临界周期输出
 */
void PIDRelayAutoTuneGetKuTu(const PID_RelayAutoTune_s *rt, float *Ku, float *Tu);

/**
 * @brief 用 Z-N 振荡法由辨识出的 Ku/Tu 计算 PID 增益
 */
PID_TuningResult_s PIDRelayAutoTuneTune(const PID_RelayAutoTune_s *rt);

#endif
