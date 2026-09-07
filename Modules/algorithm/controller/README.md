# PID 控制器库(Modules/algorithm/controller)

`controller.h/.c` 提供位置式/增量式 PID 及全套工程优化;`pid_tuning.h/.c` 提供参数整定工具。
旧接口 `PIDInit(PIDInstance*, PID_Init_Config_s*)` 与 `PIDCalculate(pid, measure, ref)` **完全兼容**,旧标志位取值不变。

- 优化开关统一写在 `config.Improve`(位标志,可用 `|` 组合)。
- 所有 `Improve` 未开启的行为与旧版一致。
- 每个优化项在下方按「指南章节 → 标志/字段/API → 说明」列出。

## 一、积分项优化

| # | 优化项 | 启用方式 | 说明 |
|---|--------|----------|------|
| 1.1 | 梯形积分 | `PID_Trapezoid_Intergral` | ΔI = Ki·(e+e₋₁)/2·dt,精度高、纹波小,嵌入式几乎零成本,推荐默认开启 |
| 1.2 | 积分分离(带滞回) | `PID_Integral_Separation` + `IntegralSeparationEnter` / `IntegralSeparationExit` | \|e\|>Enter 暂停积分、\|e\|<Exit 恢复;Exit≤Enter 形成滞回,防临界抖动(Exit=0 时与 Enter 相同,无滞回) |
| 1.3a | 积分限幅 | `PID_Integral_Limit` + `IntegralLimit` | Iout 钳位到 ±IntegralLimit,最简单抗饱和 |
| 1.3b | 条件积分 | `PID_Conditional_Integral` | 输出饱和且误差仍加深饱和时冻结积分 |
| 1.3c | 反计算抗饱和 | `PID_BackCalculation` + `BackCalcGain` | 饱和时 Iout += Kb·(u_sat − u_unsat);Kb=0 自动取 Ki(无 Ki 取 Kp) |
| 1.3d | 变速积分 | `PID_ChangingIntegrationRate` + `CoefA` / `CoefB` | 误差小全速积分、误差大减速/停止 |
| 1.3e | 积分先行 | `IntegralPreload`(初始化预置)+ `PIDSetIntegral()`(运行期) | 启动/切换前把积分预置为接近稳态所需值,避免积分爬升滞后期 |
| 1.4 | Kahan 补偿求和 | `PID_Kahan` | 大 Iout + 小 ΔI 时补偿项避免小量被"吃掉" |

## 二、微分项优化

| # | 优化项 | 启用方式 | 说明 |
|---|--------|----------|------|
| 2.1 | 微分先行 | `PID_Derivative_On_Measurement` | 微分只作用于测量值,设定值阶跃无微分冲击 |
| 2.2 | 不完全微分 | `PID_DerivativeFilter` + `Derivative_LPF_RC` | 微分串一阶惯性,大幅衰减高频噪声 |
| 2.3 | 四点差分法 | `PID_FourPointDiff` | (e₀+3e₋₁−3e₋₂−e₋₃)/6T,比两点差分噪声小 |
| 2.4 | 微分增益限幅 | `PID_DerivativeLimit` + `DerivativeLimit` | \|D\|≤Dmax,防噪声尖峰打满输出 |
| 2.5 | 测量预处理 | `PID_MeasurementFilter` + `MeasureFilterMode`(1 均值/2 中值)+ `MeasureFilterWindow` + `MeasureFilterRC`(可选 LPF) | 滤波后再进控制律;相位滞后需计入整定 |

## 三、控制结构优化

| # | 优化项 | 启用方式 | 说明 |
|---|--------|----------|------|
| 3.1 | 增量式 PID | `PID_Incremental` | u(k)=u(k−1)+Δu,输出累积+限幅,天然无积分饱和、无扰切换简单 |
| 3.2 | 二自由度 2-DOF | `PID_2DOF` + `SetpointWeightP`(b)+ `SetpointWeightD`(c) | u=Kp(b·r−y)+Ki∫(r−y)+Kd·d(c·r−y)/dt,独立调节跟踪/抗扰 |
| 3.3 | 前馈补偿 | `PID_Feedforward` + `pid.Feedforward` 字段,或 `PIDCalculateEx(pid, m, r, ff)` | 前馈直叠加到输出,让 PID 只处理残余误差 |
| 3.4 | 串级 PID | `PID_Cascade_s` + `PIDCascadeInit/Calculate` | 外环(慢)输出经限幅作内环(快)设定值 |
| 3.5 | 史密斯预估器 | `PID_SmithPredictor_s` + `PIDSmithInit/Calculate` | FOPDT 模型预估无滞后输出,改善大纯滞后稳定性 |
| 3.6 | 增益调度 | `PID_GainSchedule_s` + `PIDGainScheduleApply(pid, sched, x)` | 按工况量 x 查表线性插值切换 Kp/Ki/Kd(积分不变,无扰) |

## 四、数字实现与工程细节

| # | 优化项 | 启用方式 | 说明 |
|---|--------|----------|------|
| 4.1 | 采样周期 | `DtMin` / `DtMax` | 对 DWT 实测 dt 做钳位,防调度抖动/异常间隔;多闭环内环频率应高于外环 |
| 4.2 | 计算形式 | 累加式 Iout / `PID_Kahan` | 积分用累积变量避免每拍从头累加;定点注意 32/64 位中间量防溢出 |
| 4.3a | 输出硬限幅 | `MaxOut`(恒生效) | 输出截断 |
| 4.3b | 输出软限幅 | `PID_SlewRateLimit` + `SlewRateLimit`(单位/s) | \|u(k)−u(k−1)\|≤Δu_max,保护阀门/伺服执行机构 |
| 4.3c | 死区 | `DeadBand`(恒生效) | \|e\|≤DeadBand 时位置式输出 0、增量式保持 |
| 4.4 | 无扰切换 | `PIDSetOutput()`(跟踪/强制输出并反解积分)、`PIDSetIntegral()`、`PIDSetGains()`(保积分) | 手动↔自动、控制器切换时输出连续无跳变 |
| 4.5a | 设定值斜坡 | `PID_SetpointRamp` + `SetpointRampRate` | r 按斜率逼近目标,消除阶跃冲击 |
| 4.5b | 设定值滤波 | `PID_SetpointFilter` + `SetpointFilterRC` | 设定值一阶惯性平滑 |
| 4.6 | 测量坏值/故障 | `PID_MeasureCheck` + `MeasureMin/Max`、`MeasureJumpLimit`、`MeasureErrorThreshold`、`FaultSafeOutput` | 坏值保持上一有效值;连续坏值输出安全值并置 `PID_MEASURE_FAULT_ERROR` |

## 五、参数整定(pid_tuning.h)

| 方法 | 函数 | 输入 |
|------|------|------|
| Z-N 临界比例度法 | `PIDTuneZN_Oscillation(Ku, Tu)` | 临界增益/周期 |
| Z-N 响应曲线法 | `PIDTuneZN_ReactionCurve(K, L, T)` | 一阶+滞后模型 |
| Cohen-Coon | `PIDTuneCohenCoon(K, L, T)` | 同上,更温和 |
| Lambda / SIMC | `PIDTuneSIMC(K, L, T, Tc)` | Tc=期望闭环时间常数(≥L) |
| 4:1 衰减曲线法 | `PIDTuneDecayCurve(Kp_s, Tu_s)` | 4:1 衰减时增益/周期 |
| 继电反馈自整定 | `PIDRelayAutoTuneInit/Step/IsDone/GetKuTu/Tune` | 在线激励测 Ku/Tu,可直接接 Z-N |

```c
// 继电自整定接入示例(替代 PID 输出):
PID_RelayAutoTune_s rt;
PIDRelayAutoTuneInit(&rt, /*h*/0.5f, /*mu*/0.01f, /*cycles*/3);
while (!PIDRelayAutoTuneIsDone(&rt)) {
    float u = PIDRelayAutoTuneStep(&rt, measure, setpoint, dt);
    /* u 送给执行器 */
}
PID_TuningResult_s g = PIDRelayAutoTuneTune(&rt); // 得到 Kp/Ki/Kd
```

## 六、进阶方向(替代与增强)

- 增益调度、前馈、非线性死区已落地(见上)。
- MPC / LQR / H∞ / 模糊 / 神经网络 PID 属于独立控制框架,不在本库范围;如需 DOB(扰动观测器)等补偿,可基于 `PIDCalculateEx` 的前馈通道接入。

## 常用组合建议

```c
PID_Init_Config_s cfg = {
    .Kp = ..., .Ki = ..., .Kd = ...,
    .MaxOut = ..., .DeadBand = ...,
    .IntegralLimit = ...,
    .Improve = PID_Trapezoid_Intergral            /* 梯形积分 */
             | PID_Integral_Limit                 /* 积分限幅 */
             | PID_Derivative_On_Measurement      /* 微分先行 */
             | PID_DerivativeFilter,              /* 不完全微分 */
    .Derivative_LPF_RC = ...,
};
PIDInstance pid;
PIDInit(&pid, &cfg);
float out = PIDCalculate(&pid, measure, ref);
```
