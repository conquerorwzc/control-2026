/**
 ******************************************************************************
 * @file     controller.h
 * @author   Wang Hongxi
 * @author   modified by Codex
 * @version  V2.0.0
 * @date     2026-08-01
 * @brief    PID 控制器库:位置式/增量式 + 全套工程优化
 *           覆盖:积分优化(梯形积分/积分分离/条件积分/反计算抗饱和/变速积分/积分先行/Kahan)
 *                微分优化(微分先行/不完全微分/四点差分/微分增益限幅)
 *                结构优化(增量式/2-DOF/前馈/串级/史密斯预估/增益调度)
 *                工程细节(dt 钳位/软限幅/死区/无扰切换/设定值斜坡与滤波/坏值检测/故障安全)
 *                参数整定(见 pid_tuning.h:Z-N/Cohen-Coon/SIMC/4:1 衰减/继电反馈自整定)
 * @note     兼容旧接口 PIDInit() / PIDCalculate(),旧标志位取值不变
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#define PID_MEASURE_FILTER_MAX 16 /* 测量滤波最大窗口 */
#define PID_SMITH_MAX_DELAY 64    /* 史密斯预估最大滞后步数 */

/* PID 优化环节使能标志位,通过位与可以判断启用的优化环节 */
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                 // 0000 0000
    PID_Integral_Limit = 0b00000001,               // 0000 0001  积分限幅
    PID_Derivative_On_Measurement = 0b00000010,    // 0000 0010  微分先行
    PID_Trapezoid_Intergral = 0b00000100,          // 0000 0100  梯形积分
    PID_Proportional_On_Measurement = 0b00001000,  // 0000 1000  比例项只对测量值
    PID_OutputFilter = 0b00010000,                 // 0001 0000  输出一阶低通滤波
    PID_ChangingIntegrationRate = 0b00100000,      // 0010 0000  变速积分
    PID_DerivativeFilter = 0b01000000,             // 0100 0000  不完全微分(微分低通滤波)
    PID_ErrorHandle = 0b10000000,                  // 1000 0000  电机堵转检测

    PID_Integral_Separation = (1u << 8),           // 积分分离(带滞回)
    PID_Conditional_Integral = (1u << 9),          // 条件积分(饱和时冻结积分)
    PID_BackCalculation = (1u << 10),              // 反计算抗饱和
    PID_DerivativeLimit = (1u << 11),              // 微分增益限幅
    PID_FourPointDiff = (1u << 12),                // 四点差分(降噪微分)
    PID_MeasurementFilter = (1u << 13),            // 测量预处理滤波(均值/中值/LPF)
    PID_MeasureCheck = (1u << 14),                 // 测量坏值检测(保持/故障安全)
    PID_SlewRateLimit = (1u << 15),                // 输出变化率软限幅
    PID_Incremental = (1u << 16),                  // 增量式 PID
    PID_2DOF = (1u << 17),                         // 二自由度(设定值加权 b/c)
    PID_Feedforward = (1u << 18),                  // 前馈补偿(配合 PIDCalculateEx 或 Feedforward 字段)
    PID_SetpointRamp = (1u << 19),                 // 设定值斜坡
    PID_SetpointFilter = (1u << 20),               // 设定值一阶低通
    PID_Kahan = (1u << 21),                        // 积分 Kahan 补偿求和
    PID_SmithPredictor = (1u << 22),               // 史密斯预估器(由 PIDSmithCalculate 使用)
} PID_Improvement_e;

/* PID 报错类型枚举 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U, /* 电机堵转 */
    PID_MEASURE_FAULT_ERROR = 0x02U, /* 测量连续坏值,进入故障安全输出 */
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* 增益调度单点增益 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
} PID_Gain_s;

/* 增益调度表:按工况量 x 查表(线性插值)切换 Kp/Ki/Kd */
typedef struct
{
    const float *breakpoints; /* 升序断点 x[0..count-1] */
    const PID_Gain_s *gains;  /* 每个断点对应的增益 */
    uint8_t count;
    float Kp; /* 最近一次应用值(输出) */
    float Ki;
    float Kd;
} PID_GainSchedule_s;

/* PID 实例结构体 */
typedef struct
{
    /* ------------------------------ 配置块(持久参数) ------------------------------ */
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   /* 输出硬限幅 */
    float DeadBand; /* 死区 */

    uint32_t Improve; /* 优化使能标志位,位与判断 */

    /* 积分优化参数 */
    float IntegralLimit;            /* 积分限幅(配合 PID_Integral_Limit) */
    float CoefA;                    /* 变速积分 A:ITerm 缩放区上界 = A+B */
    float CoefB;                    /* 变速积分 B:全速积分区上界 |err|<=B */
    float IntegralSeparationEnter;  /* 积分分离进入阈值 |e|>Enter 暂停积分 */
    float IntegralSeparationExit;   /* 积分分离退出阈值 |e|<Exit 恢复积分(滞回,Exit<=Enter;0=与Enter相同) */
    float BackCalcGain;             /* 反计算抗饱和增益 Kb(0=自动:Ki>0取Ki,否则取Kp) */
    float IntegralPreload;          /* 积分先行:初始化时 Iout 预置值 */

    /* 微分优化参数 */
    float Derivative_LPF_RC; /* 微分滤波时间常数 RC=1/omegac */
    float DerivativeLimit;   /* 微分增益限幅 Dmax(>0 生效) */

    /* 结构优化参数 */
    float SetpointWeightP; /* 2-DOF 比例项设定值加权 b(通常 0~1) */
    float SetpointWeightD; /* 2-DOF 微分项设定值加权 c(通常 0~1) */

    /* 设定值处理参数 */
    float SetpointRampRate; /* 设定值斜坡速率(单位/s,>0 生效) */
    float SetpointFilterRC; /* 设定值低通滤波时间常数 */

    /* 输出处理参数 */
    float Output_LPF_RC; /* 输出滤波时间常数 */
    float SlewRateLimit; /* 输出变化率软限幅(单位/s,>0 生效) */

    /* 测量处理参数 */
    float MeasureMin;            /* 测量有效范围下限 */
    float MeasureMax;            /* 测量有效范围上限 */
    float MeasureJumpLimit;      /* 测量单拍最大跳变(>0 生效) */
    float FaultSafeOutput;       /* 测量连续坏值时的故障安全输出 */
    uint32_t MeasureErrorThreshold; /* 连续坏值计数阈值(0=默认10) */
    uint8_t MeasureFilterMode;   /* 测量滤波方式:0 关,1 移动平均,2 中值 */
    uint8_t MeasureFilterWindow; /* 测量滤波窗口(<=16) */
    float MeasureFilterRC;       /* 测量低通滤波时间常数(>0 生效) */

    /* 采样周期参数 */
    float DtMin; /* dt 钳位下限(0=不生效) */
    float DtMax; /* dt 钳位上限(0=不生效) */

    /* ------------------------------ 运行状态块 ------------------------------ */
    float Measure;             /* 当前有效测量(坏值处理后) */
    float Last_Measure;        /* 上一拍有效测量 */
    float Measure_Filtered;    /* 滤波后测量(用于控制律) */
    float Last_Measure_Filtered;
    float Prev2_Measure_Filtered;
    float Prev3_Measure_Filtered;
    float MeasureBuf[PID_MEASURE_FILTER_MAX]; /* 测量滤波环形缓冲 */
    uint8_t MeasureBufIdx;
    uint8_t MeasureBufCount;
    uint8_t MeasureReady;    /* 是否已收到首个有效测量 */
    uint8_t SeparationActive;/* 积分分离是否处于生效状态 */

    float Err;
    float Last_Err;
    float Prev2_Err;
    float Prev3_Err;

    float Pout;  /* 位置式:比例分量;增量式:比例增量 */
    float Iout;  /* 积分累积(增量式下仅作观测) */
    float Dout;  /* 位置式:微分分量;增量式:当前微分值 */
    float ITerm; /* 本拍积分增量 */
    float Last_ITerm; /* 上一拍积分增量(兼容旧字段) */
    float I_Comp;/* Kahan 补偿项 */

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;              /* 原始设定值(斜坡/滤波前) */
    float Ref_Filtered;     /* 斜坡/滤波后的设定值(用于控制律) */
    float Last_Ref_Filtered;
    float Prev2_Ref_Filtered;
    float Prev3_Ref_Filtered;

    float Feedforward;      /* 前馈量(配合 PID_Feedforward 或 PIDCalculateEx) */
    float Last_Feedforward;

    uint32_t MeasureErrorCount; /* 连续坏值计数 */
    uint32_t SatFlag;           /* 上一拍输出是否饱和 */

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于 PID 初始化的配置结构体(字段与 PIDInstance 配置块一一对应) */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    PID_Improvement_e Improve;

    float IntegralLimit;
    float CoefA;
    float CoefB;
    float IntegralSeparationEnter;
    float IntegralSeparationExit;
    float BackCalcGain;
    float IntegralPreload;

    float Derivative_LPF_RC;
    float DerivativeLimit;

    float SetpointWeightP;
    float SetpointWeightD;

    float SetpointRampRate;
    float SetpointFilterRC;

    float Output_LPF_RC;
    float SlewRateLimit;

    float MeasureMin;
    float MeasureMax;
    float MeasureJumpLimit;
    float FaultSafeOutput;
    uint32_t MeasureErrorThreshold;
    uint8_t MeasureFilterMode;
    uint8_t MeasureFilterWindow;
    float MeasureFilterRC;

    float DtMin;
    float DtMax;
} PID_Init_Config_s;

/* 史密斯预估器:对象模型采用一阶惯性+纯滞后 FOPDT:G(s)=K*e^(-Ls)/(1+Ts) */
typedef struct
{
    PIDInstance pid;          /* 内部 PID 实例 */
    float ModelGain;          /* 模型增益 K */
    float ModelTau;           /* 模型时间常数 T */
    float DelayTime;          /* 纯滞后 L(s) */
    float dt;                 /* 采样周期(s) */
    float y_model;            /* 无滞后模型输出 */
    float y_delayed;          /* 带滞后模型输出 */
    float y_pred;             /* 预估输出 = y_delayed + (y_meas - y_model) */
    float u_delay_buf[PID_SMITH_MAX_DELAY + 1]; /* 控制量延迟线 */
    uint16_t delay_steps;     /* 滞后步数 = round(L/dt) */
    uint16_t head;
} PID_SmithPredictor_s;

/* 串级 PID:外环(慢,输出作内环设定值)+ 内环(快) */
typedef struct
{
    PIDInstance outer;
    PIDInstance inner;
} PID_Cascade_s;

/**
 * @brief 初始化 PID 实例
 * @param pid    PID 实例指针
 * @param config PID 初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算 PID 输出(位置式/增量式由 PID_Incremental 标志决定)
 * @param pid     PID 实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID 计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

/**
 * @brief 带前馈的 PID 计算(无需置 PID_Feedforward 标志)
 * @param pid        PID 实例指针
 * @param measure    反馈值
 * @param ref        设定值
 * @param feedforward 前馈量
 * @return float     PID 计算输出
 */
float PIDCalculateEx(PIDInstance *pid, float measure, float ref, float feedforward);

/**
 * @brief 复位 PID 运行状态(保留配置;积分恢复为 IntegralPreload)
 */
void PIDReset(PIDInstance *pid);

/**
 * @brief 在线修改 PID 增益(积分项保持不变,无扰)
 */
void PIDSetGains(PIDInstance *pid, float kp, float ki, float kd);

/**
 * @brief 预置/修改积分项(积分先行/切换前预装载)
 */
void PIDSetIntegral(PIDInstance *pid, float value);

/**
 * @brief 无扰切换/跟踪模式:强制输出为指定值并反解积分项,
 *        用于手动<->自动切换、控制器切换时保持输出连续
 */
void PIDSetOutput(PIDInstance *pid, float value);

/**
 * @brief 立即设定设定值(跳过斜坡/滤波的过渡过程)
 */
void PIDSetSetpoint(PIDInstance *pid, float ref);

/**
 * @brief 增益调度:按工况量 x 查表(线性插值)更新 Kp/Ki/Kd
 * @param sched 调度表
 */
void PIDGainScheduleApply(PIDInstance *pid, PID_GainSchedule_s *sched, float x);

/**
 * @brief 串级 PID 初始化
 */
void PIDCascadeInit(PID_Cascade_s *c, PID_Init_Config_s *outer_cfg, PID_Init_Config_s *inner_cfg);

/**
 * @brief 串级 PID 计算
 * @param measure_outer 外环反馈
 * @param ref_outer     外环设定值
 * @param measure_inner 内环反馈
 * @return 内环输出
 */
float PIDCascadeCalculate(PID_Cascade_s *c, float measure_outer, float ref_outer, float measure_inner);

/**
 * @brief 史密斯预估器初始化
 * @param sp        预估器实例
 * @param pid_cfg   PID 配置
 * @param K         模型增益
 * @param T         模型时间常数
 * @param delay_time 纯滞后时间(s)
 * @param dt        采样周期(s)
 */
void PIDSmithInit(PID_SmithPredictor_s *sp, PID_Init_Config_s *pid_cfg, float K, float T,
                  float delay_time, float dt);

/**
 * @brief 史密斯预估 PID 计算:内部用预估输出替代测量值参与控制
 * @param measure 过程测量值(带滞后)
 * @param ref     设定值
 * @return 控制器输出
 */
float PIDSmithCalculate(PID_SmithPredictor_s *sp, float measure, float ref);

#endif

