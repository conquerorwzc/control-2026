/**
 ******************************************************************************
 * @file    chassis_lqr.h
 * @brief   双闭环轮腿十维 LQR 控制量计算接口
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/* MATLAB 与 MCU 固定一致的十维状态和四维输入数量。 */
#define WHEEL_LEGGED_LQR_STATE_COUNT 10u
#define WHEEL_LEGGED_LQR_INPUT_COUNT 4u

/* MATLAB 与 MCU 共用的十维状态槽位。 */
typedef enum
{
    WHEEL_LEGGED_LQR_STATE_S = 0,         /* 左右轮平均滚动纵向位移，单位 m。 */
    WHEEL_LEGGED_LQR_STATE_S_DOT,         /* 左右轮平均滚动纵向速度，单位 m/s。 */
    WHEEL_LEGGED_LQR_STATE_PHI,           /* 偏航角，单位 rad。 */
    WHEEL_LEGGED_LQR_STATE_PHI_DOT,       /* 偏航角速度，单位 rad/s。 */
    WHEEL_LEGGED_LQR_STATE_THETA_LEFT,    /* 左腿纵向平面角，单位 rad。 */
    WHEEL_LEGGED_LQR_STATE_THETA_LEFT_DOT, /* 左腿纵向平面角速度，单位 rad/s。 */
    WHEEL_LEGGED_LQR_STATE_THETA_RIGHT,   /* 右腿纵向平面角，单位 rad。 */
    WHEEL_LEGGED_LQR_STATE_THETA_RIGHT_DOT, /* 右腿纵向平面角速度，单位 rad/s。 */
    WHEEL_LEGGED_LQR_STATE_THETA_BODY,    /* 机身俯仰角，单位 rad。 */
    WHEEL_LEGGED_LQR_STATE_THETA_BODY_DOT, /* 机身俯仰角速度，单位 rad/s。 */
    WHEEL_LEGGED_LQR_STATE_COUNT_CHECK,   /* 状态总数，不代表实际状态。 */
} WheelLeggedChassisLqrState_e;

_Static_assert(WHEEL_LEGGED_LQR_STATE_COUNT_CHECK == WHEEL_LEGGED_LQR_STATE_COUNT,
               "LQR 状态必须保持十维");

/* MATLAB 导出与 MCU 发布的四个 LQR 输入槽位。 */
typedef enum
{
    WHEEL_LEGGED_LQR_INPUT_TP_RIGHT = 0, /* 右腿虚拟摆动广义力矩 Tp_R，单位 N*m。 */
    WHEEL_LEGGED_LQR_INPUT_TP_LEFT,      /* 左腿虚拟摆动广义力矩 Tp_L，单位 N*m。 */
    WHEEL_LEGGED_LQR_INPUT_TW_RIGHT,     /* 右轮轮端广义力矩 Tw_R，正值定义为右轮向前，单位 N*m。 */
    WHEEL_LEGGED_LQR_INPUT_TW_LEFT,      /* 左轮轮端广义力矩 Tw_L，正值定义为左轮向前，单位 N*m。 */
    WHEEL_LEGGED_LQR_INPUT_COUNT_CHECK,  /* 输入总数，不代表实际输入。 */
} WheelLeggedChassisLqrInput_e;

_Static_assert(WHEEL_LEGGED_LQR_INPUT_COUNT_CHECK == WHEEL_LEGGED_LQR_INPUT_COUNT,
               "LQR 输入必须保持四维");

/* 底盘对象中的 LQR 运行状态，供变量窗口与 MATLAB 同帧比较。 */
typedef struct
{
    uint8_t valid; /* 十维状态、零点、腿长数值和所有浮点计算均有效时为 1。 */

    float left_leg_length;  /* 本周期实测的左腿长度，仅供观察，单位 m。 */
    float right_leg_length; /* 本周期实测的右腿长度，仅供观察，单位 m。 */
    float state_reference[WHEEL_LEGGED_LQR_STATE_COUNT]; /* 固定 0.160 m 工作点的 x_ref。 */
    float state_error[WHEEL_LEGGED_LQR_STATE_COUNT];     /* x - x_ref。 */
    float gain[WHEEL_LEGGED_LQR_INPUT_COUNT][WHEEL_LEGGED_LQR_STATE_COUNT]; /* 当前 K。 */
    float trim_input[WHEEL_LEGGED_LQR_INPUT_COUNT];  /* 当前 u0。 */
    float input_sign[WHEEL_LEGGED_LQR_INPUT_COUNT];  /* MATLAB 已采用的输入符号，仅供核对。 */
    float output[WHEEL_LEGGED_LQR_INPUT_COUNT];      /* u0 - K*(x-x_ref)。 */

    float tp_right; /* output[Tp_R] 的具名调试别名，单位 N*m。 */
    float tp_left;  /* output[Tp_L] 的具名调试别名，单位 N*m。 */
    float tw_right; /* output[Tw_R] 的具名调试别名，单位 N*m。 */
    float tw_left;  /* output[Tw_L] 的具名调试别名，单位 N*m。 */
    uint32_t update_count; /* 已执行的 LQR 更新次数。 */
} WheelLeggedChassisLqr_t;

void WheelLeggedChassisLqrInit(WheelLeggedChassisLqr_t *lqr);
void WheelLeggedChassisLqrUpdate(WheelLeggedChassisLqr_t *lqr,
                                 const float state_vector[WHEEL_LEGGED_LQR_STATE_COUNT], uint8_t state_valid,
                                 uint8_t origin_captured, float left_leg_length, float right_leg_length);
