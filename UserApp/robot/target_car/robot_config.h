/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   robot配置模块实现文件，用于集中管理机器人配置与参数
 ******************************************************************************
 */
#pragma once

#include "dji_motor.h"

/* ===================== 6020(GM6020) 电机: CAN1 / 拨码ID = 4 ===================== */
#define GM6020_CAN_HANDLE (&hcan1)  // 挂载总线: CAN1
#define GM6020_ID 4                 // 6020 拨码开关 ID(=4); 反馈ID由dji_motor自动算为0x204+4=0x208, 控制帧为0x1FF

/* ===================== 遥控器右侧拨杆对应的目标角速度(单位 rad/s) ===================== */
#define TARGET_SPEED_MID_RAD_S 3.0f   //
#define TARGET_SPEED_HIGH_RAD_S 6.5f  //

/**
 * @brief 6020 电机初始化配置: 只使用速度环
 *
 * @note  单位说明(重要):
 *        dji_motor 中速度环的反馈是 measure.speed_aps, 单位是 度/秒(deg/s),
 *        所以速度环参考值必须用 度/秒 下发, 而不是 rad/s;
 *        速度环 PID 的输出是电机的电压(力矩)指令, GM6020 的范围为 ±30000。
 */
static Motor_Init_Config_s GM6020_config = {
    .controller_param_init_config =
        {
            // @note 6020 的转速反馈分辨率只有 1rpm(=6deg/s), 0.4rad/s 仅 3.8rpm, 属于超低速工况;
            //       Kp 稍大就会把"反馈跳变"放大成来回震荡, 所以从很小的增益起调, 别照抄云台的大Kp。
            .speed_PID = {.Kp = 3.0f,    // 速度环: 输入deg/s, 输出±30000; 调参顺序: 先Kp=0再慢慢加, 最后才加Ki
                          .Ki = 0.1f,   // 积分用于消除静差; 若出现低频来回摆/嗡嗡抖, 先把Ki降到0
                          .Kd = 0.0f,
                          .IntegralLimit = 15000.0f,
                          .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                          .MaxOut = 30000.0f},
        },
    .controller_setting_init_config =
        {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
    .motor_type = GM6020,
    .can_init_config =
        {
            .can_handle = GM6020_CAN_HANDLE,
            .tx_id = GM6020_ID,
        },
};

/* ===================== 左拨杆中档: 正弦变速"小陀螺" =====================
 * ω(t) = (MAX+MIN)/2 + (MAX-MIN)/2 * sin(2π t / T)
 *        = 8.5 + 6.5 * sin(π t)   [rad/s]
 * 即: 最小 2 rad/s, 最大 15 rad/s, 周期 2s。
 */
#define SPIN_SPEED_MIN_RAD_S 2.0f    // 正弦波谷, 单位 rad/s
#define SPIN_SPEED_MAX_RAD_S 10.0f   // 正弦波峰, 单位 rad/s
#define SPIN_PERIOD_S 1.7f           // 周期, 单位 s
