#ifndef CONTROL_2026_WING_H
#define CONTROL_2026_WING_H

#include "bsp_pwm.h"
#include "robot_config.h"

//一个舵机实例
typedef struct {
    PWMInstance* pwm;
    float angle;        //当前角度, 调试回读用
    float left;         //命令 0°   对应的实际舵机角
    float zero;         //命令 90°  对应的实际舵机角(中位)
    float right;        //命令 180° 对应的实际舵机角
} servo_typedef;

//舵面混控实例。参考 func/wing.h 的 wing_typedef
typedef struct {
    servo_typedef servo[SERVO_CH_NUM];
    float alpha_yaw;
    float alpha_pitch;
    float alpha_roll;
    float angle[4];
    int dirs[4];
} wing_typedef;

/**
 * @brief 初始化混控: 注册 4 路舵机 PWM, 回中位, 并复位状态
 * @param l_z_r 每路的 {left, zero, right}, 来自 robot_config.h 的 SERVO_LZR_TABLE
 * @param dirs  舵面方向表 {1/-1, ...}, 来自 robot_config.h 的 WING_DIRS
 */
void WingInit(wing_typedef *wing, float l_z_r[4][3],int dirs[4]);

/**
 * @brief 由三个轴的攻角指令解算出 4 路舵面角度并输出
 * @param alpha_yaw   偏航攻角 (deg)
 * @param alpha_pitch 俯仰攻角 (deg)
 * @param alpha_roll  滚转攻角 (deg)
 */
void WingSetAlpha(wing_typedef *wing, float alpha_yaw, float alpha_pitch, float alpha_roll);

/**
 * @brief 直接给定 4 路舵面偏角(-45~45 deg), 会做限幅和方向映射
 */
void WingSetAngle(wing_typedef *wing, float *angle);

#endif // CONTROL_2026_WING_H
