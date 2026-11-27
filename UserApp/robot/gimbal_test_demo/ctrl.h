//
// Created by 20211 on 2026/11/27.
//

#ifndef GIMBAL_TEST_DEMO_CTRL_H
#define GIMBAL_TEST_DEMO_CTRL_H

#include "robot.h"

/**
 * @brief 输入模式 (由右拨杆决定)
 */
typedef enum {
    CTRL_MODE_NONE = 0,  // [下] 失能
    CTRL_MODE_STICK,     // [上] 摇杆模式
    CTRL_MODE_KEYMOUSE,  // [中] 键鼠模式
  } Ctrl_Mode_e;

/* ---------------- 对外接口 ---------------- */

/** @brief 缓存指令指针并初始化内部状态, 必须在 GimbalInit/ShootInit 之后调用 */
void CtrlInit(RobotInstance *robot);

/** @brief 主入口: 选模式 -> 控制 -> 急停保护 (每周期调用, 使用 robot->dt) */
void CtrlSolve(RobotInstance *robot);

/* -------- 分层实现, 便于单独调试 -------- */

/** @brief 遥控器摇杆控云台 (右拨杆[上], 发射强制关闭) */
void JoyStickCtrl(RobotInstance *robot);

/** @brief 键鼠控云台 + 左键发射 (右拨杆[中]) */
void MouseKeyCtrl(RobotInstance *robot);

/** @brief 急停与失能保护 (E/R、掉线、拨杆[下]) */
void EmergencyHandler(RobotInstance *robot);

/* ---------------- 调试观察接口 ---------------- */

Ctrl_Mode_e CtrlGetMode(void);
uint8_t CtrlIsMotorEnabled(void);

#endif  // GIMBAL_TEST_DEMO_CTRL_H

