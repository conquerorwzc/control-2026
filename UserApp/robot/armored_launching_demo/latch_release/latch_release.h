//
// Created by CX330y on 2026/11/12.
//

#ifndef CONTROL_2026_LATCH_RELEASE_H
#define CONTROL_2026_LATCH_RELEASE_H

#pragma once

/**
 * @brief 注册 M2006 电机，使电机保持停机，并订阅动作指令。
 */
void LatchReleaseInit(void);

/**
 * @brief 首次收到有效反馈时记录零点，收到击发请求后驱动电机并在输出窗口结束时停机。
 */
void LatchReleaseTask(void);

#endif // CONTROL_2026_LATCH_RELEASE_H
