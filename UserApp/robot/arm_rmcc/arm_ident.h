/**
 * @file    arm_ident.h
 * @brief   场地臂辨识采集库（STM32H7 / FreeRTOS / control2026），经 J-Link VCOM 回传。
 *
 * 传输路径：USART -> J-Link 的 UART 引脚 -> PC 上的 /dev/cu.usbmodem*。
 *   **不是 STM32 自己的 USB 外设**（那是 bsp_usb/CDC_Transmit_HS，和 J-Link 无关）。
 *
 * 设计约束：
 *   - 零动态分配（H7 工程 heap 只剩约 5 KB，且 DMMotorTask 每台电机占 128 words 栈）；
 *   - 不新建控制任务（采集挂已有 MotorControlTask，发送挂已有 RobotTask）；
 *   - 发送用 IT 或 DMA，**绝不用 BLOCKING**（bsp_usart.c 的 BLOCKING 是
 *     HAL_UART_Transmit(...,100)，100 ms 超时，挂 500 Hz 任务里会直接卡死）。
 *
 * 角度约定：**电机报文给多少就用多少**。补偿律在零位平移下形式不变，辨识会把
 * 偏移吸收进系数（实测偏移 0~90° 力矩误差均为 1e-15）。所以 q_zero 不是必需输入，
 * 默认 0；只有想让系数正好等于物理静矩 m*c 时才需要填。
 * 必须由你确认的是**方向**：q1_sign/q2_sign —— 正角度是抬起还是压下连杆。
 */
#ifndef ARM_IDENT_H
#define ARM_IDENT_H

#include <stdint.h>
#include <stdbool.h>

#include "bsp_usart.h"

/**
 * @brief 初始化。在 USARTRegister 之后、第一次 Task 之前调用一次。
 * @param usart  已注册并 USARTServiceInit 的串口实例（走 J-Link 那一路）
 * @param q1_sign +1 / -1：电机报文角增大是否**抬起**连杆 1
 * @param q2_sign 同上，连杆 2
 *
 * 方向自检：把连杆摆在水平附近，手推它**向上**，看 measure.total_angle
 * 是增大（+1）还是减小（-1）。搞反的后果不是小误差，是整条方程反号，
 * 机械臂会跑到镜像位姿。
 */
void ArmIdent_Init(USARTInstance *usart, float q1_sign, float q2_sign);

/** 可选：设置零位偏移，仅为了让打印出的系数等于物理静矩 m*c。
 *  不调用也行（默认 0），补偿精度完全不受影响。 */
void ArmIdent_SetZero(float q1_zero, float q2_zero);

/** 在 500 Hz 控制任务里调用。q/dq 传电机反馈原值，tau 传**你下发的**力矩。 */
void ArmIdent_Feed(float q1_raw, float q2_raw, float dq1, float dq2,
                   float tau1_cmd, float tau2_cmd);

/** 在低频任务里轮询（建议 RobotTask，每 2 ms 一次）把缓冲推给串口。 */
void ArmIdent_Poll(void);

/** 采集降频比：1 = 500 Hz, 5 = 100 Hz（默认）。范围 1..50。 */
void ArmIdent_SetDecim(uint16_t decim);

/** 运行期开关，默认开启。 */
void ArmIdent_Enable(bool on);

/** 发送就绪谓词。参数是 USARTInstance.usart_handle。
 *  返回 true 表示可以发下一帧。
 *
 *  为什么需要它：bsp_usart 不暴露 TX 完成回调，而本模块不能改 bsp。
 *  上层拿到 handle 自己查 HAL 状态即可：
 *      static bool uart_ready(void *h) {
 *        UART_HandleTypeDef *u = (UART_HandleTypeDef *)h;
 *        return u->gState == HAL_UART_STATE_READY &&
 *               __HAL_UART_GET_FLAG(u, UART_FLAG_TC);
 *      }
 *  不设置则按"总是就绪"处理（靠 100 Hz 发送节奏兜底，见 ArmIdent_Poll）。 */
typedef bool (*ArmIdent_TxReadyFn)(void *usart_handle);
void ArmIdent_SetTxReadyFn(ArmIdent_TxReadyFn fn);

/** 统计：环满丢弃的帧数、已发帧数、串口忙导致的发送失败数。 */
void ArmIdent_Stats(uint32_t *ring_dropped, uint32_t *sent);

#endif /* ARM_IDENT_H */
