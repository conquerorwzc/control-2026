/**
 * @file    ident_proto.h
 * @brief   J-Link VCOM 采集协议。**单向**：MCU 只发，PC 只收。
 *
 * 为什么单向：辨识只需要把 (q, dq, tau_cmd) 记下来，上位机离线拟合。
 * 下行命令（改零位/改增益）会逼 MCU 多一个 RX 状态机和一个解析任务，
 * 换来的只是省一次重新烧写——不值得。零位靠上电对齐 + 离线算偏移量，
 * 见 docs/zero-point.md。
 *
 * 传输层：J-Link 虚拟串口。**这不是 USB CDC，是目标板上一路真实 UART
 *         （TX/RX 引脚）桥接到 PC**，所以：
 *           - 波特率两头必须一致（J-Link 设置里和 huartX.Init.BaudRate）；
 *           - 目标是 8N1。注意 control2026 的 UART5 是 9 位字长（SBUS 用），
 *             **不能直接拿来传二进制**，用 USART1（已是 8N1）；
 *           - 发送必须用 USART_TRANSFER_IT 或 USART_TRANSFER_DMA。bsp_usart.c 的
 *             USART_TRANSFER_BLOCKING 走 HAL_UART_Transmit(...,100)，100ms 超时，
 *             挂在 500 Hz 控制任务里会直接卡死。
 *           - 带宽：100 Hz × 40 B = 40000 bit/s（含起止位）。
 *             115200 占用 34.7%，单帧 3.47 ms / 帧间隔 10 ms —— 够但没余量；
 *             460800 以上更稳。
 *
 * 帧格式（小端）：DATA 帧共 40 字节
 *
 *   off  size  field
 *   0    2     sync   = 0x55 0xAA
 *   2    1     type   = 0x01 DATA / 0x02 STATUS
 *   3    1     seq    0..255 循环，上位机用它统计丢帧
 *   4    2     len    载荷字节数（DATA 固定 28）
 *   6    4     t_ms   float32，DWT 毫秒时间戳
 *   10   28    payload: 7 × float32
 *   38   2     crc16  CCITT-FALSE，覆盖 off=2..37
 *
 * 控制周期 500 Hz，采集降频到 **100 Hz** 发送（decim=5）：
 *   100 × 40 B = 4 KB/s，J-Link VCOM 毫无压力。
 * 不要按 500 Hz 发；也不要在这条链路上做任何阻塞等待。
 */
#ifndef IDENT_PROTO_H
#define IDENT_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define IDENT_SYNC0        0x55u
#define IDENT_SYNC1        0xAAu
#define IDENT_TYPE_DATA    0x01u
#define IDENT_TYPE_STATUS  0x02u

#define IDENT_PAYLOAD_BYTES 28u
#define IDENT_FRAME_BYTES   40u

#pragma pack(push, 1)

/* 7 通道载荷。字段顺序必须与上位机 ident/serial.py 完全一致。 */
typedef struct {
  float q1;        /* rad  —— **已减零位**，契约角（0=水平，+ 抬起） */
  float q2;        /* rad  —— 已减零位 */
  float dq1;       /* rad/s —— 用 measure.velocity，不要自己差分 */
  float dq2;       /* rad/s */
  float tau1_cmd;  /* N*m  —— **我们下发的指令力矩**，不是读回的 torque */
  float tau2_cmd;  /* N*m */
  float t_ms;      /* ms   —— DWT */
} IdentSample;

typedef struct {
  float    q1_zero;
  float    q2_zero;
  float    q1_sign;
  float    q2_sign;
  uint8_t  logging;
  uint16_t decim;
  uint32_t dropped;   /* 发送失败计数，>0 说明链路跟不上 */
  uint32_t total;
} IdentStatus;

#pragma pack(pop)

uint16_t Ident_Crc16(const uint8_t *data, uint16_t len);

/** 组 DATA 帧，返回 40。out 需 >= 40 字节。 */
uint16_t Ident_BuildData(uint8_t *out, uint8_t seq, const IdentSample *s);

/** 组 STATUS 帧，返回 40（载荷复用，便于上位机核对零位/符号配置）。 */
uint16_t Ident_BuildStatus(uint8_t *out, uint8_t seq, const IdentStatus *st);

#endif /* IDENT_PROTO_H */
