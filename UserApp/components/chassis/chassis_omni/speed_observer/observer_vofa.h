#pragma once

#include "bsp_usart.h"

#define OBSERVER_VOFA_DEFAULT_BAUDRATE 460800u
#define OBSERVER_VOFA_TX_BUFFER_SIZE 256u

// 基础传输接口，当前没有应用层调用；不依赖底盘实例，不定义通道或组包协议。
// 使用独占串口（原接口为 USART6），引脚/DMA 由硬件层配置，不可同时注册裁判/其他协议。
// 在开始收发前调用一次；8N1，无流控。baudrate 可传 OBSERVER_VOFA_DEFAULT_BAUDRATE。
HAL_StatusTypeDef ObserverVOFAInit(UART_HandleTypeDef *huart, uint32_t baudrate);

// 单任务调用：复制原始字节后 DMA 发送；忙时返回 HAL_BUSY，不覆盖在途缓冲区。
// 不自动添加 JustFloat 帧尾；HAL_OK 仅表示启动发送成功。
HAL_StatusTypeDef ObserverVOFASendBytes(const uint8_t *data, uint16_t size);

// 按指定长度轮询接收原始字节，可能阻塞至 timeout_ms；仅供非实时任务显式调用。
// 超时可能已写入部分字节，只有 HAL_OK 表示收到完整 size 字节；不自动解析或回复。
HAL_StatusTypeDef ObserverVOFAReceiveBytes(uint8_t *data, uint16_t size, uint32_t timeout_ms);
