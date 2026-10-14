#include "observer_vofa.h"

#include <string.h>

static UART_HandleTypeDef *observer_uart;
static uint8_t tx_buffer[OBSERVER_VOFA_TX_BUFFER_SIZE];

HAL_StatusTypeDef ObserverVOFAInit(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    if (huart == NULL || huart->hdmatx == NULL || baudrate == 0u) return HAL_ERROR;
    if (observer_uart != NULL) return HAL_BUSY;

    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    const HAL_StatusTypeDef status = HAL_UART_Init(huart);
    if (status == HAL_OK) observer_uart = huart;
    return status;
}

HAL_StatusTypeDef ObserverVOFASendBytes(const uint8_t *data, uint16_t size)
{
    if (observer_uart == NULL || data == NULL || size == 0u || size > sizeof(tx_buffer)) return HAL_ERROR;
    if (observer_uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;

    memcpy(tx_buffer, data, size);
    return HAL_UART_Transmit_DMA(observer_uart, tx_buffer, size);
}

HAL_StatusTypeDef ObserverVOFAReceiveBytes(uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if (observer_uart == NULL || data == NULL || size == 0u) return HAL_ERROR;
    return HAL_UART_Receive(observer_uart, data, size, timeout_ms);
}
