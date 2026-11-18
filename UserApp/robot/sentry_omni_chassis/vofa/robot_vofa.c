#include "robot_vofa.h"

#include <string.h>

static UART_HandleTypeDef *robot_uart;
static uint8_t tx_buffer[ROBOT_VOFA_TX_BUFFER_SIZE];

HAL_StatusTypeDef RobotVOFAInit(UART_HandleTypeDef *huart, uint32_t baudrate)
{
    if (huart == NULL || huart->hdmatx == NULL || baudrate == 0u) return HAL_ERROR;
    if (robot_uart != NULL) return HAL_BUSY;

    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    const HAL_StatusTypeDef status = HAL_UART_Init(huart);
    if (status == HAL_OK) robot_uart = huart;
    return status;
}

HAL_StatusTypeDef RobotVOFASendBytes(const uint8_t *data, uint16_t size)
{
    if (robot_uart == NULL || data == NULL || size == 0u || size > sizeof(tx_buffer)) return HAL_ERROR;
    if (robot_uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;

    memcpy(tx_buffer, data, size);
    return HAL_UART_Transmit_DMA(robot_uart, tx_buffer, size);
}

HAL_StatusTypeDef RobotVOFAReceiveBytes(uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if (robot_uart == NULL || data == NULL || size == 0u) return HAL_ERROR;
    return HAL_UART_Receive(robot_uart, data, size, timeout_ms);
}
