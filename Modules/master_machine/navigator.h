//
// Created by ASUS on 2025/11/23.
//

#ifndef CONTROL_2026_NAVIGATOR_H
#define CONTROL_2026_NAVIGATOR_H

#include "usart.h"
#include "crc_func.h"
#include "string.h"

// void protocol_packed(uint8_t *data, uint16_t cmd_id, uint8_t data_len, uint8_t *tx_buff, uint16_t *tx_buff_length);

uint8_t *protocol_pack(uint32_t time_stamp, const uint8_t *data, uint8_t data_len, uint8_t data_id, uint16_t *packed_length);

HAL_StatusTypeDef protocol_send(UART_HandleTypeDef* huart, uint32_t time_stamp, const uint8_t* data_ptr, uint8_t data_len, uint8_t data_id, uint32_t timeout);


#endif  // CONTROL_2026_NAVIGATOR_H
