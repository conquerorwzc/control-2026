//
// Created by ASUS on 2025/11/23.
//

#ifndef CONTROL_2026_NAVIGATOR_H
#define CONTROL_2026_NAVIGATOR_H

#include "usart.h"
#include "crc_func.h"
#include "string.h"

// void protocol_packed(uint8_t *data, uint16_t cmd_id, uint8_t data_len, uint8_t *tx_buff, uint16_t *tx_buff_length);

uint8_t *protocol_pack(uint16_t cmd_id, uint8_t *data, uint8_t data_len, uint16_t *packed_length);


#endif  // CONTROL_2026_NAVIGATOR_H
