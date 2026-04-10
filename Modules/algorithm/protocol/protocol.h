#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "usart.h"
#include "crc.h"
#include "crc_func.h"
#include "string.h"

// 自定义控制器命令ID定义
#define CMD_ID_CUSTOM_CONTROLLER 0x0302

// void protocol_packed(uint8_t *data, uint16_t cmd_id, uint8_t data_len, uint8_t *tx_buff, uint16_t *tx_buff_length);

uint8_t *custom_controller_protocol_pack(uint16_t cmd_id, uint8_t *data, uint8_t data_len, uint16_t *packed_length);

#endif