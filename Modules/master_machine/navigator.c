//
// Created by ASUS on 2025/11/23.
//
#include "navigator.h"

#define BUFFER_MAX_SIZE 128

#define START_CODE 0XA5

static uint8_t internal_tx_buffer[BUFFER_MAX_SIZE];

static uint8_t custom_data[] = {0x40, 0x50, 0x60, 0x70};

void protocol_packed(uint8_t *data, uint32_t time_stamp, uint8_t data_len, uint8_t data_id, uint8_t *tx_buff, uint16_t *tx_buff_length) {


  if (tx_buff == NULL || tx_buff_length == NULL) {
    return;
  }

  // 帧结构: 帧头(4) + 时间戳(4) + 数据段 + CRC16(2)
  uint16_t total_len = 4 + 4 + data_len + 2;

  if (total_len > BUFFER_MAX_SIZE) {
    *tx_buff_length = 0; // 缓冲区不足，返回长度为0
    return;
  }

  tx_buff[0] = 0xA5;              // sof
  tx_buff[1] = data_len;              // len (数据段长度)
  tx_buff[2] = data_id;               // id (数据段ID)


  uint8_t crc8 = get_CRC8_check_sum(&tx_buff[0], 3, 0xFF);
  tx_buff[3] = crc8;                  // crc


  tx_buff[4] = (time_stamp >> 0)  & 0xFF;
  tx_buff[5] = (time_stamp >> 8)  & 0xFF;
  tx_buff[6] = (time_stamp >> 16) & 0xFF;
  tx_buff[7] = (time_stamp >> 24) & 0xFF;

  if (data != NULL && data_len > 0) {
    memcpy(&tx_buff[8], data, data_len); // 数据从索引8开始
  }

  uint16_t crc16 = get_CRC16_check_sum(&tx_buff[0], total_len - 2, 0xFFFF);

  // 将CRC16按小端序存入缓冲区的末尾
  tx_buff[total_len - 2] = crc16 & 0xFF;
  tx_buff[total_len - 1] = (crc16 >> 8) & 0xFF;

  // 6. 设置返回的长度
  *tx_buff_length = total_len;

  return;
}

uint8_t *protocol_pack(uint32_t time_stamp, uint8_t *data, uint8_t data_len, uint8_t data_id, uint16_t *packed_length) {
  // 调用核心打包函数，使用静态的 internal_tx_buffer
  protocol_packed(data, time_stamp, data_len, data_id, internal_tx_buffer, packed_length);

  // 返回静态缓冲区的地址
  return internal_tx_buffer;
}