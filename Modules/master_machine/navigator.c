//
// Created by ASUS on 2025/11/23.
//
#include "navigator.h"

#define BUFFER_MAX_SIZE 128

#define START_CODE 0XA5

static uint8_t internal_tx_buffer[BUFFER_MAX_SIZE];

static uint8_t custom_data[] = {0x40, 0x50, 0x60, 0x70};

void protocol_packed(uint8_t *data,uint16_t cmd_id,uint8_t data_len,uint8_t *tx_buff,uint16_t *tx_buff_length){

  if (tx_buff == NULL || tx_buff_length == NULL)
  {
    return;
  }

  const uint8_t fixed_data_len = 30;
  uint16_t total_len = 7 + fixed_data_len + 2;
  if(total_len>BUFFER_MAX_SIZE){
    return;
  }
  static uint16_t seq = 0x08; // 包序号，不知道是什么东西啦
  uint16_t crc16 = 0;
  //帧头
  tx_buff[0] = START_CODE;
  tx_buff[1] = fixed_data_len & 0xFF;
  tx_buff[2] = (fixed_data_len >> 8) & 0xFF;
  tx_buff[3] = seq;
  tx_buff[4] = get_CRC8_check_sum(&tx_buff[0], 4,0xFF);
  //命令ID(自定义控制器0X0302)
  tx_buff[5] = cmd_id & 0XFF;
  tx_buff[6] = (cmd_id >> 8) & 0xFF;
  //数据部分
  if (data != NULL && data_len > 0)
  {
    // 复制实际数据
    uint8_t copy_len = (data_len < fixed_data_len) ? data_len : fixed_data_len;
    memcpy(&tx_buff[7], data, copy_len);

    // 填充剩余部分为0
    if (copy_len < fixed_data_len)
    {
      memset(&tx_buff[7 + copy_len], 0, fixed_data_len - copy_len);
    }
  }
  //帧尾CRC16
  crc16 = get_CRC16_check_sum(&tx_buff[0], fixed_data_len + 7,0xFFFF);
  tx_buff[fixed_data_len + 7] = crc16 & 0xFF;
  tx_buff[fixed_data_len + 8] = (crc16 >> 8) & 0xFF;
  *tx_buff_length = total_len;

  return;
}

uint8_t *protocol_pack(uint16_t cmd_id, uint8_t *data, uint8_t data_len, uint16_t *packed_length)
{
  protocol_packed(data, cmd_id, data_len, internal_tx_buffer, packed_length);
  return internal_tx_buffer;
}