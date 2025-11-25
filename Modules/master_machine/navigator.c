//
// Created by ASUS on 2025/11/23.
//
#include "navigator.h"
#include "crc_func.h"
// 帧头相关
#define PROTOCOL_SOF         0x5A
#define PROTOCOL_HEADER_LEN  4
#define PROTOCOL_CRC8_INIT   0xFF

// 帧尾相关
#define PROTOCOL_CRC16_INIT  0xFFFF

// 缓冲区最大尺寸
#define BUFFER_MAX_SIZE      128

static uint8_t internal_tx_buffer[BUFFER_MAX_SIZE];

static uint8_t* protocol_packed(const uint8_t* data_ptr, uint32_t time_stamp, uint8_t data_len, uint8_t data_id, uint8_t* tx_buff, uint16_t* tx_buff_len)
{
  // 1. 参数有效性检查
  if (tx_buff == NULL || tx_buff_len == NULL) {
    return NULL;
  }

  // 2. 计算总帧长并检查缓冲区是否足够
  uint16_t total_frame_len = PROTOCOL_HEADER_LEN + 4 + data_len + 2;
  if (total_frame_len > BUFFER_MAX_SIZE) {
    *tx_buff_len = 0;
    return NULL;
  }

  // 3. 填充帧头
  uint16_t current_index = 0;
  tx_buff[current_index++] = PROTOCOL_SOF;                  // sof
  tx_buff[current_index++] = data_len;                      // len
  tx_buff[current_index++] = data_id;                       // id
  tx_buff[current_index++] = get_CRC8_check_sum(&tx_buff[0], 3, PROTOCOL_CRC8_INIT); // crc

  // 4. 填充时间戳 (小端模式)
  tx_buff[current_index++] = (time_stamp >> 0)  & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 8)  & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 16) & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 24) & 0xFF;

  // 5. 填充数据段
  if (data_ptr != NULL && data_len > 0) {
    memcpy(&tx_buff[current_index], data_ptr, data_len);
    current_index += data_len;
  }

  // 6. 计算并填充帧尾CRC16
  uint16_t checksum_len = PROTOCOL_HEADER_LEN + 4 + data_len;
  uint16_t frame_crc16 = get_CRC16_check_sum(&tx_buff[0], checksum_len, PROTOCOL_CRC16_INIT);
  tx_buff[current_index++] = frame_crc16 & 0xFF;
  tx_buff[current_index++] = (frame_crc16 >> 8) & 0xFF;

  // 7. 设置最终帧长并返回
  *tx_buff_len = total_frame_len;
  return tx_buff;
}

uint8_t *protocol_pack(uint32_t time_stamp, const uint8_t *data, uint8_t data_len, uint8_t data_id, uint16_t *packed_length) {
  // 调用核心打包函数，使用静态的 internal_tx_buffer
  return protocol_packed(data, time_stamp, data_len, data_id, internal_tx_buffer, packed_length);
}

uint8_t protocol_send(UART_HandleTypeDef* huart, uint32_t time_stamp, const uint8_t* data_ptr, uint8_t data_len, uint8_t data_id, uint32_t timeout) {
  if (huart == NULL) {
    return 0;
  }

  uint16_t packed_length = 0;
  uint8_t local_buffer[BUFFER_MAX_SIZE];  // 使用局部缓冲区

  // 1. 打包到局部缓冲区
  uint8_t *packed_data = protocol_packed(data_ptr, time_stamp, data_len, data_id, local_buffer, &packed_length);

  // 2. 检查打包是否成功
  if (packed_data == NULL || packed_length == 0) {
    return 0;
  }

  // 3. 使用DMA传输局部缓冲区
  HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(huart, packed_data, packed_length);
  if (hal_status != HAL_OK) {
    return 0;
  }

  // 4. 等待DMA传输完成
  uint32_t start_tick = HAL_GetTick();
  while (huart->gState == HAL_UART_STATE_BUSY_TX) {
    if ((HAL_GetTick() - start_tick) > timeout) {
      HAL_UART_DMAStop(huart);
      return 0;
    }
    osDelay(1);
  }

  return 1;
}