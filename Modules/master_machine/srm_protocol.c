#include "srm_protocol.h"

#include "../../Hardware/stm32-h7/USB_DEVICE/App/usbd_cdc_if.h"
#include "memory.h"

int vision_data_ready_to_send;
short receive_size;
short send_size;
short send_id_list[32];
short send_id_num;
char buffer[256];
short buffer_size;


uint16_t get_srm_protocol_info(uint8_t *rx_buf, Message *receive)
{
  short buf_pos = 0, id = 0;
  char *ptr;

  if (receive == NULL || rx_buf == NULL) {
    return USBD_OK;
  }

  if (receive_size == 0) {
    memcpy(&receive_size, rx_buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size = 0;
  }

  if (receive_size < 0 || receive_size > (short)sizeof(buffer)) {
    receive_size = 0;
    buffer_size = 0;
    return USBD_OK;
  }

  short remain_size = receive_size - buffer_size;
  if (remain_size > 0) {
    if (remain_size > 64 - buf_pos) {
      memcpy(buffer + buffer_size, rx_buf + buf_pos, 64 - buf_pos);
      buffer_size += 64 - buf_pos;
    } else {
      memcpy(buffer + buffer_size, rx_buf + buf_pos, remain_size);
      buffer_size += remain_size;
    }
  }
  if (receive_size != buffer_size) {
    return (USBD_OK);
  }

  ptr = buffer;
  while (ptr < buffer + buffer_size) {
    if (ptr + sizeof(short) > buffer + buffer_size) {
      break;
    }

    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);

    if (id == -1) {
      receive_size = 0;
      buffer_size = 0;
      return (USBD_OK);
    }

    if (id < 0 || id >= 32) {
      // 非法id会导致后续长度无法解析，直接丢弃本包避免卡死
      break;
    }

    short payload_size = receive->size_list[id];
    if (payload_size < 0 || ptr + payload_size > buffer + buffer_size) {
      break;
    }

    if (receive->ptr_list[id] != NULL) {
      memcpy(receive->ptr_list[id], ptr, payload_size);
    }
    // 无论是否订阅该id，都必须跳过对应负载，避免死循环
    ptr += payload_size;
  }

  receive_size = 0;
  buffer_size = 0;
  return (USBD_OK);
}

uint16_t srm_protocol_pack_send_data(Message *send, uint8_t *tx_buffer, uint16_t *tx_len)
{
  short send_size = 0;
  short id = 0;
  char *ptr = (char*)tx_buffer;

  // 计算总发送数据大小
  for(int i = 0; i < 32; i++) {
    if(send->ptr_list[i] != NULL) {
      send_size += send->size_list[i] + sizeof(short);
    }
  }

  // 打包数据
  memcpy(ptr, &send_size, sizeof(short));
  ptr += sizeof(short);

  for(int i = 0; i < 32; i++) {
    if(send->ptr_list[i] != NULL) {
      id = i;
      memcpy(ptr, &id, sizeof(short));
      ptr += sizeof(short);
      memcpy(ptr, send->ptr_list[i], send->size_list[i]);
      ptr += send->size_list[i];
    }
  }

  *tx_len = sizeof(short) + send_size;
  return USBD_OK;
}