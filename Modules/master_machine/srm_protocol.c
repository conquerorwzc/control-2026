#include "srm_protocol.h"

#include "memory.h"
#include "usbd_cdc_if.h"

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

  if (receive_size == 0) {
    memcpy(&receive_size, rx_buf + buf_pos, sizeof(short));
    buf_pos += sizeof(short);
    buffer_size = 0;
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
    memcpy(&id, ptr, sizeof(short));
    ptr += sizeof(short);
    if (id == -1) {
      receive_size = 0;
      return (USBD_OK);
    } else if (id >= 0 && id < 32) {
      if(receive->ptr_list[id] != NULL) {
        memcpy(receive->ptr_list[id], ptr, receive->size_list[id]);
        ptr += receive->size_list[id];
      }
    }
  }

  receive_size = 0;
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