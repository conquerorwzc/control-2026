#include "selfcontrol.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define SELF_CONTROL_FRAME_SIZE 21u // 遥控器接收的buffer大小
#define ROBOT_INTERACTIVE_DATA_CMD_ID 0x0302

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance* self_rc_usart_instance;
static SelfC self_control;
static Frame_Header referee_receive_header;

void selfcontrol_data_solve(uint8_t* frame) {
  uint16_t cmd_id = 0;
  uint8_t index = 0;
  memcpy(&referee_receive_header, frame, sizeof(Frame_Header));
  index += sizeof(Frame_Header);
  index--;
  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id) {
    case ROBOT_INTERACTIVE_DATA_CMD_ID:  // 自定义控制器数据(0x0302)
      memcpy(&self_control.data, frame + index, sizeof(self_control.data));
      break;
    default:
      break;
  }
}

static void SelfControlRxCallback() {
  memcpy(&self_control.selfcontrol_buff, self_rc_usart_instance->recv_buff, SELF_CONTROL_FRAME_SIZE);
  selfcontrol_data_solve(self_control.selfcontrol_buff);
}


SelfC* SelfControlInit(UART_HandleTypeDef* usart_handle) {
  USART_Init_Config_s conf;
  conf.module_callback = SelfControlRxCallback;
  conf.usart_handle = usart_handle;
  conf.recv_buff_size = SELF_CONTROL_FRAME_SIZE;
  self_rc_usart_instance = USARTRegister(&conf);

  return &self_control;

}

