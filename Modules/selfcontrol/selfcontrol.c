#include "selfcontrol.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define SELF_CONTROL_FRAME_SIZE 21u // 遥控器接收的buffer大小


// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance* self_rc_usart_instance;
static SelfC self_control;

static void SelfControlRxCallback() {
  memcpy(&self_control.selfcontrol_buff, self_rc_usart_instance->recv_buff, sizeof(SELF_CONTROL_FRAME_SIZE));
}


SelfC* SelfControlInit(UART_HandleTypeDef* usart_handle) {
  USART_Init_Config_s conf;
  conf.module_callback = SelfControlRxCallback;
  conf.usart_handle = usart_handle;
  conf.recv_buff_size = SELF_CONTROL_FRAME_SIZE;
  self_rc_usart_instance = USARTRegister(&conf);

  return &self_control;

}

