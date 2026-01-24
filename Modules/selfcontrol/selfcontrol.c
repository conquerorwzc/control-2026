#include "selfcontrol.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include <stdbool.h>

#define SELF_CONTROL_FRAME_SIZE 21u // 遥控器接收的buffer大小
#define ROBOT_INTERACTIVE_DATA_CMD_ID 0x0302

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance* self_rc_usart_instance;
static SelfC self_control;
static Frame_Header referee_receive_header;

// 解析自定义控制器的数据包
bool parse_custom_controller_data(const uint8_t *packed_data, uint16_t packed_size, UnpackedControllerData_t *unpacked_data) {
    // 检查数据包基本结构
    if (packed_data == NULL || unpacked_data == NULL) {
        return false;
    }

    // 验证帧头 (0xA5)
    if (packed_data[0] != 0xA5) {
        return false;
    }

    // 提取数据长度 (第1-2字节)
    uint16_t data_len = ((uint16_t)packed_data[2] << 8) | packed_data[1];
    
    // 检查数据长度是否符合预期 (固定30字节数据 + 7字节头部 + 2字节CRC尾部)
    if (packed_size != (7 + data_len + 2)) {
        return false;
    }

    // 检查命令ID是否为自定义控制器 (CMD_ID_CUSTOM_CONTROLLER = 0x0302)
    uint16_t cmd_id = ((uint16_t)packed_data[6] << 8) | packed_data[5];
    if (cmd_id != 0x0302) {
        return false;
    }

    // 指向实际数据部分 (跳过协议头：帧头+长度+序号+CRC8+命令ID = 7字节)
    const uint8_t *data_ptr = &packed_data[7];

    // 数据包类型标识 (data_ptr[0] = data_ptr[7+0] = packed_data[7])
    if (data_ptr[0] != 0x20) {  // 控制器数据包标识
        return false;
    }

    // 解析3个舵机的数据
    for (int i = 0; i < 3; i++) {
        unpacked_data->servos[i].id = data_ptr[1 + i*5];  // 舵机ID
        
        // 解析角度值 (2字节，已乘以100存储)
        int16_t angle_raw = ((int16_t)data_ptr[3 + i*5] << 8) | data_ptr[2 + i*5];
        unpacked_data->servos[i].angle = (float)angle_raw / 100.0f;
        
        // 解析扭矩状态
        unpacked_data->servos[i].torque_status = data_ptr[4 + i*5];
        
        // 解析在线状态
        unpacked_data->servos[i].is_online = data_ptr[5 + i*5];
    }

    // 解析2个电位器的数据 (每个电位器占用5字节：ID + 2字节角度 + 2字节电压)
    for (int i = 0; i < 2; i++) {
        unpacked_data->pots[i].id = data_ptr[16 + i*5];  // 电位器ID
        
        // 解析角度值 (2字节，已乘以100存储)
        int16_t angle_raw = ((int16_t)data_ptr[18 + i*5] << 8) | data_ptr[17 + i*5];
        unpacked_data->pots[i].angle = (float)angle_raw / 100.0f;
        
        // 解析电压值 (2字节，已乘以100存储)
        int16_t voltage_raw = ((int16_t)data_ptr[20 + i*5] << 8) | data_ptr[19 + i*5];
        unpacked_data->pots[i].voltage = (float)voltage_raw / 100.0f;
    }

    return true;
}

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
      
      // 解析自定义控制器数据
      parse_custom_controller_data(frame, sizeof(Frame_Header) + sizeof(uint16_t) + sizeof(self_control.data), &self_control.unpacked_data);
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

