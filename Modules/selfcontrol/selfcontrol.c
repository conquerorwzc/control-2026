#include "selfcontrol.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include <stdbool.h>

#define SELF_CONTROL_FRAME_SIZE 64u // 遥控器接收的buffer大小
#define ROBOT_INTERACTIVE_DATA_CMD_ID 0x0302

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance* self_rc_usart_instance;
static SelfC self_control;
static Frame_Header referee_receive_header;

// 解析自定义控制器的数据包
// 修改 selfcontrol.c 中的 parse_custom_controller_data 函数
bool parse_custom_controller_data(const uint8_t *packed_data, uint16_t packed_size, UnpackedControllerData_t *unpacked_data) {
    if (packed_data == NULL || unpacked_data == NULL) return false;
    if (packed_data[0] != 0xA5) return false;

    uint16_t cmd_id = ((uint16_t)packed_data[6] << 8) | packed_data[5];
    if (cmd_id != 0x0302) return false;

    const uint8_t *data_ptr = &packed_data[7]; // 指向 Data 段起始位置 (0x20 处)

    if (data_ptr[0] != 0x20) return false;

    // 1. 舵机解析 (逻辑保持 i*5)
    for (int i = 0; i < 3; i++) {
        unpacked_data->servos[i].id = data_ptr[1 + i*5];
        int16_t angle_raw = ((int16_t)data_ptr[3 + i*5] << 8) | data_ptr[2 + i*5];
        unpacked_data->servos[i].angle = (float)angle_raw / 100.0f;
        unpacked_data->servos[i].torque_status = data_ptr[4 + i*5];
        unpacked_data->servos[i].is_online = data_ptr[5 + i*5];
    }

    // 2. 电位器解析 (修正起始偏移为 16)
    for (int i = 0; i < 2; i++) {
        // 每个电位器数据：ID(1) + Angle(2) + Volt(2) = 5字节
        uint8_t pot_start = 16 + (i * 5); // 0x20后的第16字节开始是电位器

        unpacked_data->pots[i].id = data_ptr[pot_start];

        // 角度读取 (偏移 + 1 和 + 2)
        int16_t angle_raw = ((int16_t)data_ptr[pot_start + 2] << 8) | data_ptr[pot_start + 1];
        unpacked_data->pots[i].angle = (float)angle_raw / 100.0f;

        // 电压读取 (偏移 + 3 和 + 4)
        int16_t voltage_raw = ((int16_t)data_ptr[pot_start + 4] << 8) | data_ptr[pot_start + 3];
        unpacked_data->pots[i].voltage = (float)voltage_raw / 100.0f;
    }

    return true;
}

// 修改 selfcontrol.c 中的 selfcontrol_data_solve 函数
void selfcontrol_data_solve(uint8_t* frame) {
    if (frame[0] != 0xA5) return;

    // 从协议头提取 Data 域的实际长度
    uint16_t data_len = frame[1] | (frame[2] << 8);
    uint16_t total_frame_len = 7 + data_len + 2; // 头(7) + 数据 + CRC16(2)

    uint16_t cmd_id = (frame[6] << 8) | frame[5];

    switch (cmd_id) {
    case ROBOT_INTERACTIVE_DATA_CMD_ID:  // 0x0302
        // 传入计算出的实际总长度
        parse_custom_controller_data(frame, total_frame_len, &self_control.unpacked_data);
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

