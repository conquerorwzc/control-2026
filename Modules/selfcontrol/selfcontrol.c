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
    if (packed_data == NULL || unpacked_data == NULL) return false;
    if (packed_data[0] != 0xA5) return false;

    // 检查命令ID (0x0302) 的位置：应为索引 5 和 6
    uint16_t cmd_id = ((uint16_t)packed_data[6] << 8) | packed_data[5];
    if (cmd_id != 0x0302) return false;

    // 指向实际数据部分 (跳过 A5 + Len(2) + Seq(1) + CRC8(1) + ID(2) = 7字节)
    const uint8_t *data_ptr = &packed_data[7];

    if (data_ptr[0] != 0x20) return false;

    // 舵机解析 (逻辑原本基本正确，保持 i*5)
    for (int i = 0; i < 3; i++) {
        unpacked_data->servos[i].id = data_ptr[1 + i*5];
        int16_t angle_raw = ((int16_t)data_ptr[3 + i*5] << 8) | data_ptr[2 + i*5];
        unpacked_data->servos[i].angle = (float)angle_raw / 100.0f;
        unpacked_data->servos[i].torque_status = data_ptr[4 + i*5];
        unpacked_data->servos[i].is_online = data_ptr[5 + i*5];
    }

    // 电位器解析 (关键点：修正 i*4 为 i*5)
    for (int i = 0; i < 2; i++) {
        // 每个电位器数据：ID(1) + Angle(2) + Volt(2) = 5字节
        unpacked_data->pots[i].id = data_ptr[16 + i*5];

        int16_t angle_raw = ((int16_t)data_ptr[18 + i*5] << 8) | data_ptr[17 + i*5];
        unpacked_data->pots[i].angle = (float)angle_raw / 100.0f;

        int16_t voltage_raw = ((int16_t)data_ptr[20 + i*5] << 8) | data_ptr[19 + i*5];
        unpacked_data->pots[i].voltage = (float)voltage_raw / 100.0f;
    }

    return true;
}

void selfcontrol_data_solve(uint8_t* frame) {
    if (frame[0] != 0xA5) return; // 增加基础校验

    uint16_t cmd_id = 0;
    // 修正：直接读取第5和第6字节（小端模式）
    cmd_id = (frame[6] << 8) | frame[5];

    switch (cmd_id) {
    case ROBOT_INTERACTIVE_DATA_CMD_ID:  // 0x0302
        // 总长度：7字节(头+ID) + 30字节(数据) + 2字节(校验) = 39
        parse_custom_controller_data(frame, 39, &self_control.unpacked_data);
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

