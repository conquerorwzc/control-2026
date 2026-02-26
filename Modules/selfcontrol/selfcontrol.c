#include "selfcontrol.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_log.h"
#include <stdbool.h>
#include <math.h>

#define SELF_CONTROL_FRAME_SIZE 64u // 接收缓冲区大小
#define ROBOT_INTERACTIVE_DATA_CMD_ID 0x0302

// 控制器实例
static SelfC self_control;
// 全局USART实例指针
static USARTInstance* g_selfcontrol_usart = NULL;

// 角度标准化函数(下位机已实现，此处留空)
float SelfControlNormalizeAngle(float angle) {
    return angle;  // 直接返回原始角度
}

// 获取电机角度
float SelfControlGetMotorAngle(const SelfC* controller, uint8_t motor_index) {
    if (controller == NULL || motor_index >= 4) {
        return 0.0f;
    }
    return SelfControlNormalizeAngle(controller->unpacked_data.motors[motor_index].angle);
}

// 获取电位器角度
float SelfControlGetPotAngle(const SelfC* controller, uint8_t pot_index) {
    if (controller == NULL || pot_index >= 1) {
        return 0.0f;
    }
    return SelfControlNormalizeAngle(controller->unpacked_data.pots[pot_index].angle);
}

UnpackedControllerData_t* GetSelfControlDataPtr(void) {
    return &self_control.unpacked_data;
}

// 解析自定义控制器数据包
static bool parse_custom_controller_data(const uint8_t *packed_data, uint16_t packed_size, UnpackedControllerData_t *unpacked_data) {
    if (packed_data == NULL || unpacked_data == NULL) return false;
    if (packed_data[0] != 0xA5) return false;

    uint16_t cmd_id = ((uint16_t)packed_data[6] << 8) | packed_data[5];
    if (cmd_id != 0x0302) return false;

    const uint8_t *data_ptr = &packed_data[7]; // 指向 Data 段起始位置

    if (data_ptr[0] != 0x20) return false;

    // 解析电机数据 (4个电机)
    for (int i = 0; i < 4; i++) {
        unpacked_data->motors[i].id = data_ptr[1 + i*5];
        int16_t angle_raw = ((int16_t)data_ptr[3 + i*5] << 8) | data_ptr[2 + i*5];
        unpacked_data->motors[i].angle = (float)angle_raw / 100.0f;
        unpacked_data->motors[i].is_online = data_ptr[5 + i*5];
        // 扭矩状态字段已移除，保留为预留字节
    }

    // 解析电位器数据 (1个电位器)
    uint8_t pot_start = 20; // 电位器数据起始位置
    
    unpacked_data->pots[0].id = data_ptr[pot_start];
    
    // 解析角度
    int16_t angle_raw = ((int16_t)data_ptr[pot_start + 2] << 8) | data_ptr[pot_start + 1];
    unpacked_data->pots[0].angle = (float)angle_raw / 100.0f;
    
    // 解析电压
    int16_t voltage_raw = ((int16_t)data_ptr[pot_start + 4] << 8) | data_ptr[pot_start + 3];
    unpacked_data->pots[0].voltage = (float)voltage_raw / 100.0f;

    return true;
}

// 数据解析函数(保持原有可用逻辑)
void selfcontrol_data_solve(uint8_t* frame) {
    uint16_t cmd_id = 0;
    uint8_t index = 0;
    static Frame_Header referee_receive_header;  // 保持原有变量
    
    memcpy(&referee_receive_header, frame, sizeof(Frame_Header));
    index += sizeof(Frame_Header);
    index--;
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case ROBOT_INTERACTIVE_DATA_CMD_ID:  // 自定义控制器数据(0x0302)
            // 使用原有解析逻辑，适配新数据结构
            parse_custom_controller_data(frame, sizeof(Frame_Header) + sizeof(uint16_t) + sizeof(self_control.unpacked_data), &self_control.unpacked_data);
            break;
        default:
            break;
    }
}

// USART接收回调函数(保持原有逻辑)
static void SelfControlRxCallback() {
    static USARTInstance* self_rc_usart_instance = NULL;  // 保持原有变量名
    if (self_rc_usart_instance == NULL) {
        extern USARTInstance* g_selfcontrol_usart;
        self_rc_usart_instance = g_selfcontrol_usart;
    }
    
    if (self_rc_usart_instance != NULL) {
        memcpy(&self_control.selfcontrol_buff, self_rc_usart_instance->recv_buff, SELF_CONTROL_FRAME_SIZE);
        selfcontrol_data_solve(self_control.selfcontrol_buff);
    }
}

// 初始化函数(保持原有逻辑)
SelfC* SelfControlInit(UART_HandleTypeDef* usart_handle) {
    USART_Init_Config_s conf;
    conf.module_callback = SelfControlRxCallback;
    conf.usart_handle = usart_handle;
    conf.recv_buff_size = SELF_CONTROL_FRAME_SIZE;
    
    extern USARTInstance* g_selfcontrol_usart;
    g_selfcontrol_usart = USARTRegister(&conf);

    return &self_control;
}