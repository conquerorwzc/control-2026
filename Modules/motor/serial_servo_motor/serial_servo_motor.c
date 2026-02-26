#include "serial_servo_motor.h"
#include "bsp_usart.h"
#include "bsp_log.h"
#include <string.h>
#include "stdlib.h"
#include "memory.h"

// 根据项目规范添加全局变量
static SerialServo_t *servo_motor_instance[SERVO_MOTOR_COUNT];
static uint8_t servo_idx = 0;
static USARTInstance *registered_usart_instance = NULL;

/**
 * @brief 计算校验和: ~(ID + Len + Cmd + Params) & 0xFF
 */
static uint8_t CheckSum(uint8_t id, uint8_t len, uint8_t cmd, uint8_t *params, uint8_t p_len) {
    uint32_t sum = id + len + cmd;
    for(uint8_t i = 0; i < p_len; i++) sum += params[i];
    return (uint8_t)(~sum & 0xFF);
}

/**
 * @brief 底层发送指令包
 */
static void Servo_SendInst(SerialServo_t *servo, uint8_t inst, uint8_t *params, uint8_t p_len) {
    uint8_t tx_buf[32];
    uint8_t len = p_len + 2; // Data Length = N + 2

    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = servo->id;
    tx_buf[3] = len;
    tx_buf[4] = inst;
    if(p_len > 0) memcpy(&tx_buf[5], params, p_len);
    tx_buf[5 + p_len] = CheckSum(servo->id, len, inst, params, p_len);

    // 使用 BSP 层 USART 发送函数，采用阻塞模式
    USARTSend(servo->usart_instance, tx_buf, p_len + 6, USART_TRANSFER_BLOCKING);
    
    // 清除由于全双工引起的回显数据
    // 这对于阻塞模式特别重要，因为我们希望确保在下次接收前清除任何回显
    memset(servo->usart_instance->recv_buff, 0, sizeof(servo->usart_instance->recv_buff));
    
    // 添加短暂延迟，确保发送完成并处理回显
    osDelay(10);
}

/**
 * @brief 舵机接收回调函数，由BSP层调用
 */
static void Servo_USART_Callback(void) {
    // 由于BSP层的回调机制，我们需要遍历所有舵机实例找到对应的实例
    for (int i = 0; i < servo_idx; i++) {
        if (servo_motor_instance[i] && servo_motor_instance[i]->usart_instance) {
            // 处理接收数据
            Servo_ReceiveHandler(servo_motor_instance[i]);
        }
    }
}

/* --- 公开控制接口 --- */

void Servo_Init(SerialServo_t *servo, uint8_t id, USARTInstance *usart_instance) {
    memset(servo, 0, sizeof(SerialServo_t));
    servo->id = id;
    servo->usart_instance = usart_instance;
    servo->rx_state = SS_WAIT_H1;
}

// 添加缺失的SerialServoInit函数
SerialServo_t* SerialServoInit(void* config) {
    // 检查数组边界
    if (servo_idx >= SERVO_MOTOR_COUNT) {
        return NULL; // 超过最大舵机数量
    }
    
    // 创建一个新的实例
    SerialServo_t* servo = (SerialServo_t*)malloc(sizeof(SerialServo_t));
    if (servo != NULL) {
        memset(servo, 0, sizeof(SerialServo_t));
        // 将新实例添加到数组中
        servo_motor_instance[servo_idx++] = servo;
    }
    return servo;
}

// 设置位置和时间
void Servo_SetPosition(SerialServo_t *servo, uint16_t pos, uint16_t time_ms) {
    uint8_t p[5];
    p[0] = ADDR_GOAL_POSITION;
    p[1] = (pos >> 8) & 0xFF;     // 大端模式: 高位在前
    p[2] = pos & 0xFF;
    p[3] = (time_ms >> 8) & 0xFF;
    p[4] = time_ms & 0xFF;
    Servo_SendInst(servo, INST_WRITE, p, 5);
}

// 读取位置请求
void Servo_ReadPosition(SerialServo_t *servo) {
    uint8_t p[2] = {ADDR_PRESENT_POS, 0x02}; // 地址0x38, 读2字节
    Servo_SendInst(servo, INST_READ, p, 2);
    
    // 添加适当的延迟，确保舵机有足够时间响应
    osDelay(10);
}

// 舵机扭矩控制
void Servo_SetTorque(SerialServo_t *servo, bool enable) {
    uint8_t p[2];
    p[0] = ADDR_TORQUE_ENABLE;
    p[1] = enable ? 1 : 0;
    Servo_SendInst(servo, INST_WRITE, p, 2);
}

// 舵机索引重置函数
void SerialServoResetIndex(void) {
    // 释放已分配的内存
    for(int i=0; i<servo_idx; i++) {
        if(servo_motor_instance[i]) {
            free(servo_motor_instance[i]);
            servo_motor_instance[i] = NULL;
        }
    }
    
    // 重置servo_idx计数器
    servo_idx = 0;
    
    // 重置registered_usart_instance指针
    registered_usart_instance = NULL;
}

/* --- 接收状态机 (核心逻辑) --- */

void Servo_ReceiveHandler(SerialServo_t *servo) {
    // 从 USART 实例获取接收到的数据
    // 注意：在阻塞模式下，我们需要逐字节处理数据
    // 这里我们仍然使用recv_buff[0]，但需要确保缓冲区已被正确填充
    
    // 由于使用阻塞模式，我们直接处理接收到的数据
    uint8_t data = servo->usart_instance->recv_buff[0];

    switch (servo->rx_state) {
        case SS_WAIT_H1:
            if (data == 0xFF) servo->rx_state = SS_WAIT_H2;
            break;
        case SS_WAIT_H2:
            if (data == 0xF5) servo->rx_state = SS_WAIT_ID; // 舵机应答字头
            else servo->rx_state = SS_WAIT_H1;
            break;
        case SS_WAIT_ID:
            // 根据JOHO协议，校验和应包含 ID + Len + Status + Params
            servo->rx_chksum = data; // 初始化校验和为ID值
            if (data == servo->id || data == SERVO_BROADCAST_ID) {
                servo->rx_state = SS_WAIT_LEN;
            } else {
                // ID不匹配，但仍继续解析，避免因回显或其他舵机数据导致状态机混乱
                servo->rx_state = SS_WAIT_LEN;
            }
            break;
        case SS_WAIT_LEN:
            servo->rx_len = data;
            servo->rx_chksum += data;
            servo->rx_state = SS_WAIT_STATUS;
            break;
        case SS_WAIT_STATUS:
            servo->last_status = data;
            servo->rx_chksum += data;
            servo->rx_idx = 0; // 在此处重置索引是合理的
            if (servo->rx_len > 2) servo->rx_state = SS_WAIT_PARAMS;
            else servo->rx_state = SS_WAIT_CHECK;
            break;
        case SS_WAIT_PARAMS:
            servo->rx_buf[servo->rx_idx++] = data;
            servo->rx_chksum += data;
            if (servo->rx_idx >= (servo->rx_len - 2)) servo->rx_state = SS_WAIT_CHECK;
            break;
        case SS_WAIT_CHECK:
            if (((uint8_t)~servo->rx_chksum) == data) { // 校验通过
                // 添加调试日志
                LOGINFO("CheckSum OK, Len:%d, Data:%02X %02X", servo->rx_len, servo->rx_buf[0], servo->rx_buf[1]);
                
                // 增强的数据解析逻辑
                switch(servo->rx_len - 2) {
                    case 2: // 位置、速度等双字节数据
                        servo->present_pos = (servo->rx_buf[0] << 8) | servo->rx_buf[1];
                        // 同时更新current_angle字段
                        servo->current_angle = servo->present_pos;
                        // 计算实际角度值 (4096 对应 270度)
                        servo->actual_angle = (float)servo->present_pos * 270.0f / 4096.0f;
                        LOGINFO("Servo ID:%d Position:%d Angle:%.2f", servo->id, servo->present_pos, servo->actual_angle);
                        break;
                    case 1: // 温度、ID等单字节数据
                        servo->present_temp = servo->rx_buf[0];
                        break;
                }
            } else {
                LOGINFO("Checksum Error: Expected:%02X, Actual:%02X", (uint8_t)(~servo->rx_chksum), data);
            }
            // 无论校验成功与否都重置状态机
            servo->rx_state = SS_WAIT_H1;
            break;
    }
}