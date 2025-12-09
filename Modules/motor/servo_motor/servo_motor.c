#include "servo_motor.h"
#include "stdlib.h"
#include "memory.h"
#include "bsp_log.h"
#include "stdio.h"
#include "string.h"
#include "bsp_usart.h"

/*第二版*/
static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static uint8_t servo_idx = 0; // register servo_idx,是该文件的全局舵机索引,在注册时使用
static void DecodeServo();
static USARTInstance* shared_usart_instance = NULL; // 共享的USART实例

// 通过此函数注册一个舵机
ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config)
{
    // 检查是否超过最大舵机数量
    if (servo_idx >= SERVO_MOTOR_CNT) {
        LOGERROR("Servo motor count exceeds maximum limit");
        return NULL;
    }

    ServoInstance *servo = (ServoInstance *)malloc(sizeof(ServoInstance));
    if (servo == NULL) {
        LOGERROR("Failed to allocate memory for servo instance");
        return NULL;
    }
    
    memset(servo, 0, sizeof(ServoInstance));
    servo->servo_type = Servo_Init_Config->servo_type;
    
    switch (Servo_Init_Config->servo_type)
    {
    case Bus_Servo:
        // 如果共享USART实例还未创建，则创建它
        if (shared_usart_instance == NULL) {
            USART_Init_Config_s config;
            config.module_callback = DecodeServo;
            config.recv_buff_size = Servo_MAX_BUFF;
            config.usart_handle = Servo_Init_Config->_handle;
            shared_usart_instance = USARTRegister(&config);
            if (shared_usart_instance == NULL) {
                LOGERROR("Failed to register USART for servo");
                free(servo);
                return NULL;
            }
        }
        // 所有舵机共享同一个USART实例
        servo->usart_instance = shared_usart_instance;
        break;
    case PWM_Servo:
        servo->pwm_instance = PWMRegister(&Servo_Init_Config->pwm_init_config);
        if (servo->pwm_instance == NULL) {
            LOGERROR("Failed to register PWM for servo %d", Servo_Init_Config->servo_id);
            free(servo);
            return NULL;
        }
        break;
    default:
        LOGERROR("Servo type error");
        free(servo);
        return NULL;
    }
    
    servo->servo_id = Servo_Init_Config->servo_id;
    servo_motor_instance[servo_idx] = servo;
    servo_idx++;

    return servo;
}

//@todo PWM舵机的角度设置需要根据相应定时器PWM等参数进行计算(是否需要规范定时器PWM的初始化参数，以便于计算)
void ServoSetAngle(ServoInstance *servo, float angle)
{

    switch (servo->servo_type)
    {
    case Bus_Servo:
        {
            char cmd[20];
            // 将角度(0-270度)映射到脉宽(500-2500us)
            uint16_t pulse = (uint16_t)(500 + angle * 2000 / 270);
            snprintf(cmd, sizeof(cmd), "#%03dP%04dT1000!\r\n", servo->servo_id, pulse);
            USARTSend(servo->usart_instance, (uint8_t*)cmd, strlen(cmd), USART_TRANSFER_DMA);
        }
        break;
    case PWM_Servo:
        servo->angle = angle;
        PWMSetDutyRatio(servo->pwm_instance, angle);
        break;
    default:
        break;
    }
}

// 设置Bus舵机为释力模式（可以手动摆动）
void Bus_Servo_Unload(ServoInstance *servo) {
    if (servo->servo_type != Bus_Servo) {
        return;
    }
    
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "#%03dPULK!\r\n", servo->servo_id);  // 释力命令
    USARTSend(servo->usart_instance, (uint8_t*)cmd, strlen(cmd), USART_TRANSFER_DMA);
}

// 获取Bus舵机角度（发送端）
void Bus_Servo_GetAngle(ServoInstance *servo) {
    if (servo->servo_type != Bus_Servo) {
        return;
    }
    
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "#%03dPRAD!\r\n", servo->servo_id);  // 读取角度命令
    USARTSend(servo->usart_instance, (uint8_t*)cmd, strlen(cmd), USART_TRANSFER_DMA);
}

// 设置Bus舵机硬件ID
void Bus_Servo_SetID(ServoInstance *servo, uint8_t new_id) {
    if (servo->servo_type != Bus_Servo) {
        return;
    }
    
    char cmd[15];
    snprintf(cmd, sizeof(cmd), "#%03dPID%03d!\r\n", servo->servo_id, new_id);  // 设置ID命令
    USARTSend(servo->usart_instance, (uint8_t*)cmd, strlen(cmd), USART_TRANSFER_DMA);
    
    // 更新软件层面的ID
    servo->servo_id = new_id;
}

// 解析Bus舵机返回的角度值
float Bus_Servo_ParseAngle(ServoInstance *servo) {
    if (servo->servo_type != Bus_Servo) {
        return 0.0f;
    }
    
    // 舵机返回格式: "#000P1500!" 其中000是舵机ID，1500是脉宽
    
    uint8_t *buffer = servo->usart_instance->recv_buff;
    float angle = 0.0f;
    
    // 查找以#开头，以!结尾的数据包
    char *start = strchr((char*)buffer, '#');
    if (start != NULL) {
        char *end = strchr(start, '!');
        if (end != NULL && end > start) {
            // 确保存在足够长的数据
            if (end - start >= 9) { // 最少 #000P1500! 格式
                // 提取ID部分
                char id_str[4] = {0};
                memcpy(id_str, start + 1, 3);
                uint8_t received_id = atoi(id_str);
                
                // 检查ID是否匹配
                if (received_id == servo->servo_id) {
                    // 查找P字符
                    char *p_pos = strchr(start + 4, 'P');
                    if (p_pos != NULL && p_pos < end) {
                        // 提取脉宽值
                        char pulse_str[5] = {0};
                        int pulse_len = end - p_pos - 1;
                        if (pulse_len <= 4) {
                            memcpy(pulse_str, p_pos + 1, pulse_len);
                            int pulse = atoi(pulse_str);
                            // 将脉宽(500-2500)转换为角度(0-270度)
                            if (pulse < 500) pulse = 500;
                            if (pulse > 2500) pulse = 2500;
                            angle = (pulse - 500.0f) * 270.0f / 2000.0f;
                        }
                    }
                }
            }
        }
    }
    return angle;
}

//@todo 只读取了角度 还有电压，动作是否完成等 且只支持一个串口
static void DecodeServo()
{
    for (uint8_t i = 0; i < servo_idx; i++)
    {
        if (servo_motor_instance[i]->servo_type == Bus_Servo)
        {
            float angle = Bus_Servo_ParseAngle(servo_motor_instance[i]);
            servo_motor_instance[i]->recv_angle = (uint16_t)(angle * 10); // 存储为整数形式(放大10倍)
        }
    }
}