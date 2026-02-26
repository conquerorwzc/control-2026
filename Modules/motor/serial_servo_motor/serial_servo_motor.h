#ifndef SERIAL_SERVO_MOTOR_H
#define SERIAL_SERVO_MOTOR_H

#include "main.h"
#include "bsp_usart.h"
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* --- 协议常量定义 --- */
#define SERVO_HEADER_TX      0xFFFF  // 主机下发字头
#define SERVO_HEADER_RX      0xFFF5  // 舵机应答字头
#define SERVO_BROADCAST_ID   0xFE    // 广播ID

/* --- 指令类型 --- */
#define INST_PING            0x01
#define INST_READ            0x02
#define INST_WRITE           0x03
#define INST_REG_WRITE       0x04
#define INST_ACTION          0x05
#define INST_SYNC_WRITE      0x83

/* --- 寄存器地址表 --- */
#define ADDR_ID              0x05
#define ADDR_TORQUE_ENABLE   0x28
#define ADDR_GOAL_POSITION   0x2A  // 2字节
#define ADDR_GOAL_TIME       0x2C  // 2字节
#define ADDR_PRESENT_POS     0x38  // 2字节
#define ADDR_PRESENT_VOLT    0x21  // 示例: 协议文档5-7页
#define ADDR_PRESENT_TEMP    0x0D

// 定义舵机数量常量
#define SERVO_MOTOR_COUNT 3

/* --- 状态机 --- */
typedef enum {
  SS_WAIT_H1, SS_WAIT_H2, SS_WAIT_ID, SS_WAIT_LEN,
  SS_WAIT_STATUS, SS_WAIT_PARAMS, SS_WAIT_CHECK
} ServoState_e;

// 完整的结构体定义向前声明
typedef struct SerialServoInstance {
  uint8_t id;
  USARTInstance *usart_instance;  // 使用 BSP 层的 USART 实例

  // 实时数据
  uint16_t present_pos;
  uint8_t  present_temp;
  uint8_t  last_status;     // 舵机状态错误码
  int16_t  current_angle;   // 当前角度
  float    actual_angle;    // 实际角度值（经过转换）

  // 接收解析
  ServoState_e rx_state;
  uint8_t rx_buf[16];
  uint8_t rx_idx;
  uint8_t rx_len;
  uint8_t rx_chksum;
  uint8_t tmp_byte;
} SerialServo_t;

/* --- API --- */
void Servo_Init(SerialServo_t *servo, uint8_t id, USARTInstance *usart_instance);
void Servo_ReceiveHandler(SerialServo_t *servo); // 放进 UART RX 中断

// 控制指令
void Servo_SetPosition(SerialServo_t *servo, uint16_t pos, uint16_t time_ms);
void Servo_SetTorque(SerialServo_t *servo, bool enable);
void Servo_Ping(SerialServo_t *servo);

// 读取指令 (异步)
void Servo_ReadPosition(SerialServo_t *servo);

// 初始化函数
SerialServo_t* SerialServoInit(void* config);

// 舵机索引重置函数
void SerialServoResetIndex(void);

#endif