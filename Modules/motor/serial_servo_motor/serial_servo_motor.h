#ifndef SERIAL_SERVO_MOTOR_H
#define SERIAL_SERVO_MOTOR_H

#include "bsp_usart.h"
#include "main.h"
#include "tim.h"
#include <stdint-gcc.h>
#include "bsp_usart.h"
#include "stdbool.h"

// 添加RS485方向控制引脚定义 (使用main.h中已定义的宏)
#define SERIAL_SERVO_RX_EN_Pin GPIO_PIN_2
#define SERIAL_SERVO_RX_EN_GPIO_Port SERIAL_SERVO_RX_EN_Pin_GPIO_Port
#define SERIAL_SERVO_TX_EN_Pin GPIO_PIN_3
#define SERIAL_SERVO_TX_EN_GPIO_Port SERIAL_SERVO_TX_EN_Pin_GPIO_Port

#define SERVO_MOTOR_CNT 7
#define Servo_Frame_First 0x55
#define Servo_Frame_Second 0x55
#define Servo_MAX_BUFF 10
#define SERVO_MOVE_CMD 0x03
#define SERVO_UNLOAD_CMD 0x14
#define SERVO_POS_READ_CMD 0x15

// 新增舵机命令定义
#define SERIAL_SERVO_MOVE_TIME_WRITE      1
#define SERIAL_SERVO_MOVE_TIME_READ       2
#define SERIAL_SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERIAL_SERVO_MOVE_TIME_WAIT_READ  8
#define SERIAL_SERVO_MOVE_START           11
#define SERIAL_SERVO_MOVE_STOP            12
#define SERIAL_SERVO_ID_WRITE             13
#define SERIAL_SERVO_ID_READ              14
#define SERIAL_SERVO_ANGLE_OFFSET_ADJUST  17
#define SERIAL_SERVO_ANGLE_OFFSET_WRITE   18
#define SERIAL_SERVO_ANGLE_OFFSET_READ    19
#define SERIAL_SERVO_ANGLE_LIMIT_WRITE    20
#define SERIAL_SERVO_ANGLE_LIMIT_READ     21
#define SERIAL_SERVO_VIN_LIMIT_WRITE      22
#define SERIAL_SERVO_VIN_LIMIT_READ       23
#define SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERIAL_SERVO_TEMP_MAX_LIMIT_READ  25
#define SERIAL_SERVO_TEMP_READ            26
#define SERIAL_SERVO_VIN_READ             27
#define SERIAL_SERVO_POS_READ             28
#define SERIAL_SERVO_OR_MOTOR_MODE_WRITE  29
#define SERIAL_SERVO_OR_MOTOR_MODE_READ   30
#define SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERIAL_SERVO_LOAD_OR_UNLOAD_READ  32
#define SERIAL_SERVO_LED_CTRL_WRITE       33
#define SERIAL_SERVO_LED_CTRL_READ        34
#define SERIAL_SERVO_LED_ERROR_WRITE      35
#define SERIAL_SERVO_LED_ERROR_READ       36

#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

typedef enum
{
  Servo_None_Type = 0,
  Bus_Servo = 1,
  // 删除PWM_Servo类型
}ServoType_e;

typedef enum {
    SERIAL_SERVO_RECV_STARTBYTE_1,
    SERIAL_SERVO_RECV_STARTBYTE_2,
    SERIAL_SERVO_RECV_SERVO_ID,
    SERIAL_SERVO_RECV_LENGTH,
    SERIAL_SERVO_RECV_COMMAND,
    SERIAL_SERVO_RECV_ARGUMENTS,
    SERIAL_SERVO_RECV_CHECKSUM,
} SerialServoRecvState;

/* 用于初始化不同舵机的结构体,各类舵机通用 */
typedef struct
{
  // 删除PWM_Init_Config_s pwm_init_config;
  ServoType_e servo_type;
  UART_HandleTypeDef *_handle;
  uint8_t servo_id;
}Servo_Init_Config_s;

#pragma pack(1)
typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    union {
        struct {
            uint8_t servo_id;
            uint8_t length;
            uint8_t command;
            uint8_t args[8];
        } elements;
        uint8_t data_raw[11];
    };
} SerialServoCmdTypeDef;
#pragma pack()

typedef struct
{
  uint8_t servo_id;
  float angle;
  uint16_t recv_angle;
  // 删除PWMInstance *pwm_instance;
  USARTInstance *usart_instance;
  ServoType_e servo_type;
  
  // 新增用于串行舵机的字段
  SerialServoRecvState rx_state;
  SerialServoCmdTypeDef rx_frame;
  uint32_t rx_args_index;
  SerialServoCmdTypeDef tx_frame;
  uint32_t tx_byte_index;
  bool tx_only;
  uint32_t proc_timeout;
}SerialServoInstance;

// 重命名函数，避免与servo_motor模块冲突
SerialServoInstance *SerialServoInit(Servo_Init_Config_s *Servo_Init_Config);
void SerialServoSetAngle(SerialServoInstance *servo, float angle);

// 新增函数声明
void SerialServoInitInternal(SerialServoInstance *servo);
void SerialServoResetIndex(void);
void SerialServoSetID(SerialServoInstance *servo, uint32_t old_id, uint32_t new_id);
int SerialServoReadID(SerialServoInstance *servo, uint32_t servo_id, uint8_t *ret_servo_id);
void SerialServoSetPosition(SerialServoInstance *servo, uint32_t servo_id, int position, uint32_t duration);
int SerialServoReadPosition(SerialServoInstance *servo, uint32_t servo_id, int16_t *position);
int SerialServoReadPositionEnhanced(SerialServoInstance *servo, uint32_t servo_id, int16_t *position);  // 添加增强版函数声明
void SerialServoStop(SerialServoInstance *servo, uint32_t servo_id);
void SerialServoSetDeviation(SerialServoInstance *servo, uint32_t servo_id, int new_deviation);
int SerialServoReadDeviation(SerialServoInstance *servo, uint32_t servo_id, int8_t *deviation);
void SerialServoSaveDeviation(SerialServoInstance *servo, uint32_t servo_id);
void SerialServoLoadUnload(SerialServoInstance *servo, uint32_t servo_id, uint32_t load);
void SerialServoSetAngleLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);
int SerialServoReadAngleLimit(SerialServoInstance *servo, uint32_t servo_id, uint16_t limit[2]);
void SerialServoSetTempLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit);
int SerialServoReadTempLimit(SerialServoInstance *servo, uint32_t servo_id, uint8_t *limit);
int SerialServoReadTemp(SerialServoInstance *servo, uint32_t servo_id, uint8_t *temp);
void SerialServoSetVinLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h);
int SerialServoReadVinLimit(SerialServoInstance *servo, uint32_t servo_id, uint16_t limit[2]);
int SerialServoReadVin(SerialServoInstance *servo, uint32_t servo_id, uint16_t *vin);
int SerialServoReadLoadUnload(SerialServoInstance *servo, uint32_t servo_id, uint8_t* load_unload);
int SerialServoSendTestCommand(SerialServoInstance *servo, uint32_t servo_id);
int SerialServoRequestAngle(SerialServoInstance *servo, uint32_t servo_id);

static inline uint8_t serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

int serial_servo_rx_handler(SerialServoInstance *servo, uint8_t rx_byte);

#endif // SERIAL_SERVO_MOTOR_H