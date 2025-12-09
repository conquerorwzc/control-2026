#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include "servo_motor.h"

// 定义电位器相关常量
#define POTENTIOMETER_COUNT 2
#define POTENTIOMETER_ADC_HANDLE hadc1  // 根据实际使用的ADC修改
#define POTENTIOMETER_ADC_CHANNEL_1 ADC_CHANNEL_0  // 根据实际连接的ADC通道修改
#define POTENTIOMETER_ADC_CHANNEL_2 ADC_CHANNEL_1  // 根据实际连接的ADC通道修改

// 定义舵机相关常量
#define SERVO_MOTOR_COUNT 3

// 自定义控制器结构体
typedef struct {
    ServoInstance* servo_motors[SERVO_MOTOR_COUNT];// 舵机相关
    uint32_t potentiometer_values[POTENTIOMETER_COUNT];// 电位器相关
} CustomControllerInstance;

// 函数声明
CustomControllerInstance* CustomControllerInit(void);
void CustomControllerUpdate(CustomControllerInstance* controller_instance);

#endif // CUSTOM_CONTROLLER_H
