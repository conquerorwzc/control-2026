#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include "serial_servo_motor.h"
#include "bsp_log.h"
#include "bsp_adc.h"

// 定义电位器相关常量
#define POTENTIOMETER_COUNT 2
#define POTENTIOMETER_ADC_HANDLE1 hadc1  // 根据实际使用的ADC修改
#define POTENTIOMETER_ADC_HANDLE2 hadc2  // 根据实际使用的ADC修改
#define POTENTIOMETER_ADC_CHANNEL_1 ADC_CHANNEL_16  // ADC1通道16
#define POTENTIOMETER_ADC_CHANNEL_2 ADC_CHANNEL_14  // ADC2通道14

// 定义舵机相关常量
#define SERVO_MOTOR_COUNT 3

// 电位器结构体
typedef struct {
    ADCInstance* adc[POTENTIOMETER_COUNT];    // 电位器ADC实例
    uint32_t values[POTENTIOMETER_COUNT];     // 电位器原始值
    float angles[POTENTIOMETER_COUNT];        // 电位器角度值 (0-270度)
} PotentiometerInstance;

// 自定义控制器结构体
typedef struct {
    SerialServoInstance* servo_motors[SERVO_MOTOR_COUNT];  // 舵机相关
    PotentiometerInstance potentiometer;             // 电位器相关
} CustomControllerInstance;

// 函数声明
CustomControllerInstance* CustomControllerInit(void);
void CustomControllerUpdate(CustomControllerInstance* controller_instance);

#endif // CUSTOM_CONTROLLER_H