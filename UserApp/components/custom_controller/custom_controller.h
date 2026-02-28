/**
 ******************************************************************************
 * @file    custom_controller.h
 * @brief   自定义控制器头文件 - UserApp/components层
 ******************************************************************************
 */
#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <bsp_log.h>
#include "user_lib.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "protocol.h"
#include "crc_func.h"
#include "usart.h"
#include "bsp_usart.h"
#include "bsp_adc.h"  // 添加ADC头文件

/* ----------------------- 电位器配置结构体 ----------------------------- */
typedef struct {
    float min_voltage;      // 电位器最小电压值(V)
    float max_voltage;      // 电位器最大电压值(V)
    float min_angle;        // 对应的最小角度(度)
    float max_angle;        // 对应的最大角度(度)
    float filter_alpha;     // 滤波系数(0.0-1.0)
} Potentiometer_Config_s;

/* ----------------------- 电位器数据结构体 ----------------------------- */
typedef struct {
    ADCInstance* adc_instance;          // ADC实例
    float current_voltage;               // 当前电压值
    float current_angle;                // 当前角度值
    bool is_initialized;                // 初始化标志
    Potentiometer_Config_s config;      // 电位器配置参数（持久化存储）
} Potentiometer_Instance_s;

/* ----------------------- 电机单元结构体 ----------------------------- */
typedef struct {
    DMMotorInstance* dm_motor;              // DM电机实例
    DJIMotorInstance* dji_motor;            // DJI电机实例
} MotorUnit_t;
// @todo: 两个3508的ID最好改成2和3，防止于4310的ID对冲
/* ----------------------- 初始化配置结构体 ----------------------------- */
typedef struct {
    Motor_Init_Config_s dm4310_config;      // DM4310电机配置
    Motor_Init_Config_s m3508_config_1;     // 第一个3508电机配置
    Motor_Init_Config_s m3508_config_2;     // 第二个3508电机配置
    Motor_Init_Config_s m2006_config;       // 2006电机配置
    Potentiometer_Config_s pot_config;      // 电位器配置
} CustomController_Init_Config_s;

/* ----------------------- 电机数据结构体 ----------------------------- */
typedef struct {
    uint8_t id;                             // 电机ID
    int16_t present_pos;                    // 当前位置
    float current_angle;                    // 当前角度
    uint8_t is_online;                      // 在线状态
} MotorData_t;

/* ----------------------- 组件结构体 ----------------------------- */
typedef struct {
    MotorUnit_t motors[4];                  // 电机单元数组
    float motor_angles[4];                  // 各电机角度反馈
    float zero_offset[4];                   // 零位偏移值
    bool motor_online_status[4];            // 电机在线状态（用于检测断电重启）
    Potentiometer_Instance_s potentiometer; // 电位器实例
    MotorData_t motor_data[4];              // 电机数据（用于发送）
    USARTInstance* usart_instance;          // USART通信实例
    bool is_initialized;                    // 初始化标志
    bool is_active;                         // 活跃状态
} CustomController_t;

/* ----------------------- 函数声明 ----------------------------- */

/**
 * @brief 初始化自定义控制器
 * @param init_config 初始化配置
 * @return CustomController_t* 控制器实例指针
 */
CustomController_t* CustomControllerInit(CustomController_Init_Config_s* init_config);

/**
 * @brief 控制器主任务函数
 * @param controller 控制器实例
 */
void CustomControllerTask(CustomController_t* controller);

/**
 * @brief 获取指定电机角度
 * @param controller 控制器实例
 * @param motor_index 电机索引(0-3)
 * @return float 电机角度
 */
float CustomControllerGetMotorAngle(const CustomController_t* controller, 
                                   uint8_t motor_index);

/**
 * @brief 获取电位器角度
 * @param controller 控制器实例
 * @return float 电位器角度
 */
float CustomControllerGetPotAngle(const CustomController_t* controller);

/**
 * @brief 发送自定义控制器的所有数据
 * @param controller 控制器实例
 */
void CustomController_SendAllData(CustomController_t* controller);

/**
 * @brief 更新电机数据用于发送
 * @param controller 控制器实例
 */
void CustomController_UpdateMotorData(CustomController_t* controller);

/**
 * @brief 更新电位器数据
 * @param controller 控制器实例
 */
void CustomController_UpdatePotData(CustomController_t* controller);

#endif // CUSTOM_CONTROLLER_H