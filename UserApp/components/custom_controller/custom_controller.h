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
#include "bsp_usart.h"
#include "bsp_gpio.h"

/* ----------------------- 电机单元结构体 ----------------------------- */
typedef struct {
    DMMotorInstance* dm_motor;              // DM电机实例
    DJIMotorInstance* dji_motor;            // DJI电机实例
} MotorUnit_t;
// @todo: 两个3508的ID最好改成2和3，防止于4310的ID对冲
/* ----------------------- 初始化配置结构体 ----------------------------- */
typedef struct {
    Motor_Init_Config_s dm4310_config_1;    // 第一个 DM4310 电机配置
    Motor_Init_Config_s dm4310_config_2;    // 第二个 DM4310 电机配置
    Motor_Init_Config_s m3508_config_1;     // 第一个 3508 电机配置
    Motor_Init_Config_s m3508_config_2;     // 第二个 3508 电机配置
    Motor_Init_Config_s m2006_config;       // 2006 电机配置
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
    MotorUnit_t motors[5];                  // 电机单元数组（5 个电机：2 个 4310、2 个 3508、1 个 2006）
    float motor_angles[5];                  // 各电机角度反馈
    float zero_offset[5];                   // 零位偏移值
    bool motor_online_status[5];            // 电机在线状态（用于检测断电重启）
    MotorData_t motor_data[5];              // 电机数据（用于发送）
    USARTInstance* usart_instance;          // USART 通信实例
    GPIOInstance* micro_switch_gpio;        // 微动开关 GPIO 实例
    bool gripper_opened;                    // 夹爪打开标志
    bool is_initialized;                    // 初始化标志
    bool is_active;                         // 活跃状态
    
    // 接收机器人发送的机械臂数据
    float robot_arm_angles[5];              // 机器人发送的5个关节角度
    uint8_t robot_grab_mode;                // 机器人机械臂控制模式 (0=键盘, 1=自定义控制器, 2=半自动)
    uint32_t last_robot_data_time;          // 上次接收时间戳
    bool robot_data_valid;                  // 数据有效性标志
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
 * @param motor_index 电机索引 (0-4)
 * @return float 电机角度
 */
float CustomControllerGetMotorAngle(const CustomController_t* controller, 
                                   uint8_t motor_index);

/**
 * @brief 更新电机数据用于发送
 * @param controller 控制器实例
 */
void CustomController_UpdateMotorData(CustomController_t* controller);

/**
 * @brief 发送自定义控制器的所有数据
 * @param controller 控制器实例
 */
void CustomController_SendAllData(CustomController_t* controller);

/**
 * @brief 获取机器人发送的指定关节角度
 * @param controller 控制器实例
 * @param joint_index 关节索引 (0-4)
 * @return float 关节角度(单位:度)，数据无效时返回0
 */
float CustomController_GetRobotArmAngle(const CustomController_t* controller, uint8_t joint_index);

#endif // CUSTOM_CONTROLLER_H