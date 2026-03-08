/**
******************************************************************************
* @file    robot.c
* @brief   自定义控制器机器人实现文件 - 调用components层的控制器组件
******************************************************************************
*/
#include "robot.h"
#include "robot_config.h"
#include "custom_controller.h"
#include "motor_task.h"
#include "bsp_usart.h"
#include <stdbool.h>
#include "bsp_gpio.h"

// 自定义控制器实例
static CustomController_t* angle_controller;
static GPIOInstance *gpio_24V_R_EN;
static GPIO_Init_Config_s gpio_init_config_24v = {
    .GPIO_Pin = POWER_24V_R_Pin,
    .GPIOx = POWER_5V_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
  };

static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
    .GPIO_Pin = POWER_5V_Pin,
    .GPIOx = POWER_24V_R_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
  };

// 微动开关 GPIO
static GPIOInstance *gpio_micro_switch;
static GPIO_Init_Config_s gpio_init_config_micro_switch = {
    .GPIO_Pin = Micro_switch_Pin,
    .GPIOx = Micro_switch_GPIO_Port,
    .pin_state = GPIO_PIN_RESET,
  };

// 调试用全局变量
volatile uint8_t debug_switch_state = 0;
static GPIO_PinState last_switch_state = GPIO_PIN_SET;  // 记录上一次状态

void RobotInit() {
    // 创建初始化配置结构体
    CustomController_Init_Config_s init_config = {
        .dm4310_config_1 = DM4310_config_1,
        .dm4310_config_2 = DM4310_config_2,
        .m3508_config_1 = M3508_config_1,
        .m3508_config_2 = M3508_config_2,
        .m2006_config = M2006_config
    };

    gpio_24V_R_EN = GPIORegister(&gpio_init_config_24v);
    GPIOSet(gpio_24V_R_EN);

    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);

    // 初始化微动开关 GPIO（输入模式）
    gpio_micro_switch = GPIORegister(&gpio_init_config_micro_switch);

    // 初始化自定义控制器（包含电机初始化）
    angle_controller = CustomControllerInit(&init_config);
    if (angle_controller == NULL) {
        // 错误处理可以根据需要添加
        return;

    }
}

void RobotTask() {
    if (angle_controller != NULL) {
        // 更新电机角度数据
        CustomControllerTask(angle_controller);
        
        // 发送所有电机数据通过 USART3
        CustomController_SendAllData(angle_controller);
    }
    
    // 简单读取微动开关电平
    GPIO_PinState switch_state = GPIORead(gpio_micro_switch);
    
    // 检测下降沿（高电平 -> 低电平）
    if (last_switch_state == GPIO_PIN_SET && switch_state == GPIO_PIN_RESET) {
        debug_switch_state = !debug_switch_state;  // 在 0 和 1 之间切换
    }
    
    // 更新上一次状态
    last_switch_state = switch_state;
}
