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
static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
    .GPIO_Pin = POWER_5V_Pin,
    .GPIOx = POWER_5V_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
  };

void RobotInit() {
    // 创建初始化配置结构体，包含电位器配置
    CustomController_Init_Config_s init_config = {
        .dm4310_config = DM4310_config,
        .m3508_config_1 = M3508_config_1,
        .m3508_config_2 = M3508_config_2,
        .m2006_config = M2006_config,
        .pot_config = POT_config  // 添加电位器配置
    };

    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);

    // 初始化自定义控制器（包含电机初始化和电位器初始化）
    angle_controller = CustomControllerInit(&init_config);
    if (angle_controller == NULL) {
        // 错误处理可以根据需要添加
        return;

    }
}

void RobotTask() {
    if (angle_controller != NULL) {
        // 更新电机角度数据和电位器数据
        CustomControllerTask(angle_controller);
        
        // 发送所有电机数据和电位器数据通过USART3
        CustomController_SendAllData(angle_controller);
    }
}

/*
// 获取各电机当前角度（未使用）
void GetMotorAngles(float* angles) {
    if (angles != NULL && angle_controller != NULL) {
        for (int i = 0; i < 4; i++) {
            angles[i] = CustomControllerGetMotorAngle(angle_controller, i);
        }
    }
}
*/