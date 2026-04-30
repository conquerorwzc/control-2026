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
#include <string.h>
#include <math.h> // 引入 math.h 以使用 M_PI

// 采样模式定义
typedef enum {
    SAMPLING_MODE_MOVE = 0, // MOVE: 电机松开/低阻尼，手动摆姿态
    SAMPLING_MODE_HOLD = 1  // HOLD: 记录角度并短暂保持，采集静态力矩
} SamplingMode_e;

static SamplingMode_e sampling_mode = SAMPLING_MODE_MOVE;
static float hold_angles[5] = {0};
static uint32_t hold_start_time = 0;

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

/**
 * @brief 执行重力补偿采样逻辑 (MOVE/HOLD 模式)
 */
static void GravitySamplingTask(void)
{
    if (angle_controller == NULL) return;

    if (sampling_mode == SAMPLING_MODE_MOVE) {
        // MOVE 模式：除了大 Yaw (索引0, 6020) 以外的电机设为开环零力矩
        for (int i = 1; i < 5; i++) {
            if (angle_controller->motors[i].dm_motor != NULL) {
                DMMotorEnable(angle_controller->motors[i].dm_motor);
                DMMotorSetRef(angle_controller->motors[i].dm_motor, 0.0f);
            }
        }
    } 
    else if (sampling_mode == SAMPLING_MODE_HOLD) {
        static bool just_entered = false;

        if (!just_entered) {
            // 【关键】进入 HOLD 瞬间，读取所有 DM 电机的实际角度
            for (int i = 1; i < 5; i++) {
                hold_angles[i] = angle_controller->motor_angles[i];
            }
            hold_start_time = HAL_GetTick();
            just_entered = true;
        }

        // 保持 2s：持续发送锁定的目标角度
        if (HAL_GetTick() - hold_start_time < 2000) {
            for (int i = 1; i < 5; i++) {
                if (angle_controller->motors[i].dm_motor != NULL) {
                    float target_rad = hold_angles[i] * (M_PI / 180.0f);
                    DMMotorSetPIDRef(angle_controller->motors[i].dm_motor, target_rad);
                }
            }
        } else {
            // 2s 结束，采集数据并发送
            // 协议格式: [Header(1)][Flag(1)][Angles(4*4)][Torques(4*4)] = 18 bytes
            uint8_t send_buf[18] = {0};
            send_buf[0] = 0xAA; // 起始位
            send_buf[1] = 0x01; // 标志位：重力采样数据
            
            int indices[4] = {1, 2, 3, 4}; // 对应: 大Roll, 大Pitch, 小Pitch, 小Roll
            for (int i = 0; i < 4; i++) {
                int idx = indices[i];
                memcpy(&send_buf[2 + i * 4], &hold_angles[idx], 4);   // 角度
            }
            
            for (int i = 0; i < 4; i++) {
                int idx = indices[i];
                float torque = angle_controller->motors[idx].dm_motor->measure.torque;
                memcpy(&send_buf[2 + 4 * 4 + i * 4], &torque, 4); // 力矩
            }

            if (angle_controller->usart_instance != NULL) {
                USARTSend(angle_controller->usart_instance, send_buf, 18, USART_TRANSFER_DMA);
            }
            
            // 恢复 MOVE 模式
            sampling_mode = SAMPLING_MODE_MOVE;
            just_entered = false;
        }
    }
}

void RobotInit() {
    // 创建初始化配置结构体
    CustomController_Init_Config_s init_config = {
        .dm4310_config_1 = DM4310_config_1,
        .dm4310_config_2 = DM4310_config_2,
        .dm4310_config_3 = DM4310_config_3,
        .dm4340_config = DM4340_config,
        .m6020_config = M6020_config
    };

    gpio_24V_R_EN = GPIORegister(&gpio_init_config_24v);
    GPIOSet(gpio_24V_R_EN);

    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);

    angle_controller = CustomControllerInit(&init_config);
    if (angle_controller == NULL) {
        return;
    }
}

void RobotTask() {
    if (angle_controller != NULL) {
        CustomControllerTask(angle_controller);
        GravitySamplingTask();
        //CustomController_SendAllData(angle_controller);
    }
}
