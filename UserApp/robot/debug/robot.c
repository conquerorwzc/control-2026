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

#define HOLD_TOTAL_MS            2700u
#define HOLD_SETTLE_MS            800u
#define SAMPLE_SEND_PERIOD_MS      50u

static uint32_t last_sample_send_time = 0;

static SamplingMode_e sampling_mode = SAMPLING_MODE_MOVE;
static float hold_angles[5] = {0};
static uint32_t hold_start_time = 0;
static uint8_t gravity_send_buf[30] = {0};  // DMA发送缓冲区，必须是static

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
 *
 * 当前 30 字节协议：
 * byte0  : 0xAA
 * byte1  : 0x01
 * float0 : theta2 大 roll，rad
 * float1 : theta3 大 pitch，rad
 * float2 : theta4 小 pitch，rad
 * float3 : theta5 小 roll，rad
 * float4 : tau2 大 roll，Nm
 * float5 : tau3 大 pitch，Nm
 * float6 : tau4 小 pitch，Nm
 *
 * 总长度：2 + 7 * 4 = 30 bytes
 */
static void GravitySamplingTask(void)
{
    if (angle_controller == NULL) return;

    if (sampling_mode == SAMPLING_MODE_MOVE) {
        /*
         * MOVE 模式：
         * J2~J5 达妙电机设为零力矩/可拖动。
         * 注意：这里默认 DMMotorSetRef(dm, 0.0f) 表示零力矩。
         */
        for (int i = 1; i < 5; i++) {
            if (angle_controller->motors[i].dm_motor != NULL) {
                DMMotorEnable(angle_controller->motors[i].dm_motor);
                DMMotorSetRef(angle_controller->motors[i].dm_motor, 0.0f);
            }
        }
        return;
    }

    if (sampling_mode == SAMPLING_MODE_HOLD) {
        static bool just_entered = false;

        uint32_t now = HAL_GetTick();

        if (!just_entered) {
            /*
             * 进入 HOLD 瞬间，保存当前角度作为 HOLD 目标。
             * 这里保持你的逻辑：
             * angle_controller->motor_angles[] 是 degree，
             * DMMotorSetPIDRef() 需要 rad。
             */
            for (int i = 1; i < 5; i++) {
                hold_angles[i] = angle_controller->motor_angles[i];  // degree
            }

            hold_start_time = now;
            last_sample_send_time = now;
            just_entered = true;
        }

        /*
         * HOLD 阶段：持续低刚度保持进入 HOLD 时的角度。
         * 你的写法是正确的：hold_angles 是 degree，SetPIDRef 需要 rad。
         */
        for (int i = 1; i < 5; i++) {
            if (angle_controller->motors[i].dm_motor != NULL) {
                float target_rad = hold_angles[i] * (M_PI / 180.0f);
                DMMotorSetPIDRef(angle_controller->motors[i].dm_motor, target_rad);
            }
        }

        /*
         * 300ms 以后开始发数据，20ms 发一帧，一直发到 2s。
         */
        if ((now - hold_start_time >= HOLD_SETTLE_MS) &&
            (now - hold_start_time < HOLD_TOTAL_MS) &&
            (now - last_sample_send_time >= SAMPLE_SEND_PERIOD_MS)) {

            last_sample_send_time = now;

            // 清空并填充发送缓冲区
            memset(gravity_send_buf, 0, sizeof(gravity_send_buf));
            
            gravity_send_buf[0] = 0xAA;
            gravity_send_buf[1] = 0x01;

            /*
             * 发送当前实时角度，而不是 hold_angles。
             * 因为真实采样姿态应该用当前实际角度。
             *
             * 角度顺序：
             * J2 大 roll  -> motor_angles[1]
             * J3 大 pitch -> motor_angles[2]
             * J4 小 pitch -> motor_angles[3]
             * J5 小 roll  -> motor_angles[4]
             *
             * 发送单位：rad
             */
            int angle_indices[4] = {1, 2, 3, 4};

            for (int i = 0; i < 4; i++) {
                int idx = angle_indices[i];

                float angle_rad = angle_controller->motor_angles[idx] * (M_PI / 180.0f);

                memcpy(&gravity_send_buf[2 + i * 4], &angle_rad, 4);
            }

            /*
             * 发送需要拟合的三个关节力矩：
             * J2 大 roll
             * J3 大 pitch
             * J4 小 pitch
             *
             * J5 小 roll 不输出补偿，所以不采 tau5。
             */
            int torque_indices[3] = {1, 2, 3};

            for (int i = 0; i < 3; i++) {
                int idx = torque_indices[i];

                float torque = 0.0f;
                
                if (angle_controller->motors[idx].dm_motor != NULL) {
                    torque = angle_controller->motors[idx].dm_motor->measure.torque;
                }

                memcpy(&gravity_send_buf[2 + 4 * 4 + i * 4], &torque, 4);
            }

            if (angle_controller->usart_instance != NULL) {
                USARTSend(angle_controller->usart_instance,
                          gravity_send_buf,
                          sizeof(gravity_send_buf),
                          USART_TRANSFER_DMA);
            }
        }

        /*
         * 2s 后自动回到 MOVE。
         * 你现在用 debug 手动改 sampling_mode 也没问题；
         * 它会自动从 HOLD 回 MOVE。
         */
        if (now - hold_start_time >= HOLD_TOTAL_MS) {
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
