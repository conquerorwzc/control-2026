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
#include <math.h>

// 力反馈参数配置
#define TORQUE_DEADBAND       3.0f    // 所有电机角度死区(度)

// 不同电机类型的力控比例增益 (Nm/度)
#define TORQUE_K_P_DM4310     0.05f   // DM4310比例增益
#define TORQUE_K_P_DM4340     0.05f   // DM4340比例增益
#define TORQUE_K_P_M6020      0.04f   // M6020比例增益

// 电机扭矩常数 (Nm/A)
#define KT_DM4310             1.0f    // DM4310直接用力矩控制
#define KT_DM4340             1.0f    // DM4340直接用力矩控制
#define KT_M6020              0.741f   // M6020扭矩常数

// GM6020 (M6020): 控制量 16384 对应 3A
#define GM6020_RAW_PER_AMP    (16384.0f / 3.0f)  // 约 5461.3

// 力反馈力矩限幅 (仅对力反馈部分限幅，重力补偿不限幅)
#define MAX_FEEDBACK_TORQUE_DM4310     3.0f    // DM4310力反馈力矩限制(N·m)
#define MAX_FEEDBACK_TORQUE_DM4340     3.0f    // DM4340力反馈力矩限制(N·m)
#define MAX_FEEDBACK_TORQUE_M6020   2.0f    // M6020力反馈力矩限制(N·m)

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
 * @brief 计算重力补偿力矩（预留接口，目前全置零）
 */
static void CalculateGravityCompensation(float gravity_torques[5])
{
    // TODO: 实现重力补偿算法
    for (int i = 0; i < 5; i++) {
        gravity_torques[i] = 0.0f;
    }
}

/**
 * @brief 计算力反馈力矩
 */
static void CalculateFeedbackTorque(float feedback_torques[5])
{
    if (angle_controller == NULL || !angle_controller->robot_data_valid) {
        return;
    }

    // 仅在mode=1时计算力反馈
    if (angle_controller->robot_grab_mode != 1) {
        for (int i = 0; i < 5; i++) {
            feedback_torques[i] = 0.0f;
        }
        return;
    }

    float custom_angles[5], real_angles[5], angle_errors[5];

    const uint8_t motor_to_angle_map[5] = {
        0,  // motors[0] (大yaw M6020)
        1,  // motors[1] (大roll DM4340)
        2,  // motors[2] (大pitch DM4310)
        3,  // motors[3] (小pitch DM4310)
        4   // motors[4] (小roll DM4310)
    };

    for (int i = 0; i < 5; i++) {
        uint8_t angle_idx = motor_to_angle_map[i];
        custom_angles[i] = angle_controller->motor_angles[i];
        real_angles[i] = angle_controller->robot_arm_angles[angle_idx];
        angle_errors[i] = real_angles[i] - custom_angles[i];

        // 选择Kp
        float torque_k_p;
        if (angle_controller->motors[i].dm_motor != NULL) {
            torque_k_p = (angle_controller->motors[i].dm_motor->motor_type == J4340) ? 
                         TORQUE_K_P_DM4340 : TORQUE_K_P_DM4310;
        } else {
            torque_k_p = TORQUE_K_P_M6020;
        }

        // 死区处理
        feedback_torques[i] = 0.0f;
        if (angle_errors[i] > TORQUE_DEADBAND) {
            feedback_torques[i] = torque_k_p * (angle_errors[i] - TORQUE_DEADBAND);
        } else if (angle_errors[i] < -TORQUE_DEADBAND) {
            feedback_torques[i] = torque_k_p * (angle_errors[i] + TORQUE_DEADBAND);
        }

        // 力反馈限幅
        if (angle_controller->motors[i].dm_motor != NULL) {
            float max_fb;
            if (angle_controller->motors[i].dm_motor->motor_type == J4340) {
                max_fb = MAX_FEEDBACK_TORQUE_DM4340;
            } else {
                max_fb = MAX_FEEDBACK_TORQUE_DM4310;
            }
            
            if (feedback_torques[i] > max_fb) {
                feedback_torques[i] = max_fb;
            } else if (feedback_torques[i] < -max_fb) {
                feedback_torques[i] = -max_fb;
            }
        } else {
            if (feedback_torques[i] > MAX_FEEDBACK_TORQUE_M6020) {
                feedback_torques[i] = MAX_FEEDBACK_TORQUE_M6020;
            } else if (feedback_torques[i] < -MAX_FEEDBACK_TORQUE_M6020) {
                feedback_torques[i] = -MAX_FEEDBACK_TORQUE_M6020;
            }
        }
    }
}

/**
 * @brief 应用总力矩控制（力反馈 + 重力补偿）
 */
static void ApplyTotalTorque(void)
{
    if (angle_controller == NULL || !angle_controller->robot_data_valid) {
        return;
    }

    float feedback_torques[5];
    float gravity_torques[5];
    float total_torques[5];

    // 1. 计算力反馈力矩
    CalculateFeedbackTorque(feedback_torques);

    // 2. 计算重力补偿力矩
    CalculateGravityCompensation(gravity_torques);

    // 3. 总力矩 = 力反馈 + 重力补偿
    for (int i = 0; i < 5; i++) {
        total_torques[i] = gravity_torques[i] + feedback_torques[i];

        // 4. 发送到电机
        if (angle_controller->motors[i].dm_motor != NULL) {
            DMMotorEnable(angle_controller->motors[i].dm_motor);
            DMMotorSetRef(angle_controller->motors[i].dm_motor, total_torques[i]);
        } else if (angle_controller->motors[i].dji_motor != NULL) {
            float current_a = total_torques[i] / KT_M6020;
            int16_t raw_cmd = (int16_t)(current_a * GM6020_RAW_PER_AMP);
            DJIMotorSetRef(angle_controller->motors[i].dji_motor, (float)raw_cmd);
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
        // 错误处理可以根据需要添加
        return;

    }
}

void RobotTask() {
    if (angle_controller != NULL) {
        CustomControllerTask(angle_controller);
        
        // 应用总力矩控制（力反馈+重力补偿）
        ApplyTotalTorque();
        
        CustomController_SendAllData(angle_controller);
    }
}
