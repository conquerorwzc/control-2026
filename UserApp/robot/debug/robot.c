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
#define TORQUE_DEADBAND       1.5f    // 所有电机角度死区(度)

// 不同电机类型的力控比例增益 (Nm/度)
#define TORQUE_K_P_DM4310     0.01f   // DM4310比例增益
#define TORQUE_K_P_DM4340     0.01f   // DM4340比例增益（扭矩更大）
#define TORQUE_K_P_M6020      0.008f  // M6020比例增益

#define MAX_TORQUE_DM4310     3.0f    // DM4310最大输出力矩限制(N·m)
#define MAX_TORQUE_DM4340     3.0f    // DM4340最大输出力矩限制(N·m)
#define MAX_TORQUE_M6020      1.0f    // M6020最大输出力矩限制(额定扭矩内，保护电机)

// 电机扭矩常数 (Nm/A)
#define KT_DM4310             1.0f    // DM4310直接用力矩控制
#define KT_DM4340             1.0f    // DM4340直接用力矩控制
#define KT_M6020              0.741f  // M6020扭矩常数

// GM6020 (M6020): 控制量 16384 对应 3A
#define GM6020_RAW_PER_AMP    (16384.0f / 3.0f)  // 约 5461.3

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
 * @brief 应用力反馈控制
 * @note 仅在自定义控制器模式下生效
 */
static void ApplyTorqueFeedback(void)
{
    if (angle_controller == NULL || !angle_controller->robot_data_valid) {
        return;
    }
    
    // Debug模式：离线测试，不需要检查robot_grab_mode
    // if (angle_controller->robot_grab_mode != 1) {
    //     return;
    // }

    float custom_angles[5], real_angles[5], angle_errors[5], torque_outputs[5];

    // 电机索引映射：自定义控制器电机 -> 工程板发送的角度索引
    // motor[0]=大yaw, [1]=大roll, [2]=大pitch, [3]=小pitch, [4]=小roll
    const uint8_t motor_to_angle_map[5] = {
        0,  // motors[0] (大yaw M6020)    <- base_joint (index 0)
        1,  // motors[1] (大roll DM4340)  <- elbow_roll (index 1)
        2,  // motors[2] (大pitch DM4310) <- elbow_pitch (index 2)
        3,  // motors[3] (小pitch DM4310) <- wrist_pitch (index 3)
        4   // motors[4] (小roll DM4310)  <- wrist_roll (index 4)
    };

    // 对5个关节分别计算力反馈
    for (int i = 0; i < 5; i++) {
        // 获取自定义控制器角度和真实机械臂角度（使用映射后的索引）
        uint8_t angle_idx = motor_to_angle_map[i];
        custom_angles[i] = angle_controller->motor_angles[i];
        real_angles[i] = angle_controller->robot_arm_angles[angle_idx];

        // 计算角度误差（取反以修正方向）
        angle_errors[i] = real_angles[i] - custom_angles[i];

        // 根据电机类型选择对应的Kp参数
        float torque_k_p;
        if (angle_controller->motors[i].dm_motor != NULL) {
            // 根据DM电机型号选择Kp
            if (angle_controller->motors[i].dm_motor->motor_type == J4340) {
                torque_k_p = TORQUE_K_P_DM4340;
            } else {
                torque_k_p = TORQUE_K_P_DM4310;
            }
        } else {
            // DJI电机
            torque_k_p = TORQUE_K_P_M6020;
        }

        // 对所有电机统一应用角度死区
        torque_outputs[i] = 0.0f;
        if (angle_errors[i] > TORQUE_DEADBAND) {
            torque_outputs[i] = torque_k_p * (angle_errors[i] - TORQUE_DEADBAND);
        } else if (angle_errors[i] < -TORQUE_DEADBAND) {
            torque_outputs[i] = torque_k_p * (angle_errors[i] + TORQUE_DEADBAND);
        }

        // 根据电机类型分别限幅和设置
        if (angle_controller->motors[i].dm_motor != NULL) {
            // DM电机：使用物理力矩单位(N·m)，根据型号限幅
            float max_torque;
            if (angle_controller->motors[i].dm_motor->motor_type == J4340) {
                max_torque = MAX_TORQUE_DM4340;
            } else {
                max_torque = MAX_TORQUE_DM4310;
            }
            
            if (torque_outputs[i] > max_torque) {
                torque_outputs[i] = max_torque;
            } else if (torque_outputs[i] < -max_torque) {
                torque_outputs[i] = -max_torque;
            }
            
            // 确保DM电机已使能，否则无法输出力矩
            DMMotorEnable(angle_controller->motors[i].dm_motor);
            
            // DM电机直接使用物理力矩单位，不需要乘以1000
            switch (angle_controller->motors[i].dm_motor->motor_type) {
                case J4310:
                case J4340:
                    DMMotorSetRef(angle_controller->motors[i].dm_motor, torque_outputs[i]);
                    break;
                default:
                    break;
            }
        } else if (angle_controller->motors[i].dji_motor != NULL) {
            // GM6020：力矩 → 电流 → CAN控制量
            float current_a = 0.0f;  // 所需电流(A)
            int16_t raw_cmd = 0;     // CAN原始控制量

            // 限幅：确保在额定扭矩内
            if (torque_outputs[i] > MAX_TORQUE_M6020) {
                torque_outputs[i] = MAX_TORQUE_M6020;
            } else if (torque_outputs[i] < -MAX_TORQUE_M6020) {
                torque_outputs[i] = -MAX_TORQUE_M6020;
            }

            // 力矩转电流：T = Kt * I → I = T / Kt
            current_a = torque_outputs[i] / KT_M6020;
            
            // 电流转CAN控制量：16384对应3A
            raw_cmd = (int16_t)(current_a * GM6020_RAW_PER_AMP);

            // 设置参考值（final_output会被转换为int16发送到CAN）
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
        // 更新电机角度数据
        CustomControllerTask(angle_controller);
        
        // 应用力反馈控制（仅在自定义控制器模式下）
        ApplyTorqueFeedback();
        
        // 发送所有电机数据通过 USART3
        CustomController_SendAllData(angle_controller);
    }
}
