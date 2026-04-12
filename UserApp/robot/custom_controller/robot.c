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
#define TORQUE_DEADBAND_4310    2.0f    // DM4310角度死区(度)
#define TORQUE_DEADBAND_DJI     1.5f    // DJI电机角度死区(度)
#define TORQUE_K_P              0.05f   // 比例增益(Nm/度)
#define MAX_TORQUE_OUTPUT       1.0f    // 最大输出力矩限制(Nm)

// 电机扭矩常数 (Nm/A)
#define KT_DM4310               1.0f    // DM4310直接用力矩控制
#define KT_M3508                0.3f    // M3508扭矩常数
#define KT_M2006                0.18f   // M2006扭矩常数

// 减速比
#define RATIO_M2006             36.0f   // M2006减速比1:36
#define RATIO_M3508             1.0f    // M3508直驱

// C620 (对应3508): 控制量 16384 对应 20A
#define C620_RAW_PER_AMP    (16384.0f / 20.0f)  // 约 819.2

// C610 (对应2006): 控制量 10000 对应 10A
#define C610_RAW_PER_AMP    (10000.0f / 10.0f)  // 1000.0

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
    
    // 仅在机械臂处于自定义控制器模式时启用力反馈
    if (angle_controller->robot_grab_mode != 2) {
        return;
    }
    
    float custom_angle, real_angle, angle_error, torque_output;
    
    // 对5个关节分别计算力反馈
    for (int i = 0; i < 5; i++) {
        // 获取自定义控制器角度和真实机械臂角度
        custom_angle = angle_controller->robot_arm_angles[i];
        real_angle = angle_controller->motor_angles[i];
        
        // 计算角度误差
        angle_error = custom_angle - real_angle;
        
        // 平滑死区逻辑（根据电机类型设定不同死区）
        float current_deadband;
        if (angle_controller->motors[i].dm_motor != NULL) {
            current_deadband = TORQUE_DEADBAND_4310;
        } else {
            current_deadband = TORQUE_DEADBAND_DJI;
        }
        
        torque_output = 0.0f;
        if (angle_error > current_deadband) {
            torque_output = TORQUE_K_P * (angle_error - current_deadband);
        } else if (angle_error < -current_deadband) {
            torque_output = TORQUE_K_P * (angle_error + current_deadband);
        }
        
        // 安全限幅
        if (torque_output > MAX_TORQUE_OUTPUT) {
            torque_output = MAX_TORQUE_OUTPUT;
        } else if (torque_output < -MAX_TORQUE_OUTPUT) {
            torque_output = -MAX_TORQUE_OUTPUT;
        }
        
        // 根据电机类型设置力矩
        if (angle_controller->motors[i].dm_motor != NULL) {
            switch (angle_controller->motors[i].dm_motor->motor_type) {
                case J4310:
                    // DM4310：直接接收目标力矩
                    DMMotorSetRef(angle_controller->motors[i].dm_motor, torque_output);
                    break;
                default:
                    break;
            }
        } else if (angle_controller->motors[i].dji_motor != NULL) {
            float current_a = 0.0f;  // 所需电流(A)
            int16_t raw_cmd = 0;     // 电调原始控制值
            
            switch (angle_controller->motors[i].dji_motor->motor_type) {
                case M3508:
                    // 计算电流: I = T_out / (Kt * Ratio)
                    current_a = torque_output / (KT_M3508 * RATIO_M3508);
                    // 转换为C620原始值
                    raw_cmd = (int16_t)(current_a * C620_RAW_PER_AMP);
                    break;
                    
                case M2006:
                    // 计算电流: I = T_out / (Kt * Ratio)
                    current_a = torque_output / (KT_M2006 * RATIO_M2006);
                    // 转换为C610原始值
                    raw_cmd = (int16_t)(current_a * C610_RAW_PER_AMP);
                    break;
                    
                default:
                    raw_cmd = 0;
                    break;
            }
            
            angle_controller->motors[i].dji_motor->motor_controller.final_output = raw_cmd;
        }
    }
}

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
