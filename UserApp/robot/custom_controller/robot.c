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
#include "gravity_trig_model.h" // 引入重力补偿模型
#include <stdbool.h>
#include <math.h>

// 力反馈参数配置 (针对 10Hz 低频链路优化)
#define TORQUE_DEADBAND       3.8f    // 所有电机角度死区(度)

// 刚度系数 (Nm/deg) - 降低以避免延迟震荡
#define TORQUE_K_P_DM4310     0.02f
#define TORQUE_K_P_DM4340     0.02f
#define TORQUE_K_P_M6020      0.012f

// 阻尼系数 (Nm/(deg/s)) - 本地速度阻尼，抑制震荡
#define TORQUE_K_D_DM4310     0.006f   
#define TORQUE_K_D_DM4340     0.006f   
#define TORQUE_K_D_M6020      0.003f   

// 力反馈力矩限幅 (仅对力反馈部分限幅)
#define MAX_FEEDBACK_TORQUE_DM4310  0.35f
#define MAX_FEEDBACK_TORQUE_DM4340  0.35f
#define MAX_FEEDBACK_TORQUE_M6020   0.20f

// 力矩斜率限制 (Nm/cycle) - 缓解 10Hz 数据跳变
#define MAX_TORQUE_STEP_DM          0.02f
#define MAX_TORQUE_STEP_M6020       0.005f

// 电机扭矩常数 (Nm/A)
#define KT_DM4310             1.0f    // DM4310直接用力矩控制
#define KT_DM4340             1.0f    // DM4340直接用力矩控制
#define KT_M6020              0.741f   // M6020扭矩常数

// GM6020 (M6020): 控制量 16384 对应 3A
#define GM6020_RAW_PER_AMP    (16384.0f / 3.0f)  // 约 5461.3

// 重力补偿验证系数 (0.0 ~ 1.0) - 可在调试时动态修改
// 建议从 0.1 开始验证方向，确认无误后再改为 1.0
static float gravity_comp_scale_roll = 0.5f;   // Roll轴(J2)重力补偿比例
static float gravity_comp_scale_pitch1 = 1.0f; // Pitch轴(J3)重力补偿比例
static float gravity_comp_scale_pitch2 = 1.0f; // Pitch轴(J4)重力补偿比例

// 自定义控制器实例
static CustomController_t* angle_controller;

// ===== 重力补偿调试变量，可在 Debug Watch 里查看 =====
static float dbg_q2 = 0.0f;
static float dbg_q3 = 0.0f;
static float dbg_q4 = 0.0f;
static float dbg_q5 = 0.0f;

static float dbg_g2 = 0.0f;
static float dbg_g3 = 0.0f;
static float dbg_g4 = 0.0f;

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
 * @brief 计算重力补偿力矩（基于三角函数模型）
 */
static void CalculateGravityCompensation(float gravity_torques[5])
{
    if (angle_controller == NULL) return;

    // 获取当前角度并转换为弧度 (q2, q3, q4, q5)
    // 假设 motor_angles 索引对应: [0]=Yaw, [1]=Roll(J2), [2]=Pitch(J3), [3]=Pitch(J4), [4]=Roll(J5)
    float q2 = angle_controller->motor_angles[1] * (M_PI / 180.0f);
    float q3 = angle_controller->motor_angles[2] * (M_PI / 180.0f);
    float q4 = angle_controller->motor_angles[3] * (M_PI / 180.0f);
    float q5 = angle_controller->motor_angles[4] * (M_PI / 180.0f);

    // 根据模型计算各关节的重力力矩
    // 注意：模型中的 J2, J3, J4 对应你的大 Roll, 大 Pitch, 小 Pitch
    gravity_torques[0] = 0.0f; // Yaw 轴通常不需要重力补偿
    gravity_torques[1] = 0.0f;//MecArm_Gravity_J2(q2, q3, q4, q5); // 大 Roll
    gravity_torques[2] = 0.0f;//MecArm_Gravity_J3(q2, q3, q4, q5); // 大 Pitch
    gravity_torques[3] = 0.0f;//MecArm_Gravity_J4(q2, q3, q4, q5); // 小 Pitch
    gravity_torques[4] = 0.0f; // 小 Roll (根据你的需求暂时不补偿或没有模型)

    dbg_q2 = q2;
    dbg_q3 = q3;
    dbg_q4 = q4;
    dbg_q5 = q5;

    dbg_g2 = gravity_torques[1];
    dbg_g3 = gravity_torques[2];
    dbg_g4 = gravity_torques[3];
}

/**
 * @brief 辅助函数：限幅
 */
static float LimitFloat(float x, float min_val, float max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

/**
 * @brief 辅助函数：斜率限制
 */
static float LimitSlew(float target, float last, float max_step)
{
    float delta = target - last;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return last + delta;
}

/**
 * @brief 计算力反馈力矩 (含本地阻尼与斜率限制)
 */
static void CalculateFeedbackTorque(float feedback_torques[5])
{
    // 静态变量用于计算速度和斜率限制
    static float last_custom_angles[5] = {0.0f};
    static float last_feedback_torques[5] = {0.0f};
    static uint32_t last_calc_time = 0;
    static bool feedback_state_inited = false;

    if (angle_controller == NULL) {
        for (int i = 0; i < 5; i++) feedback_torques[i] = 0.0f;
        feedback_state_inited = false;
        return;
    }

    // 仅在 mode=1 时计算力反馈
    if (angle_controller->robot_grab_mode != 1) {
        for (int i = 0; i < 5; i++) {
            feedback_torques[i] = 0.0f;
            last_feedback_torques[i] = 0.0f;
        }
        feedback_state_inited = false;
        return;
    }

    // 数据太久没更新，直接撤力，防止拿旧数据持续推
    if (!angle_controller->robot_data_valid ||
        (HAL_GetTick() - angle_controller->last_robot_data_time) > 300) {
        for (int i = 0; i < 5; i++) {
            feedback_torques[i] = 0.0f;
            last_feedback_torques[i] = 0.0f;
        }
        feedback_state_inited = false;
        return;
    }

    // 状态初始化：避免首次进入或断连恢复时的速度尖峰
    if (!feedback_state_inited) {
        for (int i = 0; i < 5; i++) {
            last_custom_angles[i] = angle_controller->motor_angles[i];
            last_feedback_torques[i] = 0.0f;
            feedback_torques[i] = 0.0f;
        }
        last_calc_time = HAL_GetTick();
        feedback_state_inited = true;
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = 0.001f * (float)(now - last_calc_time);
    if (last_calc_time == 0 || dt <= 0.0f || dt > 0.05f) {
        dt = 0.005f; // 默认按 200Hz 周期计算
    }
    last_calc_time = now;

    const uint8_t motor_to_angle_map[5] = {
        0,  // motors[0] (大yaw M6020)
        1,  // motors[1] (大roll DM4340)
        2,  // motors[2] (大pitch DM4310)
        3,  // motors[3] (小pitch DM4310)
        4   // motors[4] (小roll DM4310)
    };

    for (int i = 0; i < 5; i++) {
        uint8_t angle_idx = motor_to_angle_map[i];
        float custom_angle = angle_controller->motor_angles[i];
        float real_angle = angle_controller->robot_arm_angles[angle_idx];
        float error = real_angle - custom_angle;

        // 计算本地速度 (deg/s)
        float custom_vel = (custom_angle - last_custom_angles[i]) / dt;
        last_custom_angles[i] = custom_angle;

        // 选择参数
        float kp, kd, max_torque, max_step;
        if (angle_controller->motors[i].dm_motor != NULL) {
            if (angle_controller->motors[i].dm_motor->motor_type == J4340) {
                kp = TORQUE_K_P_DM4340; kd = TORQUE_K_D_DM4340;
                max_torque = MAX_FEEDBACK_TORQUE_DM4340;
            } else {
                kp = TORQUE_K_P_DM4310; kd = TORQUE_K_D_DM4310;
                max_torque = MAX_FEEDBACK_TORQUE_DM4310;
            }
            max_step = MAX_TORQUE_STEP_DM;
        } else {
            kp = TORQUE_K_P_M6020; kd = TORQUE_K_D_M6020;
            max_torque = MAX_FEEDBACK_TORQUE_M6020;
            max_step = MAX_TORQUE_STEP_M6020;
        }

        // 1. 弹簧项 (带死区)
        float spring_torque = 0.0f;
        if (error > TORQUE_DEADBAND) {
            spring_torque = kp * (error - TORQUE_DEADBAND);
        } else if (error < -TORQUE_DEADBAND) {
            spring_torque = kp * (error + TORQUE_DEADBAND);
        }

        // 2. 阻尼项 (本地速度)
        float damping_torque = -kd * custom_vel;

        // 3. 总力矩 = 弹簧 + 阻尼
        float torque = spring_torque + damping_torque;

        // 4. 限幅
        torque = LimitFloat(torque, -max_torque, max_torque);

        // 5. 斜率限制
        torque = LimitSlew(torque, last_feedback_torques[i], max_step);

        last_feedback_torques[i] = torque;
        feedback_torques[i] = torque;
    }
}

/**
 * @brief 应用总力矩控制（力反馈 + 重力补偿）或角度跟随
 */
static void ApplyTotalTorque(void)
{
    if (angle_controller == NULL) {
        return;
    }

    // mode=2: 角度跟随模式
    if (angle_controller->robot_grab_mode == 2) {
        const uint8_t motor_to_angle_map[5] = {
            0,  // motors[0] (大yaw M6020)
            1,  // motors[1] (大roll DM4340)
            2,  // motors[2] (大pitch DM4310)
            3,  // motors[3] (小pitch DM4310)
            4   // motors[4] (小roll DM4310)
        };

        for (int i = 0; i < 5; i++) {
            uint8_t angle_idx = motor_to_angle_map[i];
            float target_angle = angle_controller->robot_arm_angles[angle_idx];

            if (angle_controller->motors[i].dm_motor != NULL) {
                // DM电机使用位置控制（弧度）
                DMMotorSetPIDRef(angle_controller->motors[i].dm_motor, target_angle * (M_PI / 180.0f));
            } else if (angle_controller->motors[i].dji_motor != NULL) {
                // DJI电机（GM6020）使用位置控制（度），需要补回零点偏移
                // zero_offset[0] = -96.5f，发送时减去了它，接收后要加回来
                float actual_angle = target_angle + angle_controller->zero_offset[0];
                DJIMotorSetPIDRef(angle_controller->motors[i].dji_motor, actual_angle);
            }
        }
        return;
    }

    // mode=1 或其他: 力反馈模式
    float feedback_torques[5];
    float gravity_torques[5];
    float total_torques[5];

    // 1. 计算力反馈力矩
    CalculateFeedbackTorque(feedback_torques);

    // 2. 计算重力补偿力矩
    CalculateGravityCompensation(gravity_torques);

    // 3. 总力矩 = 重力补偿 * 验证系数 + 力反馈
    for (int i = 0; i < 5; i++) {
        float scale = 1.0f;
        if (i == 1) scale = gravity_comp_scale_roll;
        else if (i == 2) scale = gravity_comp_scale_pitch1;
        else if (i == 3) scale = gravity_comp_scale_pitch2;
        
        total_torques[i] = (gravity_torques[i] * scale) + feedback_torques[i];

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
        
        // 【强行使能】在每个周期确保电机处于工作状态
        for (int i = 1; i < 5; i++) {
            if (angle_controller->motors[i].dm_motor != NULL) {
                DMMotorEnable(angle_controller->motors[i].dm_motor);
            }
        }

        // 应用总力矩控制（力反馈+重力补偿）
        ApplyTotalTorque();
        
        CustomController_SendAllData(angle_controller);
    }
}
