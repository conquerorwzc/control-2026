#include "custom_controller.h"
//#include "adc.h"
#include "stdlib.h"
#include "memory.h"
#include "usart.h"
#include "stdio.h"
#include "bsp_log.h"
#include <stdbool.h>

#include "cmsis_os.h"

// 全局实例指针
static CustomControllerInstance* instance = NULL;

/**
 * @brief 初始化自定义控制器
 * 
 * @return CustomControllerInstance* 控制器实例指针
 */
CustomControllerInstance* CustomControllerInit(void) {
    // 分配内存
    instance = (CustomControllerInstance*)malloc(sizeof(CustomControllerInstance));
    if (instance == NULL) {
        LOGERROR("Failed to allocate memory for custom controller");
        return NULL;
    }
    
    // 清空内存
    memset(instance, 0, sizeof(CustomControllerInstance));
    
    // 初始化舵机
    Servo_Init_Config_s servo_configs[SERVO_MOTOR_COUNT] = {
        {
            .servo_type = Bus_Servo,  // 使用ASCII协议舵机
            ._handle = &huart1,  // 根据实际连接修改
            .servo_id = 1
        },
        {
            .servo_type = Bus_Servo,  // 使用ASCII协议舵机
            ._handle = &huart1,  // 根据实际连接修改
            .servo_id = 2
        },
        {
            .servo_type = Bus_Servo,  // 使用ASCII协议舵机
            ._handle = &huart1,  // 根据实际连接修改
            .servo_id = 3
        }
    };
    
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        instance->servo_motors[i] = ServoInit(&servo_configs[i]);
        if (instance->servo_motors[i] != NULL) {
            // 设置舵机为释力模式，允许手动摆动
            Bus_Servo_Unload(instance->servo_motors[i]);
            LOGINFO("Servo %d initialized successfully", servo_configs[i].servo_id);
        } else {
            LOGERROR("Failed to initialize servo %d", servo_configs[i].servo_id);
        }
    }
    
    // 检查是否有任何舵机初始化失败
    bool init_success = true;
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        if (instance->servo_motors[i] == NULL) {
            init_success = false;
        }
    }
    
    if (!init_success) {
        LOGERROR("One or more servos failed to initialize");
        // 释放已分配的内存
        free(instance);
        instance = NULL;
        return NULL;
    }
    
    // 初始化ADC
    //MX_ADC1_Init(); // 确保ADC1已在adc.c中正确初始化
    
    LOGINFO("Custom controller initialized successfully");
    return instance;
}

/**
 * @brief 更新自定义控制器状态
 * 
 * @param controller_instance 控制器实例指针
 */
void CustomControllerUpdate(CustomControllerInstance* controller_instance) {
    if (controller_instance == NULL) {
        return;
    }
    
    // 读取电位器值
    // 暂时注释掉电位器相关代码，因为电位器还没有连线
    /*
    // 启动ADC转换
    HAL_ADC_Start(&POTENTIOMETER_ADC_HANDLE);
    
    // 读取第一个电位器
    HAL_ADC_PollForConversion(&POTENTIOMETER_ADC_HANDLE, HAL_MAX_DELAY);
    controller_instance->potentiometer_values[0] = HAL_ADC_GetValue(&POTENTIOMETER_ADC_HANDLE);
    
    // 配置并读取第二个电位器
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = POTENTIOMETER_ADC_CHANNEL_2;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    
    if (HAL_ADC_ConfigChannel(&POTENTIOMETER_ADC_HANDLE, &sConfig) == HAL_OK) {
        HAL_ADC_PollForConversion(&POTENTIOMETER_ADC_HANDLE, HAL_MAX_DELAY);
        controller_instance->potentiometer_values[1] = HAL_ADC_GetValue(&POTENTIOMETER_ADC_HANDLE);
    }
    
    // 停止ADC
    HAL_ADC_Stop(&POTENTIOMETER_ADC_HANDLE);
    */

    // 发送读取舵机位置命令（针对ASCII协议舵机）
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        if (controller_instance->servo_motors[i] != NULL) {
            // 发送读取角度命令给每个舵机
            Bus_Servo_GetAngle(controller_instance->servo_motors[i]);
        }
    }

    // 注意：实际的角度值将在DecodeServo回调函数中解析并存储在recv_angle中
    // 这里可以使用存储的角度值
    for (int i = 0; i < SERVO_MOTOR_COUNT; i++) {
        if (controller_instance->servo_motors[i] != NULL) {
            float angle = controller_instance->servo_motors[i]->recv_angle / 10.0f;  // 转换为实际角度值
            LOGINFO("Servo %d Angle: %.2f", controller_instance->servo_motors[i]->servo_id, angle);
        }
    }
}