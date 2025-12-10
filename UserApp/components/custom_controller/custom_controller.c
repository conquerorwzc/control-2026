#include "custom_controller.h"
#include "stdlib.h"
#include "memory.h"
#include "usart.h"
#include "stdio.h"
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
    
    // 初始化ADC
    ADC_Init_Config_s adc_configs[POTENTIOMETER_COUNT] = {
        {
            .hadc = &POTENTIOMETER_ADC_HANDLE1,
            .channel = POTENTIOMETER_ADC_CHANNEL_1,
            .mode = ADC_MODE_POLLING,
            .vref = 3.3f,
            .callback = NULL,
            .id = NULL
        },
        {
            .hadc = &POTENTIOMETER_ADC_HANDLE2,
            .channel = POTENTIOMETER_ADC_CHANNEL_2,
            .mode = ADC_MODE_POLLING,
            .vref = 3.3f,
            .callback = NULL,
            .id = NULL
        }
    };
    
    for (int i = 0; i < POTENTIOMETER_COUNT; i++) {
        instance->potentiometer.adc[i] = ADCRegister(&adc_configs[i]);
        if (instance->potentiometer.adc[i] == NULL) {
            LOGERROR("Failed to initialize ADC for potentiometer %d", i);
        }
    }
    
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
    
    // 读取电位器值并转换为角度 (0-3.3V 映射到 0-270度)
    for (int i = 0; i < POTENTIOMETER_COUNT; i++) {
        controller_instance->potentiometer.values[i] = ADCGetRawValue(controller_instance->potentiometer.adc[i]);
        controller_instance->potentiometer.angles[i] = (float)controller_instance->potentiometer.values[i] * 270.0f / 65536.0f;
    }

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