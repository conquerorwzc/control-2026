#include "custom_controller.h"
#include "stdlib.h"
#include "memory.h"
#include "usart.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"

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

    // 初始化舵机 - 只初始化ID为1的舵机
    instance->servo_motors[0] = SerialServoInit(NULL);
    if (instance->servo_motors[0] != NULL) {
        // 创建 USART 配置
        USART_Init_Config_s usart_config = {0};
        usart_config.usart_handle = &huart6;
        usart_config.recv_buff_size = 1; // 每次接收1个字节
        usart_config.module_callback = NULL; // 可以根据需要添加回调函数
        
        // 注册 USART 实例
        USARTInstance* usart_instance = USARTRegister(&usart_config);
        
        Servo_Init(instance->servo_motors[0], 1, usart_instance);
        LOGINFO("Servo %d initialized", 1);
        
        // 延迟0.1秒发送释力命令，确保系统稳定
        osDelay(100);
        // 使用新的舵机控制接口
        Servo_SetTorque(instance->servo_motors[0], false); // false表示卸载，释力
        LOGINFO("Servo %d set to unload state", 1);
        
        // 再增加一些延时确保舵机进入卸载状态
        osDelay(200);
    } else {
        LOGERROR("Failed to initialize servo %d", 1);
    }

    // 初始化ADC
    ADC_Init_Config_s adc_configs[POTENTIOMETER_COUNT] = {
        {
            .hadc = &POTENTIOMETER_ADC_HANDLE1,
            .channel = POTENTIOMETER_ADC_CHANNEL_1,
            .mode = ADC_MODE_POLLING,
            .vref = 3.3f,
            .alpha = 0.3f,  // 添加EWMA滤波，alpha值为0.3
            .callback = NULL,
            .id = NULL
        },
        {
            .hadc = &POTENTIOMETER_ADC_HANDLE2,
            .channel = POTENTIOMETER_ADC_CHANNEL_2,
            .mode = ADC_MODE_POLLING,
            .vref = 3.3f,
            .alpha = 0.3f,  // 添加EWMA滤波，alpha值为0.3
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

    // 发送读取舵机位置命令（针对串行舵机）
    // 只处理ID为1的舵机
    if (controller_instance->servo_motors[0] != NULL) {
        // 主动请求舵机角度
        LOGINFO("About to request angle from servo %d", controller_instance->servo_motors[0]->id);
        
        // 使用新的舵机控制接口请求位置
        Servo_ReadPosition(controller_instance->servo_motors[0]);
        
        // 等待一段时间让舵机返回数据
        osDelay(300); // 进一步增加等待时间到300ms
        
        // 检查角度值是否合理再打印
        // 将current_angle转换为角度值（根据协议，0-1000映射到0-240度）
        // 在访问变量前先进行缓存同步
        SCB_InvalidateDCache_by_Addr((uint32_t*)&(controller_instance->servo_motors[0]->present_pos), sizeof(controller_instance->servo_motors[0]->present_pos));
        
        LOGINFO("Raw angle value: %d", controller_instance->servo_motors[0]->present_pos);
        
        if (controller_instance->servo_motors[0]->present_pos >= 0 && 
            controller_instance->servo_motors[0]->present_pos <= 1000) {
            // 修复RTT打印语句，使用局部变量承接
            int16_t print_angle = controller_instance->servo_motors[0]->present_pos; // 强制从结构体读到局部变量
            float angle_degrees = (float)print_angle * 240.0f / 1000.0f;
            LOGINFO("Servo %d Angle: %.2f degrees (raw: %d)", 
                   (int)controller_instance->servo_motors[0]->id, 
                   angle_degrees,
                   (int)print_angle);
        } else {
            int16_t print_angle = controller_instance->servo_motors[0]->present_pos;
            LOGWARNING("Servo %d returned invalid angle (raw: %d)", 
                      (int)controller_instance->servo_motors[0]->id,
                      (int)print_angle);
        }
    }
}