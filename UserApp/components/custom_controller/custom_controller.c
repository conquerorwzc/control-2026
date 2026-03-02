#include "custom_controller.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// USART3实例声明
static USARTInstance* custom_controller_usart = NULL;

/* ----------------------- 私有函数声明 ----------------------------- */
static float DM_RadianToDegree(float radian);
static void CalibrateMotorZeroPosition(CustomController_t* controller);
static bool CheckMotorOnlineStatus(CustomController_t* controller);
static float VoltageToAngle(float voltage, const Potentiometer_Config_s* config);
static void InitPotentiometer(CustomController_t* controller, const Potentiometer_Config_s* pot_config);

/* ----------------------- 公共函数实现 ----------------------------- */

/**
 * @brief 初始化自定义控制器
 * @param init_config 初始化配置
 * @return CustomController_t* 控制器实例指针
 */
CustomController_t* CustomControllerInit(CustomController_Init_Config_s* init_config)
{
    if (init_config == NULL) {
        LOGERROR("CustomController: Init config is NULL");
        return NULL;
    }
    
    // 分配内存
    CustomController_t* controller = (CustomController_t*)zmalloc(sizeof(CustomController_t));
    if (controller == NULL) {
        LOGERROR("CustomController: Memory allocation failed");
        return NULL;
    }
    
    // 初始化电机
    // DM4310电机 (索引0)
    controller->motors[0].dm_motor = DMMotorInit(&init_config->dm4310_config);
    controller->motors[0].dji_motor = NULL;
    
    // 第一个3508电机 (索引1)
    controller->motors[1].dm_motor = NULL;
    controller->motors[1].dji_motor = DJIMotorInit(&init_config->m3508_config_1);
    
    // 第二个3508电机 (索引2)
    controller->motors[2].dm_motor = NULL;
    controller->motors[2].dji_motor = DJIMotorInit(&init_config->m3508_config_2);
    
    // 2006电机 (索引3)
    controller->motors[3].dm_motor = NULL;
    controller->motors[3].dji_motor = DJIMotorInit(&init_config->m2006_config);
    
    // 初始化角度数据
    for (int i = 0; i < 4; i++) {
        controller->motor_angles[i] = 0.0f;
        controller->zero_offset[i] = 0.0f;  // 初始化零位偏移数组
        controller->motor_online_status[i] = false;  // 初始化在线状态
    }
    
    // 初始化电机数据
    for (int i = 0; i < 4; i++) {
        controller->motor_data[i].id = i + 1;  // 电机ID为1, 2, 3, 4
        controller->motor_data[i].present_pos = 0;
        controller->motor_data[i].current_angle = 0.0f;
        controller->motor_data[i].is_online = 0;
    }
    
    // 初始化电位器
    InitPotentiometer(controller, &init_config->pot_config);
    
    // 初始化USART实例，使用USART1
    if (custom_controller_usart == NULL) {
        USART_Init_Config_s usart_config = {0};
        usart_config.recv_buff_size = 256;
        extern UART_HandleTypeDef huart1;  // 声明外部USART3句柄
        usart_config.usart_handle = &huart1;
        usart_config.module_callback = NULL;  // 如果需要接收回调可以设置
        custom_controller_usart = USARTRegister(&usart_config);
    }
    controller->usart_instance = custom_controller_usart;
    
    controller->is_initialized = true;
    controller->is_active = true;
    
    // 等待电机数据稳定
    osDelay(100);
    
    // 首次上电校准
    CalibrateMotorZeroPosition(controller);
    
    LOGINFO("CustomController: Initialized with 4 motors and potentiometer");
    return controller;
}

/**
 * @brief 控制器主任务函数
 * @param controller 控制器实例
 */
void CustomControllerTask(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    // 检测电机在线状态变化，触发重新校准
    bool need_recalibration = CheckMotorOnlineStatus(controller);
    if (need_recalibration) {
        LOGINFO("CustomController: Motor reconnected, recalibrating zero position...");
        osDelay(100);  // 等待电机稳定
        CalibrateMotorZeroPosition(controller);
    }
    
    // 读取四个电机的角度值并应用零位偏移
    if (controller->motors[0].dm_motor != NULL) {
        // DM电机角度转换：弧度转角度，并减去零位偏移
        float raw_angle = DM_RadianToDegree(controller->motors[0].dm_motor->measure.total_angle);
        controller->motor_angles[0] = raw_angle - controller->zero_offset[0];
        
        // 直接调用4310电机编码器零点标定
        static bool first_calibration_done = false;
        if (!first_calibration_done) {
            DMMotorCaliEncoder(controller->motors[0].dm_motor);
            first_calibration_done = true;
        }
    }
    if (controller->motors[1].dji_motor != NULL) {
        float raw_angle = controller->motors[1].dji_motor->measure.total_angle;
        controller->motor_angles[1] = -(raw_angle - controller->zero_offset[1]);
    }
    if (controller->motors[2].dji_motor != NULL) {
        float raw_angle = controller->motors[2].dji_motor->measure.total_angle;
        controller->motor_angles[2] = raw_angle - controller->zero_offset[2];
    }
    if (controller->motors[3].dji_motor != NULL) {
        float raw_angle = controller->motors[3].dji_motor->measure.total_angle;
        controller->motor_angles[3] = raw_angle - controller->zero_offset[3];
    }
    
    // 更新电位器数据
    CustomController_UpdatePotData(controller);
    
    // 更新电机数据用于发送
    CustomController_UpdateMotorData(controller);
}

/**
 * @brief 获取指定电机角度
 * @param controller 控制器实例
 * @param motor_index 电机索引(0-3)
 * @return float 电机角度
 */
float CustomControllerGetMotorAngle(const CustomController_t* controller, 
                                   uint8_t motor_index)
{
    if (controller == NULL || !controller->is_initialized || motor_index >= 4) {
        return 0.0f;
    }
    return controller->motor_angles[motor_index];
}

/**
 * @brief 获取电位器角度
 * @param controller 控制器实例
 * @return float 电位器角度
 */
float CustomControllerGetPotAngle(const CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized || !controller->potentiometer.is_initialized) {
        return 0.0f;
    }
    return controller->potentiometer.current_angle;
}

/**
 * @brief 发送自定义控制器的所有数据
 * @param controller 控制器实例
 */
void CustomController_SendAllData(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    // 直接使用CustomController_t结构体中的数据填充发送缓冲区
    uint8_t controller_data[64] = {0};
    
    // 数据包类型标识
    controller_data[0] = 0x20; // 控制器数据包标识
    
    // 电机数据 - 每个电机占用5个字节（ID + 2字节角度 + 1字节预留 + 1字节在线状态）
    for (int i = 0; i < 4; i++) {
        controller_data[1 + i*5] = controller->motor_data[i].id;  // 电机ID
        
        // 电机角度值，放大100倍存储
        int16_t angle_value = (int16_t)(controller->motor_data[i].current_angle * 100.0f);
        controller_data[2 + i*5] = angle_value & 0xFF;
        controller_data[3 + i*5] = (angle_value >> 8) & 0xFF;
        
        // 预留字节（原扭矩状态位置）
        controller_data[4 + i*5] = 0;
        // 电机在线状态
        controller_data[5 + i*5] = controller->motor_data[i].is_online;
    }
    
    // 电位器数据 - 占用3个字节（2字节角度 + 1字节电压）
    int16_t pot_angle_value = (int16_t)(controller->potentiometer.current_angle * 100.0f);
    controller_data[21] = pot_angle_value & 0xFF;  // 电位器角度低字节
    controller_data[22] = (pot_angle_value >> 8) & 0xFF;  // 电位器角度高字节
    
    uint8_t pot_voltage_value = (uint8_t)(controller->potentiometer.current_voltage * 50.0f); // 0-3.3V映射到0-255
    controller_data[23] = pot_voltage_value;  // 电位器电压值
    
    // 发送数据包
    uint16_t packed_length;
    uint8_t *packed_data = custom_controller_protocol_pack(CMD_ID_CUSTOM_CONTROLLER, controller_data, sizeof(controller_data), &packed_length);
    
    if (packed_data != NULL && packed_length > 0 && controller->usart_instance != NULL) {
        // 通过UART发送打包后的数据
        USARTSend(controller->usart_instance, packed_data, packed_length, USART_TRANSFER_DMA);
    }
}

/**
 * @brief 设置自定义控制器的USART通信实例
 * @param controller 控制器实例
 * @param usart_instance USART实例指针
 */
void CustomController_SetUsartInstance(CustomController_t* controller, USARTInstance* usart_instance)
{
    if (controller != NULL) {
        controller->usart_instance = usart_instance;
    }
}

/**
 * @brief 更新电机数据用于发送
 * @param controller 控制器实例
 */
void CustomController_UpdateMotorData(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    // 更新所有电机的角度数据
    for (int i = 0; i < 4; i++) {
        controller->motor_data[i].current_angle = controller->motor_angles[i];
        controller->motor_data[i].present_pos = (int16_t)controller->motor_angles[i];
        controller->motor_data[i].is_online = 1;
    }
}

/**
 * @brief 更新电位器数据
 * @param controller 控制器实例
 */
void CustomController_UpdatePotData(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized || !controller->potentiometer.is_initialized) {
        return;
    }
    
    // 读取ADC电压值
    controller->potentiometer.current_voltage = ADCGetVoltage(controller->potentiometer.adc_instance);
    
    // 电压转换为角度（修复：使用保存在实例中的正确配置）
    controller->potentiometer.current_angle = VoltageToAngle(
        controller->potentiometer.current_voltage, 
        &controller->potentiometer.config  // 使用持久化的配置参数
    ) - 101.0f;
}

/* ----------------------- 私有函数实现 ----------------------------- */

/**
 * @brief DM电机弧度转角度
 * @param radian 弧度值
 * @return float 角度值
 */
static float DM_RadianToDegree(float radian)
{
    // 弧度转角度：1弧度 = 180/π 度
    return radian * 180.0f / 3.14159265359f;
}

/**
 * @brief 校准电机零位，将当前角度设为0度
 * @param controller 控制器实例
 */
static void CalibrateMotorZeroPosition(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    LOGINFO("CustomController: Starting zero position calibration...");
    
    // 读取当前各电机的实际角度作为零位参考
    float current_angles[4] = {0.0f};
    
    // 获取当前角度作为零位基准
    if (controller->motors[0].dm_motor != NULL) {
        current_angles[0] = DM_RadianToDegree(controller->motors[0].dm_motor->measure.total_angle);
    }
    if (controller->motors[1].dji_motor != NULL) {
        current_angles[1] = controller->motors[1].dji_motor->measure.total_angle;
    }
    if (controller->motors[2].dji_motor != NULL) {
        current_angles[2] = controller->motors[2].dji_motor->measure.total_angle;
    }
    if (controller->motors[3].dji_motor != NULL) {
        current_angles[3] = controller->motors[3].dji_motor->measure.total_angle;
    }
    
    // 设置零位偏移值（当前角度即为偏移量）
    for (int i = 0; i < 4; i++) {
        controller->zero_offset[i] = current_angles[i];
        controller->motor_angles[i] = 0.0f;  // 初始化为0
        controller->motor_online_status[i] = true;  // 标记为在线
        LOGINFO("Motor %d zero offset set to: %.2f degrees", i+1, current_angles[i]);
    }
    
    LOGINFO("CustomController: Zero position calibration completed");
}

/**
 * @brief 检测电机在线状态变化
 * @param controller 控制器实例
 * @return bool 是否需要重新校准
 */
static bool CheckMotorOnlineStatus(CustomController_t* controller)
{
    bool need_recalibration = false;
    
    // 检查DM4310电机
    if (controller->motors[0].dm_motor != NULL) {
        bool current_online = (controller->motors[0].dm_motor->measure.state == 0);  // 假设state=0表示在线
        if (!controller->motor_online_status[0] && current_online) {
            // 电机从离线变为在线，需要重新校准
            need_recalibration = true;
            LOGINFO("DM4310 motor reconnected, triggering recalibration");
        }
        controller->motor_online_status[0] = current_online;
    }
    
    // 检查3508电机1
    if (controller->motors[1].dji_motor != NULL) {
        bool current_online = (controller->motors[1].dji_motor->daemon->temp_count > 0);  // 通过daemon计数判断
        if (!controller->motor_online_status[1] && current_online) {
            need_recalibration = true;
            LOGINFO("M3508 motor 1 reconnected, triggering recalibration");
        }
        controller->motor_online_status[1] = current_online;
    }
    
    // 检查3508电机2
    if (controller->motors[2].dji_motor != NULL) {
        bool current_online = (controller->motors[2].dji_motor->daemon->temp_count > 0);
        if (!controller->motor_online_status[2] && current_online) {
            need_recalibration = true;
            LOGINFO("M3508 motor 2 reconnected, triggering recalibration");
        }
        controller->motor_online_status[2] = current_online;
    }
    
    // 检查2006电机
    if (controller->motors[3].dji_motor != NULL) {
        bool current_online = (controller->motors[3].dji_motor->daemon->temp_count > 0);
        if (!controller->motor_online_status[3] && current_online) {
            need_recalibration = true;
            LOGINFO("M2006 motor reconnected, triggering recalibration");
        }
        controller->motor_online_status[3] = current_online;
    }
    
    return need_recalibration;
}

/**
 * @brief 电压值转换为角度值
 * @param voltage 电压值(V)
 * @param config 电位器配置
 * @return float 角度值(度)
 */
static float VoltageToAngle(float voltage, const Potentiometer_Config_s* config)
{
    // 限制电压在有效范围内
    if (voltage < config->min_voltage) voltage = config->min_voltage;
    if (voltage > config->max_voltage) voltage = config->max_voltage;
    
    // 线性映射：(voltage - min_voltage) / (max_voltage - min_voltage) * (max_angle - min_angle) + min_angle
    float ratio = (voltage - config->min_voltage) / (config->max_voltage - config->min_voltage);
    return ratio * (config->max_angle - config->min_angle) + config->min_angle;
}

/**
 * @brief 初始化电位器
 * @param controller 控制器实例
 * @param pot_config 电位器配置
 */
static void InitPotentiometer(CustomController_t* controller, const Potentiometer_Config_s* pot_config)
{
    // 初始化电位器实例
    controller->potentiometer.is_initialized = false;
    controller->potentiometer.current_voltage = 0.0f;
    controller->potentiometer.current_angle = 0.0f;
    
    // 将配置参数拷贝到实例中（关键修改：持久化配置）
    controller->potentiometer.config = *pot_config;
    
    // 注册ADC实例 (ADC1_IN14, 16位单端)
    extern ADC_HandleTypeDef hadc1;  // 声明外部ADC1句柄
    ADC_Init_Config_s adc_config = {0};
    adc_config.hadc = &hadc1;
    adc_config.channel = ADC_CHANNEL_14;  // ADC1_IN14
    adc_config.mode = ADC_MODE_POLLING;   // 轮询模式
    adc_config.vref = 3.3f;               // 参考电压3.3V (与电位器供电电压一致)
    adc_config.alpha = pot_config->filter_alpha;  // 滤波系数
    
    controller->potentiometer.adc_instance = ADCRegister(&adc_config);
    if (controller->potentiometer.adc_instance != NULL) {
        controller->potentiometer.is_initialized = true;
        LOGINFO("Potentiometer ADC (16-bit, IN14) initialized successfully");
    } else {
        LOGERROR("Failed to initialize potentiometer ADC");
    }
}