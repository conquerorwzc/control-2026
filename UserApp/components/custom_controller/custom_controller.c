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
    }
    
    // 初始化电机数据
    for (int i = 0; i < 4; i++) {
        controller->motor_data[i].id = i + 1;  // 电机ID为1, 2, 3, 4
        controller->motor_data[i].present_pos = 0;
        controller->motor_data[i].current_angle = 0.0f;
        controller->motor_data[i].is_online = 0;
    }
    
    // 初始化USART实例，使用USART6
    if (custom_controller_usart == NULL) {
        USART_Init_Config_s usart_config = {0};
        usart_config.recv_buff_size = 256;
        extern UART_HandleTypeDef huart6;  // 声明外部USART6句柄
        usart_config.usart_handle = &huart6;
        usart_config.module_callback = NULL;  // 如果需要接收回调可以设置
        custom_controller_usart = USARTRegister(&usart_config);
    }
    controller->usart_instance = custom_controller_usart;
    
    controller->is_initialized = true;
    controller->is_active = true;
    
    // 等待电机数据稳定
    osDelay(100);
    
    // 执行零位校准
    CalibrateMotorZeroPosition(controller);
    
    LOGINFO("CustomController: Initialized with 4 motors, zero position calibrated");
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
    
    // 读取四个电机的角度值并应用零位偏移
    if (controller->motors[0].dm_motor != NULL) {
        // DM电机角度转换：弧度转角度，并减去零位偏移
        float raw_angle = DM_RadianToDegree(controller->motors[0].dm_motor->measure.total_angle);
        controller->motor_angles[0] = raw_angle - controller->zero_offset[0];
    }
    if (controller->motors[1].dji_motor != NULL) {
        float raw_angle = controller->motors[1].dji_motor->measure.total_angle;
        controller->motor_angles[1] = raw_angle - controller->zero_offset[1];
    }
    if (controller->motors[2].dji_motor != NULL) {
        float raw_angle = controller->motors[2].dji_motor->measure.total_angle;
        controller->motor_angles[2] = raw_angle - controller->zero_offset[2];
    }
    if (controller->motors[3].dji_motor != NULL) {
        float raw_angle = controller->motors[3].dji_motor->measure.total_angle;
        controller->motor_angles[3] = raw_angle - controller->zero_offset[3];
    }
    
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
    
    // 发送数据包
    uint16_t packed_length;
    uint8_t *packed_data = custom_controller_protocol_pack(CMD_ID_CUSTOM_CONTROLLER, controller_data, sizeof(controller_data), &packed_length);
    
    if (packed_data != NULL && packed_length > 0 && controller->usart_instance != NULL) {
        // 直接发送，双缓冲区机制确保数据安全
        // 当DMA正在发送一个缓冲区时，CPU可以安全地准备下一个缓冲区
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
        // 假设电机都是在线的
        controller->motor_data[i].is_online = 1;
    }
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
        LOGINFO("Motor %d zero offset set to: %.2f degrees", i+1, current_angles[i]);
    }
    
    LOGINFO("CustomController: Zero position calibration completed");
}