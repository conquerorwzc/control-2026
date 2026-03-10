#include "custom_controller.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// USART3 实例声明
static USARTInstance* custom_controller_usart = NULL;

// 微动开关 GPIO 配置（在组件内部创建）
static GPIO_Init_Config_s gpio_init_config_micro_switch = {
    .GPIO_Pin = Micro_switch_Pin,
    .GPIOx = Micro_switch_GPIO_Port,
    .pin_state = GPIO_PIN_RESET,
};

/* ----------------------- 私有函数声明 ----------------------------- */
static float DM_RadianToDegree(float radian);
static void CalibrateMotorZeroPosition(CustomController_t* controller);
static bool CheckMotorOnlineStatus(CustomController_t* controller);
static void MicroSwitchMonitor(CustomController_t* controller);
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
    controller->motors[0].dm_motor = DMMotorInit(&init_config->dm4310_config_1);
    controller->motors[0].dji_motor = NULL;
    //DMMotorCaliEncoder(controller->motors[0].dm_motor);

    // DM4310电机 (索引1)
    controller->motors[1].dm_motor = DMMotorInit(&init_config->dm4310_config_2);
    controller->motors[1].dji_motor = NULL;
    
    // 第一个3508电机 (索引2)
    controller->motors[2].dm_motor = NULL;
    controller->motors[2].dji_motor = DJIMotorInit(&init_config->m3508_config_1);
    
    // 第二个3508电机 (索引3)
    controller->motors[3].dm_motor = NULL;
    controller->motors[3].dji_motor = DJIMotorInit(&init_config->m3508_config_2);
    
    // 2006电机 (索引4)
    controller->motors[4].dm_motor = NULL;
    controller->motors[4].dji_motor = DJIMotorInit(&init_config->m2006_config);
    
    // 初始化角度数据和零点标定数组
    for (int i = 0; i < 5; i++) {
        controller->motor_angles[i] = 0.0f;
    }
    // 初始化 DJI 电机零点标定数组（3 个 DJI 电机：索引 2, 3, 4）
    for (int i = 0; i < 3; i++) {
        controller->dji_zero_total_round[i] = 0;
        controller->dji_zero_ecd[i] = 0;
    }
    // 初始化电机在线状态
    for (int i = 0; i < 5; i++) {
        controller->motor_online_status[i] = false;
    }
    
    // 初始化电机数据
    for (int i = 0; i < 5; i++) {
        controller->motor_data[i].id = i + 1;  // 电机 ID 为 1, 2, 3, 4
        controller->motor_data[i].present_pos = 0;
        controller->motor_data[i].current_angle = 0.0f;
        controller->motor_data[i].is_online = 0;
    }
        
    // 初始化 USART 实例，使用 USART1
    if (custom_controller_usart == NULL) {
        USART_Init_Config_s usart_config = {0};
        usart_config.recv_buff_size = 256;
        extern UART_HandleTypeDef huart1;  // 声明外部USART3句柄
        usart_config.usart_handle = &huart1;
        usart_config.module_callback = NULL;  // 如果需要接收回调可以设置
        custom_controller_usart = USARTRegister(&usart_config);
    }
    controller->usart_instance = custom_controller_usart;
        
    // 在组件内部创建微动开关 GPIO 实例
    controller->micro_switch_gpio = GPIORegister(&gpio_init_config_micro_switch);
    
    // 检查 GPIO 实例是否有效
    if (controller->micro_switch_gpio == NULL) {
        LOGERROR("CustomController: micro_switch_gpio is NULL");
    }
    
    // 初始化夹爪状态
    controller->gripper_opened = false;
        
    controller->is_initialized = true;
    controller->is_active = true;

    // 等待电机数据稳定
    osDelay(100);
    
    // 首次上电校准
    CalibrateMotorZeroPosition(controller);
    
    LOGINFO("CustomController: Initialized with 5 motors");
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
    
    // 读取五个电机的角度值并应用零位偏移
    if (controller->motors[0].dm_motor != NULL) {
        // DM 电机角度转换：弧度转角度 (DM 电机已有固定零点，无需额外偏移)
        float raw_angle = DM_RadianToDegree(controller->motors[0].dm_motor->measure.total_angle);
        controller->motor_angles[0] = raw_angle;
    }
    if (controller->motors[1].dm_motor != NULL) {
        // DM 电机角度转换：弧度转角度
        float raw_angle = DM_RadianToDegree(controller->motors[1].dm_motor->measure.total_angle);
        controller->motor_angles[1] = raw_angle;
    }
    if (controller->motors[2].dji_motor != NULL) {
        // DJI 电机 1: 使用校准后的 total_round 和 ecd 计算角度
        int32_t calibrated_total_round = controller->motors[2].dji_motor->measure.total_round - 
                                         controller->dji_zero_total_round[0];
        uint16_t calibrated_ecd = controller->motors[2].dji_motor->measure.ecd - 
                                  controller->dji_zero_ecd[0];
        float calibrated_angle = calibrated_total_round * 360.0f + calibrated_ecd * 0.0439453125f;
        controller->motor_angles[2] = -calibrated_angle;  // 反向
    }
    if (controller->motors[3].dji_motor != NULL) {
        // DJI 电机 2: 使用校准后的 total_round 和 ecd 计算角度
        int32_t calibrated_total_round = controller->motors[3].dji_motor->measure.total_round - 
                                         controller->dji_zero_total_round[1];
        uint16_t calibrated_ecd = controller->motors[3].dji_motor->measure.ecd - 
                                  controller->dji_zero_ecd[1];
        float calibrated_angle = calibrated_total_round * 360.0f + calibrated_ecd * 0.0439453125f;
        controller->motor_angles[3] = -calibrated_angle;  // 反向
    }
    if (controller->motors[4].dji_motor != NULL) {
        // DJI 电机 3: 使用校准后的 total_round 和 ecd 计算角度
        int32_t calibrated_total_round = controller->motors[4].dji_motor->measure.total_round - 
                                         controller->dji_zero_total_round[2];
        uint16_t calibrated_ecd = controller->motors[4].dji_motor->measure.ecd - 
                                  controller->dji_zero_ecd[2];
        float calibrated_angle = calibrated_total_round * 360.0f + calibrated_ecd * 0.0439453125f;
        controller->motor_angles[4] = calibrated_angle;  // 正向
    }

    // 更新电机数据用于发送
    CustomController_UpdateMotorData(controller);
    
    // 监控微动开关状态，控制夹爪
    MicroSwitchMonitor(controller);
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
    if (controller == NULL || !controller->is_initialized || motor_index >= 5) {
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
    
    // DM 电机数据 - 每个电机占用 2 字节（仅角度值）
    // 索引 0, 1 对应两个 DM 电机
    for (int i = 0; i < 2; i++) {
        // DM 电机角度值，放大 100 倍存储 (int16_t, 小端格式)
        int16_t angle_value = (int16_t)(controller->motor_angles[i] * 100.0f);
        controller_data[1 + i*2] = angle_value & 0xFF;
        controller_data[2 + i*2] = (angle_value >> 8) & 0xFF;
    }
    
    // DJI 电机数据 - 每个电机占用 6 字节（仅 total_round + ecd）
    // 索引 2, 3, 4 对应三个 DJI 电机
    for (int i = 0; i < 3; i++) {
        int dji_index = i + 2;  // DJI 电机在数组中的索引
            
        // 发送校准后的 total_round 和 ecd（已考虑方向映射）
        int32_t calibrated_total_round = 0;
        uint16_t calibrated_ecd = 0;
        
        if (controller->motors[dji_index].dji_motor != NULL) {
            calibrated_total_round = controller->motors[dji_index].dji_motor->measure.total_round - 
                                     controller->dji_zero_total_round[i];
            calibrated_ecd = controller->motors[dji_index].dji_motor->measure.ecd - 
                            controller->dji_zero_ecd[i];
            
            // 根据方向映射处理数据：索引 2 和 3 反向，索引 4 正向
            if (dji_index == 2 || dji_index == 3) {
                // 反向电机：对 total_round 和 ecd 取负
                if (calibrated_ecd == 0) {
                    calibrated_total_round = -calibrated_total_round;
                    calibrated_ecd = 0;
                } else {
                    calibrated_total_round = -calibrated_total_round - 1;
                    calibrated_ecd = 8192 - calibrated_ecd;
                }
            }
            // 索引 4 (dji_index == 4) 保持正向，无需处理
        }
        
        // 小端格式发送 total_round (4 字节)
        controller_data[5 + i*6] = calibrated_total_round & 0xFF;
        controller_data[6 + i*6] = (calibrated_total_round >> 8) & 0xFF;
        controller_data[7 + i*6] = (calibrated_total_round >> 16) & 0xFF;
        controller_data[8 + i*6] = (calibrated_total_round >> 24) & 0xFF;
        
        // 小端格式发送 ecd (2 字节)
        controller_data[9 + i*6] = calibrated_ecd & 0xFF;
        controller_data[10 + i*6] = (calibrated_ecd >> 8) & 0xFF;
    }
    
    // 夹爪状态 - 放在电机数据之后（第 23 字节）
    // 0: 关闭，1: 打开
    controller_data[23] = controller->gripper_opened ? 1 : 0;
    
    // 发送数据包
    uint16_t packed_length;
    uint8_t *packed_data = custom_controller_protocol_pack(CMD_ID_CUSTOM_CONTROLLER, controller_data, sizeof(controller_data), &packed_length);
    
    if (packed_data != NULL && packed_length > 0 && controller->usart_instance != NULL) {
        // 通过 UART 发送打包后的数据
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
    
    // 更新所有电机的角度数据（5 个电机）
    for (int i = 0; i < 5; i++) {
        controller->motor_data[i].current_angle = controller->motor_angles[i];
        controller->motor_data[i].present_pos = (int16_t)controller->motor_angles[i];
        controller->motor_data[i].is_online = 1;
    }
}

/* ----------------------- 私有函数实现 ----------------------------- */

/**
 * @brief 监控微动开关状态，控制夹爪打开/关闭
 * @param controller 控制器实例
 */
static void MicroSwitchMonitor(CustomController_t* controller)
{
    static GPIO_PinState last_switch_state = GPIO_PIN_SET;   // 记录上一次状态
    static uint32_t last_debounce_time = 0;                  // 上次消抖时间戳
    static bool trigger_lock = false;                        // 触发锁，防止一次按压中重复触发
    const uint32_t debounce_delay_ms = 30;                   // 消抖延时 30ms
    
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    // 检查 GPIO 实例是否有效
    if (controller->micro_switch_gpio == NULL) {
        return;
    }
    
    // 读取当前微动开关状态
    GPIO_PinState switch_state = GPIORead(controller->micro_switch_gpio);
    
    // 如果状态发生变化，更新消抖时间戳
    if (switch_state != last_switch_state) {
        last_debounce_time = DWT_GetTimeline_ms();
    }
    
    // 更新上一次状态
    last_switch_state = switch_state;
    
    // 检查是否超过消抖时间
    if ((DWT_GetTimeline_ms() - last_debounce_time) > debounce_delay_ms) {
        // 检测下降沿（高电平 -> 低电平）- 按下时触发
        if (last_switch_state == GPIO_PIN_RESET && !trigger_lock) {
            // 切换夹爪状态
            controller->gripper_opened = !controller->gripper_opened;
            trigger_lock = true;  // 锁定，防止本次按压中重复触发
        }
        // 检测上升沿（低电平 -> 高电平）- 松开时重置锁
        else if (last_switch_state == GPIO_PIN_SET) {
            trigger_lock = false;  // 解锁，允许下次触发
        }
    }
}

/**
 * @brief DM 电机弧度转角度
 * @param radian 弧度值
 * @return float 角度值
 */
static float DM_RadianToDegree(float radian)
{
    // 弧度转角度：1弧度 = 180/π 度
    return radian * 180.0f / 3.14159265359f;
}

/**
 * @brief 校准电机零位，记录上电时的 total_round 和 ecd
 * @param controller 控制器实例
 */
static void CalibrateMotorZeroPosition(CustomController_t* controller)
{
    if (controller == NULL || !controller->is_initialized) {
        return;
    }
    
    LOGINFO("CustomController: Starting zero position calibration...");
    
    // DM 电机已有固定零点，无需额外标定
    // 仅对三个 DJI 电机进行零点标定（索引 2, 3, 4）
    
    // DJI 电机 1 (索引 2)
    if (controller->motors[2].dji_motor != NULL) {
        controller->dji_zero_total_round[0] = controller->motors[2].dji_motor->measure.total_round;
        controller->dji_zero_ecd[0] = controller->motors[2].dji_motor->measure.ecd;
        controller->motor_angles[2] = 0.0f;
        controller->motor_online_status[2] = true;
        LOGINFO("DJI Motor 1 zero calibrated: total_round=%ld, ecd=%u", 
                controller->dji_zero_total_round[0], controller->dji_zero_ecd[0]);
    }
    
    // DJI 电机 2 (索引 3)
    if (controller->motors[3].dji_motor != NULL) {
        controller->dji_zero_total_round[1] = controller->motors[3].dji_motor->measure.total_round;
        controller->dji_zero_ecd[1] = controller->motors[3].dji_motor->measure.ecd;
        controller->motor_angles[3] = 0.0f;
        controller->motor_online_status[3] = true;
        LOGINFO("DJI Motor 2 zero calibrated: total_round=%ld, ecd=%u", 
                controller->dji_zero_total_round[1], controller->dji_zero_ecd[1]);
    }
    
    // DJI 电机 3 (索引 4)
    if (controller->motors[4].dji_motor != NULL) {
        controller->dji_zero_total_round[2] = controller->motors[4].dji_motor->measure.total_round;
        controller->dji_zero_ecd[2] = controller->motors[4].dji_motor->measure.ecd;
        controller->motor_angles[4] = 0.0f;
        controller->motor_online_status[4] = true;
        LOGINFO("DJI Motor 3 zero calibrated: total_round=%ld, ecd=%u", 
                controller->dji_zero_total_round[2], controller->dji_zero_ecd[2]);
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
    
    // 检查 DM4310 电机 1 (索引 0)
    if (controller->motors[0].dm_motor != NULL) {
        bool current_online = (controller->motors[0].dm_motor->measure.state == 0);  // 假设 state=0 表示在线
        if (!controller->motor_online_status[0] && current_online) {
            // 电机从离线变为在线，需要重新校准
            need_recalibration = true;
            LOGINFO("DM4310 motor 1 reconnected, triggering recalibration");
        }
        controller->motor_online_status[0] = current_online;
    }
    
    // 检查 DM4310 电机 2 (索引 1)
    if (controller->motors[1].dm_motor != NULL) {
        bool current_online= (controller->motors[1].dm_motor->measure.state == 0);
        if (!controller->motor_online_status[1] && current_online) {
            need_recalibration = true;
            LOGINFO("DM4310 motor 2 reconnected, triggering recalibration");
        }
        controller->motor_online_status[1] = current_online;
    }
    
    // 检查 M3508 电机 1 (索引 2)
    if (controller->motors[2].dji_motor != NULL) {
        bool current_online = (controller->motors[2].dji_motor->daemon->temp_count > 0);  // 通过 daemon 计数判断
        if (!controller->motor_online_status[2] && current_online) {
            need_recalibration = true;
            LOGINFO("M3508 motor 1 reconnected, triggering recalibration");
        }
        controller->motor_online_status[2] = current_online;
    }
    
    // 检查 M3508 电机 2 (索引 3)
    if (controller->motors[3].dji_motor != NULL) {
        bool current_online = (controller->motors[3].dji_motor->daemon->temp_count > 0);
        if (!controller->motor_online_status[3] && current_online) {
            need_recalibration = true;
            LOGINFO("M3508 motor 2 reconnected, triggering recalibration");
        }
        controller->motor_online_status[3] = current_online;
    }
    
    // 检查 M2006 电机 (索引 4)
    if (controller->motors[4].dji_motor != NULL) {
        bool current_online = (controller->motors[4].dji_motor->daemon->temp_count > 0);
        if (!controller->motor_online_status[4] && current_online) {
            need_recalibration = true;
            LOGINFO("M2006 motor reconnected, triggering recalibration");
        }
        controller->motor_online_status[4] = current_online;
    }
    
    return need_recalibration;
}
