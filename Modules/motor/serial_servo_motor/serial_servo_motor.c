#include "serial_servo_motor.h"
#include "stdlib.h"
#include "memory.h"
#include "bsp_log.h"

// 重命名全局变量，避免冲突
uint8_t serial_servo_angle_read[6]={0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t serial_servo_angle_write[16]={0x55 ,0x55, 0x08, 0x03, 0x01 ,0xF4 ,0x01 ,0x01 ,0x20 ,0x03,0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t serial_servo_unload[6]={0x55,0x55,0x04,0x14,0x01,0x01};

/*第二版*/
static SerialServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static uint8_t servo_idx = 0; // register servo_idx,是该文件的全局舵机索引,在注册时使用
static USARTInstance *registered_usart_instance = NULL; // 添加已注册的USART实例指针
static void SerialDecodeServo();

// 添加RS485方向控制函数
static void SerialServoSetDirection(uint8_t direction) {
    if (direction == 1) {
        // 设置为发送模式
        HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_SET);
    } else {
        // 设置为接收模式
        HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_SET);
    }
}

// 通过此函数注册一个舵机，重命名函数避免冲突
SerialServoInstance *SerialServoInit(Servo_Init_Config_s *Servo_Init_Config)
{
    static uint8_t is_gpio_initialized = 0;  // 确保GPIO只初始化一次
    
    SerialServoInstance *servo = (SerialServoInstance *)malloc(sizeof(SerialServoInstance));
    if (servo == NULL) {
        LOGERROR("Failed to allocate memory for servo instance");
        return NULL;
    }
    
    // 初始化GPIO引脚（只需执行一次）
    if (!is_gpio_initialized) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        // 使能GPIOE时钟
        __HAL_RCC_GPIOE_CLK_ENABLE();
        
        // 配置SERIAL_SERVO_RX_EN_Pin和SERIAL_SERVO_TX_EN_Pin为输出模式
        GPIO_InitStruct.Pin = SERIAL_SERVO_RX_EN_Pin | SERIAL_SERVO_TX_EN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SERIAL_SERVO_RX_EN_GPIO_Port, &GPIO_InitStruct);
        
        // 初始化时设置为接收模式
        SerialServoSetDirection(0);
        
        is_gpio_initialized = 1;
    }
    
    memset(servo, 0, sizeof(SerialServoInstance));
    USART_Init_Config_s config;
    servo->servo_type = Servo_Init_Config->servo_type;
    
    // 只支持Bus_Servo类型
    if (Servo_Init_Config->servo_type == Bus_Servo)
    {
        // 检查是否已经为这个UART句柄注册了USART实例
        if (registered_usart_instance == NULL || 
            registered_usart_instance->usart_handle != Servo_Init_Config->_handle) {
            // 为这个UART句柄注册USART实例
            config.module_callback = SerialDecodeServo;
            config.recv_buff_size = Servo_MAX_BUFF;
            config.usart_handle = Servo_Init_Config->_handle;
            registered_usart_instance = USARTRegister(&config);
            if (registered_usart_instance == NULL) {
                LOGERROR("Failed to register USART instance");
                free(servo);
                return NULL;
            }
        }
        
        // 使用已注册的USART实例
        servo->usart_instance = registered_usart_instance;
        
        // 初始化串行舵机相关参数
        SerialServoInitInternal(servo);
        servo->servo_id = Servo_Init_Config->servo_id;
        
        // 检查是否超出数组范围
        if (servo_idx >= SERVO_MOTOR_CNT) {
            LOGERROR("Servo motor count exceeds limit");
            free(servo);
            return NULL;
        }
        
        servo_motor_instance[servo_idx++] = servo;
    }
    else
    {
        LOGERROR("Only Bus_Servo type is supported");
        free(servo);
        return NULL;
    }

    return servo;
}

void SerialServoSetAngle(SerialServoInstance *servo, float angle)
{
    // 只处理Bus_Servo类型
    if (servo->servo_type == Bus_Servo)
    {
        serial_servo_angle_write[8] = (uint16_t)angle&0xff;
        serial_servo_angle_write[9] = (uint16_t)angle>>8;
        // 设置为发送模式
        SerialServoSetDirection(1);
        // 短暂延时确保方向切换完成
        HAL_Delay(1);
        USARTSend(servo->usart_instance, serial_servo_angle_write, 16, USART_TRANSFER_DMA);
        
        // 等待发送完成
        uint32_t timeout = 100000; // 超时计数
        while (HAL_UART_GetState(servo->usart_instance->usart_handle) == HAL_UART_STATE_BUSY_TX && timeout--) {
            // 等待发送完成
        }
        
        // 发送完成后短暂延时并恢复为接收模式
        HAL_Delay(2);
        SerialServoSetDirection(0);
    }
}

//@todo 只读取了角度 还有电压，动作是否完成等 且只支持一个串口
static void SerialDecodeServo()
{
    for (uint8_t i = 0; i < servo_idx; i++)
    {
        if (servo_motor_instance[i] != NULL && servo_motor_instance[i]->servo_type == Bus_Servo)
        {
            // 处理串行舵机协议
            // 注意: USARTInstance结构体中没有recv_len成员，我们需要使用recv_buff_size
            for (int j = 0; j < servo_motor_instance[i]->usart_instance->recv_buff_size; j++) {
                uint8_t rx_byte = servo_motor_instance[i]->usart_instance->recv_buff[j];
                if (rx_byte != 0) { // 只处理非零字节，避免处理空数据
                    int result = serial_servo_rx_handler(servo_motor_instance[i], rx_byte);
                    
                    // 检查是否接收完成
                    if (result == 0) {
                        // 解析完成，检查是否是角度读取命令的回复
                        if (servo_motor_instance[i]->rx_frame.elements.command == SERIAL_SERVO_POS_READ) {
                            // 从接收到的数据中提取角度值
                            servo_motor_instance[i]->recv_angle = 
                                (servo_motor_instance[i]->rx_frame.elements.args[1] << 8) |
                                servo_motor_instance[i]->rx_frame.elements.args[0];
                        }
                    }
                }
            }
            
            // 原有的简单处理保留作为兼容
            if (servo_motor_instance[i]->usart_instance->recv_buff[0] == Servo_Frame_First && 
                servo_motor_instance[i]->usart_instance->recv_buff[1] == Servo_Frame_Second)
            {
                if (servo_motor_instance[i]->usart_instance->recv_buff[2] == 21)
                {
                    servo_motor_instance[i]->recv_angle = (servo_motor_instance[i]->usart_instance->recv_buff[6] << 8 | 
                                                         servo_motor_instance[i]->usart_instance->recv_buff[5]);
                }
            }
            
            // 添加调试信息，查看实际接收到的数据
            /*
            LOGDEBUG("Servo %d received data:", servo_motor_instance[i]->servo_id);
            for (int j = 0; j < servo_motor_instance[i]->usart_instance->recv_buff_size; j++) {
                if (servo_motor_instance[i]->usart_instance->recv_buff[j] != 0) {
                    LOGDEBUG("  [%d]: 0x%02X", j, servo_motor_instance[i]->usart_instance->recv_buff[j]);
                }
            }
            */
            
            // 清空接收缓冲区，避免残留数据影响下次解析
            memset(servo_motor_instance[i]->usart_instance->recv_buff, 0, 
                   servo_motor_instance[i]->usart_instance->recv_buff_size);
        }
    }
}

// 新增函数实现
void SerialServoInitInternal(SerialServoInstance *servo)
{
    servo->proc_timeout = 4;
    servo->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
    memset(&servo->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
    memset(&servo->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
    servo->tx_byte_index = 0;
    servo->tx_only = true;
}

static void cmd_frame_init(SerialServoCmdTypeDef *frame, int servo_id, int cmd)
{
    frame->header_1 = Servo_Frame_First;
    frame->header_2 = Servo_Frame_Second;
    frame->elements.servo_id = servo_id;
    frame->elements.command = cmd;
}

static void cmd_frame_complete(SerialServoCmdTypeDef *frame, int args_num)
{
    frame->elements.length = args_num + 3;
    frame->elements.args[args_num] = serial_servo_checksum((uint8_t*)frame);
}

static int serial_servo_tx(SerialServoInstance *servo, SerialServoCmdTypeDef *frame, bool tx_only)
{
    // 打印发送的命令用于调试
    /*
    LOGDEBUG("Sending command to servo %d:", frame->elements.servo_id);
    for (int i = 0; i < frame->elements.length + 2; i++) {
        LOGDEBUG("  [%d]: 0x%02X", i, ((uint8_t*)frame)[i]);
    }
    */
    
    // 设置为发送模式
    SerialServoSetDirection(1);
    // 短暂延时确保方向切换完成
    HAL_Delay(1);
    
    uint16_t len = frame->elements.length + 2;
    USARTSend(servo->usart_instance, (uint8_t*)frame, len, USART_TRANSFER_DMA);
    
    // 等待发送完成
    uint32_t timeout = 100000; // 超时计数
    while (HAL_UART_GetState(servo->usart_instance->usart_handle) == HAL_UART_STATE_BUSY_TX && timeout--) {
        // 等待发送完成
    }
    
    // 如果需要等待回复
    if (!tx_only) {
        // 等待一段时间以便接收回复
        HAL_Delay(10);
        // 恢复为接收模式
        SerialServoSetDirection(0);
    } else {
        // 对于只发送的命令，短暂延时后恢复为接收模式
        HAL_Delay(2);
        SerialServoSetDirection(0);
    }
    
    return 0;
}

/**
 * @brief 重置舵机索引计数器
 *        在多次初始化场景下需要显式重置，避免因残留状态导致后续舵机实例注册失败
 */
void SerialServoResetIndex(void) {
    servo_idx = 0;
    registered_usart_instance = NULL;
    // 清空舵机实例指针数组
    for (int i = 0; i < SERVO_MOTOR_CNT; i++) {
        servo_motor_instance[i] = NULL;
    }
}

void SerialServoSetID(SerialServoInstance *servo, uint32_t old_id, uint32_t new_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, old_id, SERIAL_SERVO_ID_WRITE);
    frame.elements.args[0] = new_id;
    cmd_frame_complete(&frame, 1);
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadID(SerialServoInstance *servo, uint32_t servo_id, uint8_t *ret_servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ID_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    return -1; // 占位返回值
}

void SerialServoSetPosition(SerialServoInstance *servo, uint32_t servo_id, int position, uint32_t duration)
{
    SerialServoCmdTypeDef frame;
    position = position > 1000 ? 1000 : position;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(position);
    frame.elements.args[1] = GET_HIGH_BYTE(position);
    frame.elements.args[2] = GET_LOW_BYTE(duration);
    frame.elements.args[3] = GET_HIGH_BYTE(duration);
    cmd_frame_complete(&frame, 4);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadPosition(SerialServoInstance *servo, uint32_t servo_id, int16_t *position)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_POS_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *position = (int)(*((int16_t*)servo->rx_frame.elements.args));
    // 直接从servo实例中获取角度值
    if (position != NULL) {
        *position = (int16_t)servo->recv_angle;
        return 0;
    }
    return -1;
}

/**
 * @brief 读取舵机角度（增强版）
 * 
 * @param servo 舵机实例指针
 * @param servo_id 舵机ID
 * @param position 返回的角度值指针
 * @return int 0表示成功，-1表示失败
 */
int SerialServoReadPositionEnhanced(SerialServoInstance *servo, uint32_t servo_id, int16_t *position)
{
    if (servo == NULL || position == NULL) {
        return -1;
    }
    
    // 直接从已解析的数据中获取角度值
    *position = (int16_t)servo->recv_angle;
    return 0;
}

void SerialServoStop(SerialServoInstance *servo, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_STOP);
    cmd_frame_complete(&frame, 0);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

void SerialServoSetDeviation(SerialServoInstance *servo, uint32_t servo_id, int new_deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_ADJUST);
    frame.elements.args[0] = (uint8_t) ((int8_t) new_deviation);
    cmd_frame_complete(&frame, 1);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadDeviation(SerialServoInstance *servo, uint32_t servo_id, int8_t *deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *deviation = (int8_t)(servo->rx_frame.elements.args[0]);
    return -1; // 占位返回值
}

void SerialServoSaveDeviation(SerialServoInstance *servo, uint32_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_WRITE);
    cmd_frame_complete(&frame, 0);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

void SerialServoLoadUnload(SerialServoInstance *servo, uint32_t servo_id, uint32_t load)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE);
    frame.elements.args[0] = load;
    cmd_frame_complete(&frame, 1);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

void SerialServoSetAngleLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_WRITE);
    limit_l = limit_l > 1000 ? 1000 : limit_l;
    limit_h = limit_h > 1000 ? 1000 : limit_h;
    uint32_t real_limit_l = limit_l > limit_h ? limit_h : limit_l;
    uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadAngleLimit(SerialServoInstance *servo, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // limit[0] = *((uint16_t*)(&servo->rx_frame.elements.args[0]));
    // limit[1] = *((uint16_t*)(&servo->rx_frame.elements.args[2]));
    return -1; // 占位返回值
}

void SerialServoSetTempLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE);
    frame.elements.args[0] = limit > 100 ? 100 : (uint8_t)limit;
    cmd_frame_complete(&frame, 1);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadTempLimit(SerialServoInstance *servo, uint32_t servo_id, uint8_t *limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *limit = (uint8_t)(servo->rx_frame.elements.args[0]);
    return -1; // 占位返回值
}

int SerialServoReadTemp(SerialServoInstance *servo, uint32_t servo_id, uint8_t *temp)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *temp = (uint8_t)(servo->rx_frame.elements.args[0]);
    return -1; // 占位返回值
}

void SerialServoSetVinLimit(SerialServoInstance *servo, uint32_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_WRITE);
    limit_l = limit_l < 4500 ? 4500 : limit_l;
    limit_h = limit_h > 14000 ? 14000 : limit_h;
    uint32_t real_limit_l  = limit_l > limit_h ? limit_h : limit_l;
    uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.elements.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.elements.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.elements.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.elements.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    // 发送命令
    serial_servo_tx(servo, &frame, true);
}

int SerialServoReadVinLimit(SerialServoInstance *servo, uint32_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // limit[0] = *((uint16_t*)(&servo->rx_frame.elements.args[0]));
    // limit[1] = *((uint16_t*)(&servo->rx_frame.elements.args[2]));
    return -1; // 占位返回值
}

int SerialServoReadVin(SerialServoInstance *servo, uint32_t servo_id, uint16_t *vin)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *vin = ((uint32_t) * ((uint16_t*)servo->rx_frame.elements.args));
    return -1; // 占位返回值
}

int SerialServoReadLoadUnload(SerialServoInstance *servo, uint32_t servo_id, uint8_t* load_unload)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_READ);
    cmd_frame_complete(&frame, 0);
    // 发送命令并等待回复
    serial_servo_tx(servo, &frame, false);
    // 等待并处理回复
    // *load_unload = (uint8_t)(servo->rx_frame.elements.args[0]);
    return -1; // 占位返回值
}

/**
 * @brief 发送测试命令以验证舵机通信
 * 
 * @param servo 舵机实例指针
 * @param servo_id 舵机ID
 * @return int 0表示成功，-1表示失败
 */
int SerialServoSendTestCommand(SerialServoInstance *servo, uint32_t servo_id)
{
    if (servo == NULL) {
        return -1;
    }
    
    // 发送一个简单的ID读取命令来测试通信
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ID_READ);
    cmd_frame_complete(&frame, 0);
    
    LOGINFO("Sending test command to servo %d", servo_id);
    serial_servo_tx(servo, &frame, false);
    
    return 0;
}

/**
 * @brief 主动读取舵机角度
 * 
 * @param servo 舵机实例指针
 * @param servo_id 舵机ID
 * @return int 0表示成功，-1表示失败
 */
int SerialServoRequestAngle(SerialServoInstance *servo, uint32_t servo_id)
{
    if (servo == NULL) {
        return -1;
    }
    
    // 发送角度读取命令
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_POS_READ);
    cmd_frame_complete(&frame, 0);
    
    LOGINFO("Requesting angle from servo %d", servo_id);
    serial_servo_tx(servo, &frame, false);
    
    return 0;
}

int serial_servo_rx_handler(SerialServoInstance *servo, uint8_t rx_byte)
{
    switch (servo->rx_state) {
        case SERIAL_SERVO_RECV_STARTBYTE_1: {
            servo->rx_state = Servo_Frame_First == rx_byte ? SERIAL_SERVO_RECV_STARTBYTE_2 : SERIAL_SERVO_RECV_STARTBYTE_1;
            servo->rx_frame.header_1 = Servo_Frame_First;
            return -1;
        }
        case SERIAL_SERVO_RECV_STARTBYTE_2: {
            servo->rx_state = Servo_Frame_First == rx_byte ? SERIAL_SERVO_RECV_SERVO_ID : SERIAL_SERVO_RECV_STARTBYTE_1;
            servo->rx_frame.header_2 = Servo_Frame_First;
            return -2;
        }
        case SERIAL_SERVO_RECV_SERVO_ID: {
            servo->rx_frame.elements.servo_id = rx_byte;
            servo->rx_state = SERIAL_SERVO_RECV_LENGTH;
            return 1;
        }
        case SERIAL_SERVO_RECV_LENGTH: {
            if(rx_byte > 7) {
                servo->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1; /* 包长度超过允许长度 */
                return -3;
            }
            servo->rx_frame.elements.length = rx_byte;
            servo->rx_state = SERIAL_SERVO_RECV_COMMAND;
            return 2;
        }
        case SERIAL_SERVO_RECV_COMMAND: {
            servo->rx_frame.elements.command = rx_byte;
            servo->rx_args_index = 0;
            servo->rx_state = servo->rx_frame.elements.length == 6 ? SERIAL_SERVO_RECV_CHECKSUM : SERIAL_SERVO_RECV_ARGUMENTS; /* 没有参数的话直接进入校验字段 */
            return 3;
        }
        case SERIAL_SERVO_RECV_ARGUMENTS: {
            servo->rx_frame.elements.args[servo->rx_args_index++] = rx_byte;
            if (servo->rx_args_index + 3 == servo->rx_frame.elements.length) {
                servo->rx_state = SERIAL_SERVO_RECV_CHECKSUM;
            }
            return 4;
        }
        case SERIAL_SERVO_RECV_CHECKSUM: {
            if(serial_servo_checksum((uint8_t*)&servo->rx_frame) != rx_byte) {
                servo->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
                return -99;
            } else {
                servo->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
                return 0; // 接收完成
            }
        }
        default: {
            servo->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
            return -100;
        }
    }
}