//
// Created by ASUS on 2025/11/23.
//
#include "navigator.h"
#include "bsp_usart.h"
#include "crc_func.h"
#include "bsp_dwt.h"

static navigator_send_t send_data;
static USARTInstance *navigator_usart_instance ;
static navigator_recv_t recv_data;
// 接收状态变量
static recv_state_t recv_state = RECV_STATE_SOF;
static uint8_t recv_buffer[NAVIGATOR_RECV_SIZE];
static uint16_t recv_index = 0;
static uint16_t expected_data_len = 0;
static uint8_t current_packet_id = 0;
static uint32_t last_packet_time = 0;
// 缓冲区最大尺寸
#define BUFFER_MAX_SIZE    256
static uint8_t internal_tx_buffer[BUFFER_MAX_SIZE];

static uint8_t* protocol_packed(const uint8_t* data_ptr, uint32_t time_stamp, uint8_t data_len, uint8_t data_id, uint8_t* tx_buff, uint16_t* tx_buff_len)
{
  // 1. 参数有效性检查
  if (tx_buff == NULL || tx_buff_len == NULL) {
    return NULL;
  }

  // 2. 计算总帧长并检查缓冲区是否足够
  uint16_t total_frame_len = PROTOCOL_HEADER_LEN + 4 + data_len + 2;
  if (total_frame_len > BUFFER_MAX_SIZE) {
    *tx_buff_len = 0;
    return NULL;
  }

  // 3. 填充帧头
  uint16_t current_index = 0;
  tx_buff[current_index++] = PROTOCOL_SOF;                  // sof
  tx_buff[current_index++] = data_len;                      // len
  tx_buff[current_index++] = data_id;                       // id
  tx_buff[current_index++] = get_CRC8_check_sum(&tx_buff[0], 3, PROTOCOL_CRC8_INIT); // crc

  // 4. 填充时间戳 (小端模式)
  tx_buff[current_index++] = (time_stamp >> 0)  & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 8)  & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 16) & 0xFF;
  tx_buff[current_index++] = (time_stamp >> 24) & 0xFF;

  // 5. 填充数据段
  if (data_ptr != NULL && data_len > 0) {
    memcpy(&tx_buff[current_index], data_ptr, data_len);
    current_index += data_len;
  }

  // 6. 计算并填充帧尾CRC16
  uint16_t checksum_len = PROTOCOL_HEADER_LEN + 4 + data_len;
  uint16_t frame_crc16 = get_CRC16_check_sum(&tx_buff[0], checksum_len, PROTOCOL_CRC16_INIT);
  tx_buff[current_index++] = frame_crc16 & 0xFF;
  tx_buff[current_index++] = (frame_crc16 >> 8) & 0xFF;

  // 7. 设置最终帧长并返回
  *tx_buff_len = total_frame_len;
  return tx_buff;
}

uint8_t *protocol_pack(uint32_t time_stamp, const uint8_t *data, uint8_t data_len, uint8_t data_id, uint16_t *packed_length) {
  // 调用核心打包函数，使用静态的 internal_tx_buffer
  return protocol_packed(data, time_stamp, data_len, data_id, internal_tx_buffer, packed_length);
}

uint8_t protocol_send(UART_HandleTypeDef* huart, uint32_t time_stamp, const uint8_t* data_ptr, uint8_t data_len, uint8_t data_id, uint32_t timeout) {
  if (huart == NULL) {
    return 0;
  }

  uint16_t packed_length = 0;
  uint8_t local_buffer[BUFFER_MAX_SIZE];  // 使用局部缓冲区

  // 1. 打包到局部缓冲区
  uint8_t *packed_data = protocol_packed(data_ptr, time_stamp, data_len, data_id, local_buffer, &packed_length);

  // 2. 检查打包是否成功
  if (packed_data == NULL || packed_length == 0) {
    return 0;
  }

  // 3. 使用DMA传输局部缓冲区
  HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(huart, packed_data, packed_length);
  if (hal_status != HAL_OK) {
    return 0;
  }

  // 4. 等待DMA传输完成
  uint32_t start_tick = HAL_GetTick();
  while (huart->gState == HAL_UART_STATE_BUSY_TX) {
    if ((HAL_GetTick() - start_tick) > timeout) {
      HAL_UART_DMAStop(huart);
      return 0;
    }
    osDelay(1);
  }

  return 1;
}

// ========== RoboMaster C型开发板发送的数据包发送函数 ==========

/**
 * @brief 发送Debug数据
 * @param huart UART句柄
 * @param name 调试数据名称
 * @param type 数据类型
 * @param data 数据值
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_debug_data(UART_HandleTypeDef* huart, const char* name, uint8_t type, float data)
{
    if (huart == NULL || name == NULL) return 0;

    debug_data_t debug_data;
    memset(&debug_data, 0, sizeof(debug_data));

    // 安全拷贝名称
    strncpy(debug_data.name, name, sizeof(debug_data.name) - 1);
    debug_data.type = type;
    debug_data.data = data;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)&debug_data,
                        sizeof(debug_data),
                        PKT_ID_DEBUG, 10);
}

/**
 * @brief 发送机器人状态信息
 * @param huart UART句柄
 * @param state_info 状态信息结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_state_info(UART_HandleTypeDef* huart, const robot_state_info_t* state_info)
{
    if (huart == NULL || state_info == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)state_info,
                        sizeof(robot_state_info_t),
                        PKT_ID_ROBOT_STATE_INFO, 10);
}

/**
 * @brief 发送事件数据
 * @param huart UART句柄
 * @param event_data 事件数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_event_data(UART_HandleTypeDef* huart, const event_data_t* event_data)
{
    if (huart == NULL || event_data == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)event_data,
                        sizeof(event_data_t),
                        PKT_ID_EVENT, 10);
}

/**
 * @brief 发送所有机器人血量数据
 * @param huart UART句柄
 * @param robot_hp 血量数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_all_robot_hp(UART_HandleTypeDef* huart, const all_robot_hp_t* robot_hp)
{
    if (huart == NULL || robot_hp == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)robot_hp,
                        sizeof(all_robot_hp_t),
                        PKT_ID_ALL_ROBOT_HP, 10);
}

/**
 * @brief 发送游戏状态数据
 * @param huart UART句柄
 * @param game_status 游戏状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_game_status(UART_HandleTypeDef* huart, const game_status_t* game_status)
{
    if (huart == NULL||game_status==NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)game_status,
                        sizeof(game_status_t),
                        PKT_ID_GAME_STATUS, 10);
}

/**
 * @brief 发送机器人运动数据
 * @param huart UART句柄
 * @param vx x方向速度
 * @param vy y方向速度
 * @param wz z轴角速度
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_motion(UART_HandleTypeDef* huart, const robot_motion_t* motion)
{
    if (huart == NULL||motion==NULL) return 0;


    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)motion,
                        sizeof(robot_motion_t),
                        PKT_ID_ROBOT_MOTION, 10);
}

/**
 * @brief 发送地面机器人位置数据
 * @param huart UART句柄
 * @param position 位置数据结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_ground_robot_position(UART_HandleTypeDef* huart, const ground_robot_position_t* position)
{
    if (huart == NULL || position == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)position,
                        sizeof(ground_robot_position_t),
                        PKT_ID_GROUND_ROBOT_POS, 10);
}

/**
 * @brief 发送RFID状态数据
 * @param huart UART句柄
 * @param rfid_status RFID状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_rfid_status(UART_HandleTypeDef* huart, const rfid_status_t* rfid_status)
{
    if (huart == NULL || rfid_status == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)rfid_status,
                        sizeof(rfid_status_t),
                        PKT_ID_RFID_STATUS, 10);
}

/**
 * @brief 发送机器人状态数据
 * @param huart UART句柄
 * @param robot_status 机器人状态结构体指针
 * @return 发送成功返回1，失败返回0
 */
static uint8_t send_robot_status(UART_HandleTypeDef* huart, const robot_status_t* robot_status)
{
    if (huart == NULL || robot_status == NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)robot_status,
                        sizeof(robot_status_t),
                        PKT_ID_ROBOT_STATUS, 10);
}

/**
 * @brief 发送云台关节状态
 * @param huart UART句柄
 * @param joint_state 关节状态结构体指针
 */
static  uint8_t send_joint_state(UART_HandleTypeDef* huart, const joint_state_t* joint_state)
{
    if (huart == NULL||joint_state==NULL) return 0;

    uint32_t timestamp = HAL_GetTick();
    return protocol_send(huart, timestamp,
                        (uint8_t*)joint_state,
                        sizeof(joint_state_t),
                        PKT_ID_JOINT_STATE, 10);
}

void updata_senddata(void) {
  send_data.game_status.game_progress=10;
  send_data.game_status.stage_remain_time=0xBA;
}

void navigator_send(UART_HandleTypeDef *instance) {
  updata_senddata();
  // send_all_robot_hp(instance->usart_handle,&send_data.all_robot_hp);
  // send_event_data(instance->usart_handle,&send_data.event_data);
  send_game_status(instance,&send_data.game_status);
  // send_ground_robot_position(instance->usart_handle,&send_data.ground_robot_position);
  // send_joint_state(instance->usart_handle,&send_data.joint_state);
  // send_rfid_status(instance->usart_handle,&send_data.rfid_status);
  // send_robot_motion(instance->usart_handle,&send_data.robot_motion);
  // send_robot_state_info(instance->usart_handle,&send_data.state_info);
  // send_robot_status(instance->usart_handle,&send_data.robot_status);
}


//下面是发送相关
static uint8_t parse_robot_cmd_packet(uint8_t *data)
{
  if (data == NULL) {
    return 0;
  }

  // 数据段起始位置 (帧头4字节 + 时间戳4字节)
  uint8_t *data_segment = &data[PROTOCOL_HEADER_LEN + 4];
  uint8_t data_len = data[1];

  // 检查数据长度是否匹配
  if (data_len != sizeof(robot_cmd_t)) {
#ifdef NAVIGATOR_DEBUG
    printf("Robot CMD packet length mismatch: expected %lu, got %d\n",
           sizeof(robot_cmd_t), data_len);
#endif
    return 0;
  }

  // 解析数据到临时结构体
  robot_cmd_t temp_cmd;
  memcpy(&temp_cmd, data_segment, sizeof(robot_cmd_t));

  // 拷贝数据到接收结构体
  recv_data.robot_cmd.speed_vector.vx = temp_cmd.speed_vector.vx;
  recv_data.robot_cmd.speed_vector.vy = temp_cmd.speed_vector.vy;
  recv_data.robot_cmd.speed_vector.wz = temp_cmd.speed_vector.wz;

  recv_data.robot_cmd.chassis.roll = temp_cmd.chassis.roll;
  recv_data.robot_cmd.chassis.pitch = temp_cmd.chassis.pitch;
  recv_data.robot_cmd.chassis.yaw = temp_cmd.chassis.yaw;
  recv_data.robot_cmd.chassis.leg_length = temp_cmd.chassis.leg_length;

  recv_data.robot_cmd.gimbal.pitch = temp_cmd.gimbal.pitch;
  recv_data.robot_cmd.gimbal.yaw = temp_cmd.gimbal.yaw;

  recv_data.robot_cmd.shoot.fire = temp_cmd.shoot.fire;
  recv_data.robot_cmd.shoot.fric_on = temp_cmd.shoot.fric_on;

  return 1;
}

static uint8_t parse_navigator_packet(uint8_t *data, uint16_t length)
{
  if (data == NULL || length < (PROTOCOL_HEADER_LEN + 4 + 2)) {
    return 0;
  }

  // 1. 检查SOF
  if (data[0] != PROTOCOL_SOF) {
    return 0;
  }

  // 2. 提取基本信息
  uint8_t data_len = data[1];
  uint8_t data_id = data[2];

  // 3. 检查CRC8
  uint8_t crc8_calc = get_CRC8_check_sum(data, 3, PROTOCOL_CRC8_INIT);
  if (crc8_calc != data[3]) {
    return 0;
  }

  // 4. 提取时间戳 (小端模式)
  uint32_t timestamp = (data[7] << 24) | (data[6] << 16) |
                      (data[5] << 8) | data[4];

  // 5. 检查CRC16
  uint16_t checksum_len = PROTOCOL_HEADER_LEN + 4 + data_len;
  uint16_t crc16_calc = get_CRC16_check_sum(data, checksum_len, PROTOCOL_CRC16_INIT);
  uint16_t crc16_recv = (data[checksum_len + 1] << 8) | data[checksum_len];

  if (crc16_calc != crc16_recv) {
    return 0;
  }

  // 6. 根据数据ID处理不同的数据包
  switch (data_id) {
    case PKT_ID_ROBOT_CMD:
      // 解析机器人控制命令
      return parse_robot_cmd_packet(data);

      // 可以添加其他数据包的解析
    default:
      // 未知的数据包ID
#ifdef NAVIGATOR_DEBUG
      printf("Unknown packet ID: 0x%02X\n", data_id);
#endif
      return 0;
  }
}



  static void DecodeNavigator() {
    // 检查是否有可用的接收数据
    if (navigator_usart_instance == NULL) {
        return;
    }

    // 获取接收缓冲区中的数据
    uint8_t *rx_data = navigator_usart_instance->recv_buff;
    uint16_t data_size = navigator_usart_instance->recv_buff_size;

    // 处理接收到的数据
    for (uint16_t i = 0; i < data_size; i++) {
        uint8_t byte = rx_data[i];

        switch (recv_state) {
            case RECV_STATE_SOF:
                if (byte == PROTOCOL_SOF) {
                    recv_index = 0;
                    recv_buffer[recv_index++] = byte;
                    recv_state = RECV_STATE_LEN;
                }
                break;

            case RECV_STATE_LEN:
                recv_buffer[recv_index++] = byte;
                expected_data_len = byte;
                recv_state = RECV_STATE_ID;
                break;

            case RECV_STATE_ID:
                recv_buffer[recv_index++] = byte;
                current_packet_id = byte;
                recv_state = RECV_STATE_CRC8;
                break;

            case RECV_STATE_CRC8:
                recv_buffer[recv_index++] = byte;
                // 校验CRC8
                uint8_t crc8_calc = get_CRC8_check_sum(recv_buffer, 3, PROTOCOL_CRC8_INIT);
                if (crc8_calc == byte) {
                    recv_state = RECV_STATE_TIMESTAMP;
                } else {
                    // CRC8校验失败，重置状态机
                    recv_state = RECV_STATE_SOF;
                    recv_data.crc_errors++;
                }
                break;

            case RECV_STATE_TIMESTAMP:
                recv_buffer[recv_index++] = byte;
                if (recv_index >= (PROTOCOL_HEADER_LEN + 4)) {
                    recv_state = RECV_STATE_DATA;
                }
                break;

            case RECV_STATE_DATA:
                recv_buffer[recv_index++] = byte;
                if (recv_index >= (PROTOCOL_HEADER_LEN + 4 + expected_data_len)) {
                    recv_state = RECV_STATE_CRC16;
                }
                break;

            case RECV_STATE_CRC16:
                recv_buffer[recv_index++] = byte;
                if (recv_index >= (PROTOCOL_HEADER_LEN + 4 + expected_data_len + 2)) {
                    // 完整帧接收完成，开始解析
                    if (parse_navigator_packet(recv_buffer, recv_index)) {
                        recv_data.last_update_time = HAL_GetTick();
                        recv_data.data_valid = 1;
                        last_packet_time = HAL_GetTick();

                        // 打印接收到的控制命令（调试用）
                        #ifdef NAVIGATOR_DEBUG
                        printf("Received robot command: vx=%.2f, vy=%.2f, wz=%.2f\n",
                               recv_data.speed_vector.vx,
                               recv_data.speed_vector.vy,
                               recv_data.speed_vector.wz);
                        #endif
                    } else {
                        recv_data.data_valid = 0;
                        recv_data.crc_errors++;
                    }

                    // 重置状态机
                    recv_state = RECV_STATE_SOF;
                }
                break;

            default:
                recv_state = RECV_STATE_SOF;
                break;
        }

        // 防止缓冲区溢出
        if (recv_index >= NAVIGATOR_RECV_SIZE) {
            recv_state = RECV_STATE_SOF;
            recv_index = 0;
        }
    }
}

navigator_recv_t* navigator_init(UART_HandleTypeDef *usart_handle) {
  USART_Init_Config_s conf;
  conf.module_callback = DecodeNavigator;
  conf.recv_buff_size = NAVIGATOR_RECV_SIZE;
  conf.usart_handle =usart_handle;
  navigator_usart_instance = USARTRegister(&conf);


  return &recv_data;
}