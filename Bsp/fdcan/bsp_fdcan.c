#include "bsp_fdcan.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* fdcan instance ptrs storage, used for recv callback */
// 在FDCAN产生接收中断会遍历数组,选出hfdcan和rxid与发生中断的实例相同的那个,调用其回调函数
static CANInstance* fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局FDCAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by FDCANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被FDCANRegister()调用
 *        给FDCAN添加过滤器后,FDCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note H7的FDCAN每个实例都有独立的过滤器,这里为每个实例分配独立的过滤器索引
 *       初始化时,奇数id的模块会被分配到RxFIFO0,偶数id的模块会被分配到RxFIFO1
 *
 * @attention FDCAN的过滤器配置比传统CAN更加灵活,支持标准ID和扩展ID的独立配置
 *
 * @param _instance fdcan instance owned by specific module
 */
static void FDCANAddFilter(CANInstance* _instance) {
  FDCAN_FilterTypeDef fdcan_filter_conf;
  static uint8_t fdcan1_filter_idx = 0, fdcan2_filter_idx = 0, fdcan3_filter_idx = 0;
  uint8_t* current_filter_idx;

  // 根据不同的FDCAN实例选择对应的过滤器索引
  if (_instance->can_handle == &hfdcan1)
    current_filter_idx = &fdcan1_filter_idx;
  else if (_instance->can_handle == &hfdcan2)
    current_filter_idx = &fdcan2_filter_idx;
  else if (_instance->can_handle == &hfdcan3)
    current_filter_idx = &fdcan3_filter_idx;
  else
    return; // 无效的FDCAN实例

  fdcan_filter_conf.IdType = FDCAN_STANDARD_ID; // 使用标准ID
  fdcan_filter_conf.FilterIndex = (*current_filter_idx)++; // 过滤器索引
  fdcan_filter_conf.FilterType = FDCAN_FILTER_MASK; // 使用掩码过滤器
  fdcan_filter_conf.FilterConfig = (_instance->rx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
  fdcan_filter_conf.FilterID1 = _instance->rx_id; // 要接收的ID
  fdcan_filter_conf.FilterID2 = 0x7FF; // 掩码,精确匹配

  HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);
}

/**
 * @brief 在第一个FDCAN实例初始化的时候会自动调用此函数,启动FDCAN服务
 *
 * @note 此函数会启动所有可用的FDCAN实例,开启RxFIFO0和RxFIFO1的新消息中断
 */
static void FDCANServiceInit() {
  // 启动FDCAN1
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  // 启动FDCAN2
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  // 启动FDCAN3 (如果存在)
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

/* ----------------------- extern callable functions -----------------------*/

CANInstance* CANRegister(CAN_Init_Config_s* config) {
  if (!idx) {
    FDCANServiceInit(); // 第一次注册,先进行硬件初始化
    LOGINFO("[bsp_fdcan] FDCAN Service Init");
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) // 超过最大实例数
  {
    while (1)
      LOGERROR("[bsp_fdcan] FDCAN instance exceeded MAX num, consider balance the load of FDCAN bus");
  }
  for (size_t i = 0; i < idx; i++) {
    // 重复注册 | id重复
    if (fdcan_instance[i]->rx_id == config->rx_id && fdcan_instance[i]->can_handle == config->can_handle) {
      while (1)
        LOGERROR("[bsp_fdcan] FDCAN id crash, tx [%d] or rx [%d] already registered", config->tx_id,
                 config->rx_id);
    }
  }

  CANInstance* instance = (CANInstance*)malloc(sizeof(CANInstance)); // 分配空间
  memset(instance, 0, sizeof(CANInstance)); // 分配的空间未必是0,所以要先清空

  // 进行发送报文的配置
  instance->txconf.Identifier = config->tx_id; // 发送id
  instance->txconf.IdType = FDCAN_STANDARD_ID; // 使用标准id
  instance->txconf.TxFrameType = FDCAN_DATA_FRAME; // 发送数据帧
  instance->txconf.DataLength = FDCAN_DLC_BYTES_8; // 默认发送长度为8字节，可到64字节
  instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  instance->txconf.BitRateSwitch = FDCAN_BRS_OFF; // 经典CAN模式,不使用位率切换
  instance->txconf.FDFormat = FDCAN_CLASSIC_CAN; // 使用经典CAN格式
  instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  instance->txconf.MessageMarker = 0;

  // 设置回调函数和接收发送id
  instance->can_handle = config->can_handle;
  instance->tx_id = config->tx_id;
  instance->rx_id = config->rx_id;
  instance->can_module_callback = config->can_module_callback;
  instance->id = config->id;

  FDCANAddFilter(instance); // 添加FDCAN过滤器规则
  fdcan_instance[idx++] = instance; // 将实例保存到fdcan_instance中

  return instance; // 返回fdcan实例指针
}

uint8_t CANTransmit(CANInstance* _instance, float timeout) {
  static uint32_t busy_count;
  static volatile float wait_time __attribute__((unused)); // for cancel warning
  float dwt_start = DWT_GetTimeline_ms();

  while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0) // 等待TX FIFO有空闲空间
  {
    if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
    {
      LOGWARNING("[bsp_fdcan] FDCAN TX FIFO full! failed to add msg to FIFO. Cnt [%d]", busy_count);
      busy_count++;
      return 0;
    }
  }
  wait_time = DWT_GetTimeline_ms() - dwt_start;

  // 将消息添加到TX FIFO队列
  if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK) {
    LOGWARNING("[bsp_fdcan] FDCAN bus BUSY! cnt:%d", busy_count);
    busy_count++;
    return 0;
  }
  return 1; // 发送成功
}

void CANSetDLC(CANInstance* _instance, uint8_t length) {
  // 根据长度设置对应的DLC值
  uint32_t dlc_value;

  if (length <= 8) {
    // 经典CAN模式下的DLC映射
    switch (length) {
      case 0:
        dlc_value = FDCAN_DLC_BYTES_0;
        break;
      case 1:
        dlc_value = FDCAN_DLC_BYTES_1;
        break;
      case 2:
        dlc_value = FDCAN_DLC_BYTES_2;
        break;
      case 3:
        dlc_value = FDCAN_DLC_BYTES_3;
        break;
      case 4:
        dlc_value = FDCAN_DLC_BYTES_4;
        break;
      case 5:
        dlc_value = FDCAN_DLC_BYTES_5;
        break;
      case 6:
        dlc_value = FDCAN_DLC_BYTES_6;
        break;
      case 7:
        dlc_value = FDCAN_DLC_BYTES_7;
        break;
      case 8:
        dlc_value = FDCAN_DLC_BYTES_8;
        break;
      default:
        while (1)
          LOGERROR("[bsp_fdcan] FDCAN DLC error! Invalid length for classic CAN");
    }
  } else if (length <= 64) {
    // CAN FD模式下的DLC映射
    if (length <= 12) dlc_value = FDCAN_DLC_BYTES_12;
    else if (length <= 16) dlc_value = FDCAN_DLC_BYTES_16;
    else if (length <= 20) dlc_value = FDCAN_DLC_BYTES_20;
    else if (length <= 24) dlc_value = FDCAN_DLC_BYTES_24;
    else if (length <= 32) dlc_value = FDCAN_DLC_BYTES_32;
    else if (length <= 48) dlc_value = FDCAN_DLC_BYTES_48;
    else dlc_value = FDCAN_DLC_BYTES_64;

    // 需要切换到CAN FD模式
    _instance->txconf.FDFormat = FDCAN_FD_CAN;
    _instance->txconf.BitRateSwitch = FDCAN_BRS_ON;
  } else {
    while (1)
      LOGERROR("[bsp_fdcan] FDCAN DLC error! Maximum length is 64 bytes");
  }

  _instance->txconf.DataLength = dlc_value;
}

/* -----------------------callback definitions--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理RxFIFO0和RxFIFO1中断
 *        所有的实例都会被遍历,找到fdcan_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hfdcan FDCAN句柄
 * @param fifo_num RxFIFO编号 (0 or 1)
 */
static void FDCANRxFIFOCallback(FDCAN_HandleTypeDef* _hfdcan, uint32_t fifox) {
  FDCAN_RxHeaderTypeDef rxconf;
  uint8_t fdcan_rx_buff[64];
  while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空
  {
    if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff) == HAL_OK) {
      for (size_t i = 0; i < idx; ++i) {
        // 找到对应的实例
        if (_hfdcan == fdcan_instance[i]->can_handle && rxconf.Identifier == fdcan_instance[i]->rx_id) {
          // 回调函数不为空就调用
          if (fdcan_instance[i]->can_module_callback != NULL)
          {
            // 根据DLC值计算实际数据长度
            uint8_t rx_length = 0;
            switch (rxconf.DataLength) {
              case FDCAN_DLC_BYTES_0:
                rx_length = 0;
                break;
              case FDCAN_DLC_BYTES_1:
                rx_length = 1;
                break;
              case FDCAN_DLC_BYTES_2:
                rx_length = 2;
                break;
              case FDCAN_DLC_BYTES_3:
                rx_length = 3;
                break;
              case FDCAN_DLC_BYTES_4:
                rx_length = 4;
                break;
              case FDCAN_DLC_BYTES_5:
                rx_length = 5;
                break;
              case FDCAN_DLC_BYTES_6:
                rx_length = 6;
                break;
              case FDCAN_DLC_BYTES_7:
                rx_length = 7;
                break;
              case FDCAN_DLC_BYTES_8:
                rx_length = 8;
                break;
              case FDCAN_DLC_BYTES_12:
                rx_length = 12;
                break;
              case FDCAN_DLC_BYTES_16:
                rx_length = 16;
                break;
              case FDCAN_DLC_BYTES_20:
                rx_length = 20;
                break;
              case FDCAN_DLC_BYTES_24:
                rx_length = 24;
                break;
              case FDCAN_DLC_BYTES_32:
                rx_length = 32;
                break;
              case FDCAN_DLC_BYTES_48:
                rx_length = 48;
                break;
              case FDCAN_DLC_BYTES_64:
                rx_length = 64;
                break;
              default:
                rx_length = 8;
                break;
            }
            fdcan_instance[i]->rx_len = rx_length; // 保存接收到的数据长度
            memcpy(fdcan_instance[i]->rx_buff, fdcan_rx_buff, rx_length); // 消息拷贝到对应实例
            fdcan_instance[i]->can_module_callback(fdcan_instance[i]); // 触发回调进行数据解析和处理
          }
          break;
        }
      }
    }
  }
}

/**
 * @brief RxFIFO0中断回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    FDCANRxFIFOCallback(hfdcan, FDCAN_RX_FIFO0);
  }
}

/**
 * @brief RxFIFO1中断回调函数
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0) {
    FDCANRxFIFOCallback(hfdcan, FDCAN_RX_FIFO1);
  }
}
