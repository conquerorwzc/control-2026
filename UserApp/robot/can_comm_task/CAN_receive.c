#include "CAN_receive.h"
#include "main.h"
#include "bsp_can.h"
#include "robot.h"

static CANInstance* can_tx_instance = NULL;  // 发送实例
static CANInstance* can_rx_instance = NULL;  // 接收实例

// 接收数据全局变量（在robot.h中声明为extern）
uint8_t received_ui_flag = 0;
uint8_t received_fric_flag = 0;
uint8_t received_chassis_vx = 0;
uint8_t received_chassis_vy = 0;
int16_t received_pitch_abs = 0;
uint8_t received_chassis_behaviour = 0;
uint8_t received_cap_flag = 0;
uint8_t data_received = 0;

// CAN实例指针
//static CANInstance* board_comm_instance = NULL;

/**
 * @brief CAN接收回调函数 - 通过bsp_can框架调用
 */
static void BoardCommRxCallback(CANInstance* instance)
{
  // 从CAN实例的rx_buff中读取数据
  uint8_t* rx_data = instance->rx_buff;

  // 解析双板通信数据
  received_ui_flag = rx_data[0];
  received_fric_flag = rx_data[1];
  received_chassis_vx = rx_data[2];
  received_chassis_vy = rx_data[3];
  received_pitch_abs = (rx_data[4] << 8) | rx_data[5];
  received_chassis_behaviour = rx_data[6];
  received_cap_flag = rx_data[7];
  data_received = 1;  // 设置数据接收标志
}

/**
 * @brief 初始化CAN发送和接收实例
 */
void CANReceive_Init(void)
{
  if (DEVICE_ROLE_TX) {
    // 发送板：只初始化发送实例
    CAN_Init_Config_s tx_config = {
      .can_handle = &hcan1,
      .tx_id = 0x218,      // 发送ID
      .rx_id = 0x000,      // 不接收
      .can_module_callback = NULL,
      .id = NULL
  };
    can_tx_instance = CANRegister(&tx_config);

  } else {
    // 接收板：只初始化接收实例
    CAN_Init_Config_s rx_config = {
      .can_handle = &hcan1,
            .tx_id = 0x000,      // 不发送
            .rx_id = 0x218,      // 接收ID
            .can_module_callback = BoardCommRxCallback,
            .id = NULL
        };
    can_rx_instance = CANRegister(&rx_config);
  }
}

// /**
//  * @brief 初始化CAN接收
//  */
// void CANReceive_Init(void) {
//   CAN_Init_Config_s config = {
//     .can_handle = &hcan1,                    // 使用CAN1
//     .tx_id = 0x000,                          // 发送ID（如果不发送可以设为任意值）
//     .rx_id = 0x218,                          // 接收ID：双板通信数据
//     .can_module_callback = BoardCommRxCallback, // 接收回调函数
//     .id = NULL                               // 不需要额外标识
//   };
//   // 注册CAN实例
//   board_comm_instance = CANRegister(&config);
// }

/* 和bsp_can.c回调重复
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1)  // 只处理CAN1的数据
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
      case 0x218:  // 双板通信数据
      {
        // 解析双板通信数据
        received_ui_flag = rx_data[0];
        received_fric_flag = rx_data[1];
        received_chassis_vx = rx_data[2];
        received_chassis_vy = rx_data[3];
        received_pitch_abs = (rx_data[4] << 8) | rx_data[5];
        received_chassis_behaviour = rx_data[6];
        received_cap_flag = rx_data[7];
        data_received = 1;  // 设置数据接收标志
        break;
      }
      default:
        break;
    }
  }
}
*/
