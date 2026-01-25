/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据顺序发送
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "can_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "robot_config.h"

bool init_finish = false;

uint8_t* received_data = NULL;
CANCommInstance* can_comm_instance = NULL;

void RobotInit() {
  // 空实现

}
void RobotTask()
{
    //要在云台和底盘任务开始之前完成该任务的初始化
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    // 初始化CAN接收
    can_comm_instance = CANCommInit(&comm_config);

    init_finish = true;

    if (DEVICE_ROLE_TX)
    {
      while(1)
      {
          // 测试数据,实际应用中这些数据应该来自其他模块
          board_can_comm_data.tx_buff[0] = 1;  // ui_flag
          board_can_comm_data.tx_buff[1] = 0;  // fric_flag
          board_can_comm_data.tx_buff[2] = 100; // chassis_vx
          board_can_comm_data.tx_buff[3] = 50;  // chassis_vy
          board_can_comm_data.tx_buff[4] = (3000 >> 8) & 0xFF;  // pitch_abs 高字节
          board_can_comm_data.tx_buff[5] = 3000 & 0xFF;         // pitch_abs 低字节
          board_can_comm_data.tx_buff[6] = 1;  // chassis_behaviour
          board_can_comm_data.tx_buff[7] = 0;  // cap_flag

          CANCommSend(can_comm_instance, board_can_comm_data.tx_buff);
          //can数据数据发送
          vTaskDelay(CAN_COMM_TASK_TIME);
      }
    }
    else {
      // 接收板逻辑
      while(1) {
        // 检查CAN通信是否在线
        if (CANCommIsOnline(can_comm_instance)) {
          // 检查是否有新数据更新
          received_data = (uint8_t*)CANCommGet(can_comm_instance);

          // 如果收到数据，可以在这里处理
          if (received_data != NULL) {
            // 解析接收到的数据到全局变量
            memcpy(board_can_comm_data.rx_buff, received_data, 8);
          }
        }
        vTaskDelay(50); // 接收板可以延时较长
      }
    }
}