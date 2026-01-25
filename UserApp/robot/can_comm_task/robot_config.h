#ifndef CONTROL_2026_ROBOT_CONFIG_H
#define CONTROL_2026_ROBOT_CONFIG_H

#pragma once

#include "robot.h"

#if DEVICE_ROLE_TX
    // 发送板配置
    #define BOARD_TX_ID 0x218
    #define BOARD_RX_ID 0x219
#else
    // 接收板配置
    #define BOARD_TX_ID 0x219
    #define BOARD_RX_ID 0x218
#endif

static CANComm_Init_Config_s comm_config = {
  .recv_data_len = 8,        // 接收数据长度，根据实际需求调整
  .send_data_len = 8,        // 发送数据长度，根据实际需求调整
  .daemon_count = 1000,      // 看门狗重载计数，根据实际需求调整
  .can_config = {
    .can_handle = &hcan1,  // 假设使用CAN1，根据实际使用的CAN句柄调整
    .tx_id = BOARD_TX_ID,        // 发送ID，根据实际需求调整
    .rx_id = BOARD_RX_ID,        // 接收ID，根据实际需求调整
    .id = NULL                   // 将在CANCommInit中设置
  }
};

// CAN实例配置（用于数据存储）
static CANInstance board_can_comm_data = {
  .can_handle = &BOARD_CAN,
  .tx_id = 0x218,          // 与comm_config中的ID保持一致
  .rx_id = 0x218,
  .txconf = {
    .StdId = 0x218,      // 发送ID
    .IDE = CAN_ID_STD,   // 标准帧
    .RTR = CAN_RTR_DATA, // 数据帧
    .DLC = 0x08,         // 数据长度8字节
  }
};
#endif  // CONTROL_2026_ROBOT_CONFIG_H
