//
// Created by 独梦幻想 on 2025/12/25.
//

#ifndef CONTROL_2026_QQSUPER_CAP_H
#define CONTROL_2026_QQSUPER_CAP_H

#endif  // CONTROL_2026_QQSUPER_CAP_H
#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
  uint8_t err;
  uint8_t status;
  float vol; // 电压
  float current; // 电流
  float power; // 功率
  float power_target;
} QQSuperCap_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
  CANInstance *can_ins; // CAN实例
  QQSuperCap_Msg_s cap_msg; // 超级电容信息
} QQSuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
  CAN_Init_Config_s can_config;
} QQSuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
QQSuperCapInstance *QQSuperCapInit(QQSuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void QQSuperCapSend(QQSuperCapInstance *instance, uint8_t *data);

