/*
* @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "daemon.h"

typedef struct
{
  float cap_v;
  uint8_t error_detect;
  float out_p;//除以100以后单位是W
  float in_p;//除以100以后单位是W
} SuperCap_Measure_s;

/* 超级电容实例 */
typedef struct
{
  CANInstance *can_ins; // CAN实例
  SuperCap_Measure_s cap_msg; // 超级电容信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
  CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);


/**
 * @brief 发送超级电容控制消息
 *
 * @param instance 超级电容实例
 * @param power 功率值
 * @param buffer 缓冲区值
 * @param state 状态值
 */
void SuperCapSendMessage(SuperCapInstance *instance, int16_t power, uint16_t buffer, uint8_t state);

#endif // !SUPER_CAP_H