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

#pragma pack(1)
typedef struct//发给超电的数据结构体
{
  uint8_t enableDCDC : 1;
  uint8_t systemRestart : 1;
  uint8_t resv0 : 6;
  uint16_t feedbackRefereePowerLimit;
  uint16_t feedbackRefereeEnergyBuffer;
  uint8_t resv1[3];
} TxData;

typedef struct //接收的数据结构体
{
  uint8_t errorCode;
  float chassisPower;
  uint16_t chassisPowerLimit;
  uint8_t capEnergy;
} RxData;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
  CANInstance *can_instance; // CAN实例
  DaemonInstance* daemon; //看门狗
  RxData cap_msg; //超级电容信息
  TxData send_msg; //发送给电容的信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
  CAN_Init_Config_s can_config; //tx是0x51, rx是0x61
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
void SuperCapSendMessage(uint8_t enable, int16_t powerlimit, uint16_t buffer);

#endif // !SUPER_CAP_H