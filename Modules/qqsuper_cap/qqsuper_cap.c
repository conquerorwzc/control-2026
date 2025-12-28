/*
* @Descripttion:基于super_cap移植的齐奇超电兼容模块
 * @version:
 * @Author: Yangzhecheng
 * @Date: 2025/12/25 15:05:03
 * @LastEditTime: 2025/12/25 15:05:03
 */

#include "qqsuper_cap.h"
#include "memory.h"
#include "stdlib.h"

static QQSuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void QQSuperCapRxCallback(CANInstance *_instance)
{
  uint8_t *rxbuff;
  QQSuperCap_Msg_s *Msg;
  rxbuff = _instance->rx_buff;
  Msg = &super_cap_instance->cap_msg;
  Msg->err = rxbuff[0];
  Msg->status = rxbuff[1];
  Msg->vol = (float)(rxbuff[2] << 8 | rxbuff[3])/100;       		//电容电压
  Msg->power = (float)(rxbuff[4] << 8 | rxbuff[5])/100;      	//电容端电流
  Msg->power_target =(float)(rxbuff[6] << 8 | rxbuff[7])/100; 			//电容剩余能量百分比
}

QQSuperCapInstance *QQSuperCapInit(QQSuperCap_Init_Config_s *supercap_config)
{
  super_cap_instance = (QQSuperCapInstance *)malloc(sizeof(QQSuperCapInstance));
  memset(super_cap_instance, 0, sizeof(QQSuperCapInstance));

  supercap_config->can_config.can_module_callback = QQSuperCapRxCallback;
  super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
  return super_cap_instance;
}

void QQSuperCapSend(QQSuperCapInstance *instance, uint8_t *data)
{
  memcpy(instance->can_ins->tx_buff, data, 8);
  CANTransmit(instance->can_ins,1);
}

QQSuperCap_Msg_s QQSuperCapGet(QQSuperCapInstance *instance)
{
  return instance->cap_msg;
}