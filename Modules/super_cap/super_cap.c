/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2025-12-10
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void DecodeSuperCap(CANInstance *_instance)
{
    uint8_t *rxbuff;
    SuperCap_Measure_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &super_cap_instance->cap_msg;
    Msg->cap_v = rxbuff[1]<<8|rxbuff[0];
    Msg->out_p = rxbuff[3]<<8|rxbuff[2];
    Msg->in_p = rxbuff[5]<<8|rxbuff[4];
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = DecodeSuperCap;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}


void SuperCapSendMessage(SuperCapInstance *instance, int16_t power, uint16_t buffer, uint8_t state)
{
    uint8_t tx_data[8] = {0}; // 初始化发送数据

    // 按照原函数的格式填充数据
    memcpy(tx_data, &power, sizeof(power));
    memcpy(tx_data + 4, &buffer, sizeof(buffer));
    tx_data[6] = state;
    
    // 使用现有的CAN发送机制发送数据
    memcpy(instance->can_ins->tx_buff, tx_data, 8);
    CANTransmit(instance->can_ins, 1);
}