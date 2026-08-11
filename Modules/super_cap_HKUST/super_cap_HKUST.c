/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2025-12-10
 */
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"
#include "super_cap_HKUST.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针
DaemonInstance *supercap_daemon_instance; // daemon实例，用于监测通信状态


/**
 * @brief 超级电容离线回调函数
 *
 * @param instance 超级电容实例指针
 */
static void SuperCapOfflineCallback()
{
    LOGWARNING("[supercap] supercap offline, restart communication.");
    // 可以在这里添加离线处理逻辑，例如禁用超电输出等
}

static void DecodeSuperCap(CANInstance *_instance)
{
    memcpy(&super_cap_instance->cap_msg, _instance->rx_buff, sizeof(super_cap_instance->cap_msg));
    // 重载daemon，表示正常接收到数据
    DaemonReload(supercap_daemon_instance);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = DecodeSuperCap;
    super_cap_instance->can_instance = CANRegister(&supercap_config->can_config);

    Daemon_Init_Config_s daemon_config= {
      .callback = SuperCapOfflineCallback, // 离线时调用的回调函数,会重启串口接收
      .owner_id = NULL,
      .reload_count = 5, // 50ms
    };; // daemon配置
    supercap_daemon_instance = DaemonRegister(&daemon_config);

    return super_cap_instance;
}

//港科的超电控制逻辑


void SuperCapSendMessage(uint8_t enable, int16_t powerlimit, uint16_t buffer)
{
    memset(&super_cap_instance->send_msg,0,sizeof(super_cap_instance->send_msg));// 初始化发送数据

    super_cap_instance->send_msg.enableDCDC = enable;
    super_cap_instance->send_msg.systemRestart = 0;
    super_cap_instance->send_msg.feedbackRefereePowerLimit = powerlimit;
    super_cap_instance->send_msg.feedbackRefereeEnergyBuffer=buffer;

    // 使用现有的CAN发送机制发送数据
    memcpy(super_cap_instance->can_instance->tx_buff, &super_cap_instance->send_msg, sizeof(super_cap_instance->send_msg));
    CANTransmit(super_cap_instance->can_instance, 1);
}