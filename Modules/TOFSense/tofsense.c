//
// Created by zeg on 2026/4/13.
//
#include "tofsense.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx = 0; // 全局索引，用于记录注册了多少个TOF模块
static TOFSenseInstance* tof_instance[TOF_MAX_CNT] = {NULL};

/**
 * @brief 接收回调函数，根据返回的 can_instance 对反馈报文进行解析
 * @param _instance 收到数据的CAN实例
 */
static void DecodeTOFSense(CANInstance* _instance) {
    uint8_t* rxbuff = _instance->rx_buff;

    // 通过ID指针强转，获取对应的TOF实例
    TOFSenseInstance* tof = (TOFSenseInstance*)_instance->id;
    TOFSense_Measure_s* measure = &tof->measure;

    // 解析距离 (协议：24位有符号数，除以256，再除以1000转为米)
    int32_t temp = (int32_t)(rxbuff[0] << 8 | rxbuff[1] << 16 | rxbuff[2] << 24) / 256;

    measure->dis = temp / 1000.0f;
    measure->dis_status = rxbuff[3];
    measure->signal_strength = rxbuff[4] | (rxbuff[5] << 8);
}

/**
 * @brief 初始化 TOFSense 实例
 * @param config 初始化配置
 * @return TOFSenseInstance* 返回模块实例指针
 */
TOFSenseInstance* TOFSenseInit(TOFSense_Init_Config_s* config) {
    if (idx >= TOF_MAX_CNT) {
        LOGERROR("[tofsense] Exceed max instance count!");
        return NULL;
    }

    TOFSenseInstance* instance = (TOFSenseInstance*)malloc(sizeof(TOFSenseInstance));
    memset(instance, 0, sizeof(TOFSenseInstance));

    // 基础参数配置
    instance->mode = config->mode;
    instance->tof_id = config->tof_id;

    // 注册CAN总线
    config->can_init_config.can_module_callback = DecodeTOFSense;
    config->can_init_config.id = instance; // 将实例地址作为身份标识传入
    instance->can_instance = CANRegister(&config->can_init_config);

    // 将实例存入全局数组
    tof_instance[idx++] = instance;
    return instance;
}

/**
 * @brief TOFSense 发送任务，仅在“查询/级联”模式下有效
 * @note 遍历所有的TOF实例，如果处于查询模式，则通过CAN下发查询帧
 */
void TOFSenseTask(void) {
    for (size_t i = 0; i < idx; ++i) {
        TOFSenseInstance* tof = tof_instance[i];

        if (tof->mode == TOF_QUERY_MODE) {
            // 装填查询帧数据
            tof->can_instance->tx_buff[0] = 0xFF;
            tof->can_instance->tx_buff[1] = 0xFF;
            tof->can_instance->tx_buff[2] = 0xFF;
            tof->can_instance->tx_buff[3] = tof->tof_id; // 写入要查询的传感器ID
            tof->can_instance->tx_buff[4] = 0xFF;
            tof->can_instance->tx_buff[5] = 0xFF;
            tof->can_instance->tx_buff[6] = 0xFF;
            tof->can_instance->tx_buff[7] = 0xFF;

            // 确保查询命令的发送ID为 0x402 (根据官方手册配置)
            #ifdef STM32H723xx
            tof->can_instance->txconf.Identifier = 0x402;
            #else
            tof->can_instance->txconf.StdId = 0x402;
            #endif

            // 触发发送
            CANTransmit(tof->can_instance, 1);
        }
    }
}
