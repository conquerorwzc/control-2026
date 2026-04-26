//
// Created by zeg on 2026/4/13.
//
#ifndef _TOFSENSE_H_
#define _TOFSENSE_H_

#include <stdint.h>
#include "bsp_can.h"   // 确保包含你们的CAN bsp头文件

#define TOF_MAX_CNT 8  // 最大支持的TOF传感器数量

// TOFSense 工作模式枚举
typedef enum {
  TOF_ACTIVE_MODE = 0,   // 主动输出模式 (仅接收)
  TOF_QUERY_MODE  = 1,   // 查询/级联模式 (需要定时发送查询帧)
} TOFSense_Mode_e;

// TOFSense 测量数据结构体 (仅保留单点数据)
typedef struct {
  float dis;                  // 距离数据 (m)
  uint8_t dis_status;         // 距离状态指示
  uint16_t signal_strength;   // 信号强度
} TOFSense_Measure_s;

// TOFSense 实例结构体
typedef struct {
  TOFSense_Mode_e mode;           // 传感器工作模式
  CANInstance* can_instance;      // CAN实例句柄
  TOFSense_Measure_s measure;     // 解析后的数据
  uint8_t tof_id;                 // 传感器自身的ID (用于级联查询)
} TOFSenseInstance;

// TOFSense 初始化配置结构体
typedef struct {
  TOFSense_Mode_e mode;           // 工作模式
  uint8_t tof_id;                 // 模块ID
  CAN_Init_Config_s can_init_config; // CAN底层配置
} TOFSense_Init_Config_s;

/**
 * @brief 初始化 TOFSense 模块
 * @param config 初始化配置指针
 * @return TOFSenseInstance* 实例指针
 */
TOFSenseInstance* TOFSenseInit(TOFSense_Init_Config_s* config);

/**
 * @brief TOFSense 定时任务，用于在查询模式下发送查询帧
 * @note  需放在 RTOS 的定时 Task 中运行，频率根据 interval (约20ms) 决定
 */
void TOFSenseTask(void);

#endif // _TOFSENSE_H_