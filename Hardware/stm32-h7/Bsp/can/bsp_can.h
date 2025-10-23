#ifndef bsp_can_H
#define bsp_can_H

#include <stdint.h>

#include "fdcan.h"

// 最多能够支持的FDCAN设备数
#define CAN_MX_REGISTER_CNT 16      // 这个数量取决于FDCAN总线的负载，所有总线上最多的设备数
#define MX_CAN_FILTER_CNT (3 * 28)  // H7系列FDCAN每个实例最多28个标准ID过滤器
#define DEVICE_CAN_CNT 3            // H7系列通常有FDCAN1,FDCAN2,FDCAN3,因此为3

/* FDCAN instance typedef, every module registered to FDCAN should have this variable */
#pragma pack(1)
typedef struct _ {
  FDCAN_HandleTypeDef* can_handle;  // fdcan句柄
  FDCAN_TxHeaderTypeDef txconf;     // FDCAN报文发送配置
  uint32_t tx_id;                   // 发送id
  uint8_t tx_buff[64];              // 发送缓存,支持CAN FD最大64字节,经典CAN模式下为8字节
  uint8_t rx_buff[64];              // 接收缓存,支持CAN FD最大64字节
  uint32_t rx_id;                   // 接收id
  uint8_t rx_len;                   // 接收长度,经典CAN为0-8,CAN FD可达64
  // 接收的回调函数,用于解析接收到的数据
  void (*can_module_callback)(struct _*);  // callback needs an instance to tell among registered ones
  void* id;                                // 使用fdcan外设的模块指针(即id指向的模块拥有此fdcan实例,是父子关系)
} CANInstance;
#pragma pack()

/* FDCAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct {
  FDCAN_HandleTypeDef* can_handle;            // fdcan句柄
  uint32_t tx_id;                             // 发送id
  uint32_t rx_id;                             // 接收id
  void (*can_module_callback)(CANInstance*);  // 处理接收数据的回调函数
  void* id;                                   // 拥有fdcan实例的模块地址,用于区分不同的模块
} CAN_Init_Config_s;

/**
 * @brief Register a module to FDCAN service,remember to call this before using a FDCAN device
 *        注册(初始化)一个fdcan实例,需要传入初始化配置的指针.
 * @param config init config
 * @return FDCANInstance* fdcan instance owned by module
 */
CANInstance* CANRegister(CAN_Init_Config_s* config);

/**
 * @brief 修改FDCAN发送报文的数据帧长度;注意经典CAN最大长度为8,CAN FD模式可达64字节
 *        在没有进行修改的时候,默认长度为8(经典CAN模式)
 *
 * @param _instance 要修改长度的fdcan实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance* _instance, uint8_t length);

/**
 * @brief transmit mesg through FDCAN device,通过fdcan实例发送消息
 *        发送前需要向FDCAN实例的tx_buff写入发送数据
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 *
 * @param timeout 超时时间,单位为ms
 * @param _instance fdcan instance owned by module
 */
uint8_t CANTransmit(CANInstance* _instance, float timeout);

#endif
