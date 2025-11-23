/**
 * @file can_comm.h
 * @author Neo neozng1@hnu.edu.cn
 * @brief  用于多机CAN通信的收发模块
 * @version 0.1
 * @date 2022-11-27
 *
 * @copyright Copyright (c) 2022 HNUYueLu EC all rights reserved
 *
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"

#define MX_CAN_COMM_COUNT 4  // 注意均衡负载,一条总线上不要挂载过多的外设

#define CAN_COMM_MAX_BUFFSIZE 60  // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'       // 帧头
#define CAN_COMM_TAIL 'e'         // 帧尾
#define CAN_COMM_OFFSET_BYTES 4   // 's'+ datalen + 'e' + crc8

//can设备单次发送最大值
#define CAN_COMM_SINGLE_TRANSMIT_MAX_SIZE 8
//can通信队列容量
#define CAN_COMM_QUEUE_CAPACITY 11

//bool类型
typedef enum
{
  true = 1,
  false = 0
} bool;

//can通信目标
typedef enum
{
  CAN_COMM_NONE,
  CAN_COMM_GIMBAL,
  CAN_COMM_CHASSIS,
  CAN_COMM_SHOOT,
  CAN_COMM_TRIGGER,
  CAN_COMM_SHOOT_FLAGS,
  CAN_COMM_PITCHANGLE,
  CAN_COMM_SHOOT_FLAG,
} can_comm_target_e;

#pragma pack(1)
/* CAN comm 结构体, 拥有CAN comm的app应该包含一个CAN comm指针 */
typedef struct {
  CANInstance *can_ins;
  /* 发送部分 */
  uint8_t send_data_len;  // 发送数据长度
  uint8_t send_buf_len;   // 发送缓冲区长度,为发送数据长度+帧头单包数据长度帧尾以及校验和(4)
  uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES];  // 额外4个bytes保存帧头帧尾和校验和
  /* 接收部分 */
  uint8_t recv_data_len;  // 接收数据长度
  uint8_t recv_buf_len;   // 接收缓冲区长度,为接收数据长度+帧头单包数据长度帧尾以及校验和(4)
  uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES];  // 额外4个bytes保存帧头帧尾和校验和
  uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];  // 解包后的数据,调用CANCommGet()后cast成对应的类型通过指针读取即可
  /* 接收和更新标志位*/
  uint8_t recv_state;    // 接收状态,
  uint8_t cur_recv_len;  // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
  uint8_t update_flag;   // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0

  DaemonInstance *comm_daemon;
} CANCommInstance;
#pragma pack()

//can通信数据结构体
typedef struct
{
  //can设备
  CAN_HandleTypeDef *can_handle;
  //can通信目标
  can_comm_target_e can_comm_target;
  //can发送数据句柄
  CAN_TxHeaderTypeDef transmit_message;
  //通信数据
  uint8_t data[8];
}can_comm_data_t;

//can设备通信队列
typedef struct
{
  //通信数据队列存储缓存
  can_comm_data_t can_comm_data[CAN_COMM_QUEUE_CAPACITY];
  //队列容量
  int capacity;
  //队列数据量
  int size;
  //头指针
  int head;
  //尾指针
  int tail;
}can_comm_queue_t;

/* CAN comm 初始化结构体 */
typedef struct {
  CAN_Init_Config_s can_config;  // CAN初始化结构体
  uint8_t send_data_len;         // 发送数据长度
  uint8_t recv_data_len;         // 接收数据长度

  uint16_t daemon_count;  // 守护进程计数,用于初始化守护进程
} CANComm_Init_Config_s;

/**
 * @brief 初始化CANComm
 *
 * @param config CANComm初始化结构体
 * @return CANCommInstance*
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);

/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 */
void CANCommSend(CANCommInstance *instance, uint8_t *data);

/**
 * @brief 获取CANComm接收的数据,需要自己使用强制类型转换将返回的void指针转换成指定类型
 *
 * @return void* 返回的数据指针
 * @attention 注意如果希望直接通过转换指针访问数据,如果数据是union或struct,要检查是否使用了pack(n)
 *            CANComm接收到的数据可以看作是pack(1)之后的,是连续存放的.
 *            如果使用了pack(n)可能会导致数据错乱,并且无法使用强制类型转换通过memcpy直接访问,转而需要手动解包.
 *            强烈建议通过CANComm传输的数据使用pack(1)
 */
void *CANCommGet(CANCommInstance *instance);

/**
 * @brief  can设备通信函数,发送can通信数据结构体内数据
 *
 * @param can_commit_data can通信数据
 */
void can_transmit(can_comm_data_t* can_commit_data);

/**
 * @brief 检查CANComm是否在线
 *
 * @param instance
 * @return uint8_t
 */
uint8_t CANCommIsOnline(CANCommInstance *instance);

/**
 * @brief can通信队列开辟函数
 *
 * @param queue_capacity 队列空间大小
 * @return can_comm_queue_t* 返回开辟队列指针
 */
can_comm_queue_t *can_comm_queue_init();

/**
 * @brief can通信队列添加, 数据添加使用memcpy进行内存拷贝，无需担心添加数据会变
 *
 * @param comm_queue can通信队列结构体
 * @param can_comm_data can通信队列数据
 * @return bool_t 添加成功返回true，添加失败返回false
 */
bool can_comm_queue_push(can_comm_queue_t *comm_queue, const can_comm_data_t *can_comm_data);

/**
 * @brief can通信队列出队
 *
 * @param comm_queue
 * @return can_comm_data_t* 返回出队元素指针，如果队空则返回空指针
 */
can_comm_data_t *can_comm_queue_pop(can_comm_queue_t *comm_queue);

/**
 * @brief 返回队列大小
 *
 * @param comm_queue 队列结构体
 * @return 返回队列大小，类型为整形
 */
int can_comm_queue_size(can_comm_queue_t *comm_queue);

/**
 * @brief 判断是否发生队空
 *
 * @param comm_queue can通信队列
 * @return true 是队空
 * @return false 不是队空
 */
bool can_comm_queue_is_empty(can_comm_queue_t *comm_queue);

#endif  // !CAN_COMM_H