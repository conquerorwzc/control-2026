#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "crc_func.h"
#include "referee.h"
#include "string.h"
#include "usart.h"

/* ===========================================================================
 * SRM 校内赛导航通信协议 V1.0.0 —— 下位机侧实现
 *
 * 串口: USART1, 115200 8N1, 无流控; 所有整数与浮点数一律小端(little-endian)
 *
 * 上 -> 下 (固定 19 B, 帧头之后直接是数据段, 没有 cmd_id):
 *   [0]      0xA5
 *   [1..2]   data_length = 12 (uint16 LE), 下位机只接受 12
 *   [3]      seq (上位机流水号, 下位机不校验)
 *   [4]      crc8  = CRC8(byte[0..3],   init 0xFF, poly 0x8C)
 *   [5..16]  vx, vy, wz (float32 LE)
 *   [17..18] crc16 = CRC16(byte[0..16], init 0xFFFF, poly 0x8408), 低字节在前
 *
 * 下 -> 上 (9 + data_length B, 帧头之后多一个 2 B 的 cmd_id):
 *   [0]      0xA5
 *   [1..2]   data_length (uint16 LE)
 *   [3]      seq
 *   [4]      crc8  = CRC8(byte[0..3],   init 0xFF, poly 0x8C)
 *   [5..6]   cmd_id (uint16 LE)
 *   [7..]    data (data_length 字节)
 *   [末 2]   crc16 = CRC16(byte[0 .. 7+len-1], init 0xFFFF), 低字节在前, 含 cmd_id
 *   cmd_id = 0x0001 比赛状态(11 B) / 0x000B 机器人状态(13 B), 每个包独立成帧
 *
 * 自检: CRC8("123456789") = 0x0B, CRC16("123456789") = 0x6F91
 * =========================================================================== */

/* 帧头与校验 */
#define PROTOCOL_SOF        0xA5
#define PROTOCOL_HEADER_LEN 5
#define PROTOCOL_CMD_ID_LEN 2
#define PROTOCOL_CRC8_INIT  0xFF
#define PROTOCOL_CRC16_INIT 0xFFFF
#define BUFFER_MAX_SIZE     256
#define NAVIGATOR_RECV_SIZE 64

/* 上 -> 下: 速度指令帧 */
#define NAV_CMD_DATA_LEN  12                                                    /* data_length 固定 12 */
#define NAV_CMD_FRAME_LEN (PROTOCOL_HEADER_LEN + NAV_CMD_DATA_LEN + 2)          /* 整帧 19 B */

/* 下 -> 上: 反馈包 cmd_id 与数据段长度 */
#define NAV_CMD_ID_GAME_STATUS   0x0001u /* 比赛状态 */
#define NAV_CMD_ID_ROBOT_STATUS  0x000Bu /* 机器人状态 */
#define NAV_GAME_STATUS_DATA_LEN  11
#define NAV_ROBOT_STATUS_DATA_LEN 13

/* 时序参数 */
#define NAV_CMD_TIMEOUT_MS      500 /* 看门狗: 超过该时间未收到有效帧 -> 速度归零(协议 §6.1.7) */
#define NAV_FEEDBACK_PERIOD_MS  10  /* 反馈帧节拍 10ms, 两个包轮流发 -> 各自 50Hz(协议 §6.2.4) */
#define NAV_DAEMON_RELOAD_COUNT 50  /* daemon 任务 100Hz, 50 拍 = 500ms 未收到有效帧即判离线 */

#pragma pack(push, 1)

/* 帧头, 两个方向一致, 共 5 字节 */
typedef struct {
  uint8_t sof;          /* 固定 0xA5 */
  uint16_t data_length; /* 数据段长度, 小端 */
  uint8_t seq;          /* 包流水号 */
  uint8_t crc8;         /* 对偏移 0-3 计算, 初值 0xFF */
} __attribute__((packed)) HeaderFrame;

/* 上 -> 下 数据段: 12 字节, 协议层不缩放/不限幅, 原样传给底盘 */
typedef struct {
  struct {
    float vx;
    float vy;
    float wz;
  } __attribute__((__packed__)) speed_vector;
} __attribute__((__packed__)) robot_cmd_t;

/* 下 -> 上 数据段: 比赛状态 (cmd_id = 0x0001, 11 字节, 紧凑布局无填充) */
typedef struct {
  uint8_t game_type : 4;     /* 低 4 位: 比赛类型 */
  uint8_t game_progress : 4; /* 高 4 位: 当前阶段 */
  uint16_t stage_remain_time;
  uint64_t sync_time_stamp;
} __attribute__((__packed__)) nav_game_status_t;

/* 下 -> 上 数据段: 机器人状态 (cmd_id = 0x000B, 13 字节) */
typedef struct {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_hp;
  uint16_t maximum_hp;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output : 1;  /* bit0, 1 = 24V 输出 */
  uint8_t power_management_chassis_output : 1; /* bit1 */
  uint8_t power_management_shooter_output : 1; /* bit2 */
} __attribute__((__packed__)) nav_robot_status_t;

#pragma pack(pop)

/* 线上帧结构体长度必须与协议一致, 改动数据段会在编译期直接报错 */
_Static_assert(sizeof(HeaderFrame) == PROTOCOL_HEADER_LEN, "HeaderFrame 必须为 5 字节");
_Static_assert(sizeof(robot_cmd_t) == NAV_CMD_DATA_LEN, "上->下数据段必须为 12 字节");
_Static_assert(sizeof(nav_game_status_t) == NAV_GAME_STATUS_DATA_LEN, "0x0001 数据段必须为 11 字节");
_Static_assert(sizeof(nav_robot_status_t) == NAV_ROBOT_STATUS_DATA_LEN, "0x000B 数据段必须为 13 字节");

/* 模块运行时状态(不是线上帧结构, 保持自然对齐) */
typedef struct {
  robot_cmd_t robot_cmd;    /* 最近一次校验通过的速度指令(协议原值, 未做量纲换算) */
  uint32_t last_update_ms;  /* 最近一次有效帧的到达时刻, HAL_GetTick() */
  uint32_t rx_frame_cnt;    /* 有效帧计数 */
  uint32_t error_cnt;       /* crc8/长度/crc16 出错的帧计数 */
  uint8_t seq;              /* 最近一帧的流水号(仅记录, 不参与有效性判断) */
  uint8_t data_valid;       /* 1: 指令仍在超时窗口内 */
} navigator_recv_t;

/**
 * @brief 导航模块初始化, 注册 USART1 接收与离线守护 daemon
 * @param usart_handle 导航串口句柄(C 板固定为 &huart1)
 * @return 接收数据结构体指针, 内容随接收实时更新
 */
navigator_recv_t *navigator_init(UART_HandleTypeDef *usart_handle);

/**
 * @brief 发送反馈包(0x0001 比赛状态 / 0x000B 机器人状态), 定频轮流发送
 * @param instance     导航串口句柄, 保留形参以兼容原有调用点(内部使用注册的串口实例)
 * @param referee_data 裁判系统数据, 未接裁判系统时其内容为 0, 仍然照常发帧
 * @note  本函数非阻塞, 可直接放在 1kHz 的机器人任务里调用
 */
void navigator_send(UART_HandleTypeDef *instance, referee_info_t *referee_data);

/**
 * @brief 取一次导航速度指令, 内含 500ms 看门狗
 * @param cmd 输出: 有效时写入最近一次有效指令; 超时则写入零速
 * @return 1: 超时窗口内收到过有效指令; 0: 超时(指令已置零)
 * @note  应用层必须在每次控制周期调用, 用返回值判断上位机是否在线
 */
uint8_t NavigatorGetCmd(robot_cmd_t *cmd);

/**
 * @brief 协议自检: 校验 CRC 自检值 / 文档 §5 参考帧 / 长度与 CRC 拒绝逻辑
 * @note  仅在 Debug(ESC_DEBUG) 构建下有效, 通过 RTT 输出结果
 */
void NavigatorSelfTest(void);

#endif
