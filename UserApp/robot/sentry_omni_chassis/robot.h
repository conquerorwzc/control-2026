#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "remote_control.h"
#include "navigator.h"
#include "master_process.h"
#include "rm_referee.h"
#include "super_cap.h"
// todo: add vision_module

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON,
} Robot_Mode_e;

// 定义枚举体，包含自动模式和手动模式
typedef enum {
  MANUAL_MODE=0,   // 手动控制
  AUTO_MODE,    // 自动控制
} Control_Mode_e;

#pragma pack(1)
typedef union{
  // 方式 1：直接作为 4 字节数组访问（用于底层串口发送/接收）
  uint8_t raw_data[4];

  // 方式 2：作为整个 32 位整型访问
  uint32_t value;

  // 方式 3：结构化访问具体含义（位域）
  struct {
    /** * bit 0: 哨兵机器人是否确认复活
     * 0: 不确认, 1: 确认
     */
    uint32_t confirm_respawn : 1;

    /** * bit 1: 哨兵机器人是否确认兑换立即复活
     * 0: 不兑换, 1: 确认兑换
     */
    uint32_t confirm_instant_respawn : 1;

    /** * bit 2-12: 哨兵将兑换的允许发弹量 (11 bits)
     * 范围: 0 ~ 2047. 需单调递增，否则视为非法
     */
    uint32_t projectile_amount : 11;

    /** * bit 13-16: 哨兵远程兑换发弹量的请求次数 (4 bits)
     * 范围: 0 ~ 15. 需单调递增
     */
    uint32_t projectile_req_cnt : 4;

    /** * bit 17-20: 哨兵远程兑换血量的请求次数 (4 bits)
     * 范围: 0 ~ 15. 需单调递增
     */
    uint32_t hp_req_cnt : 4;

    /** * bit 21-22: 哨兵修改当前姿态指令 (2 bits)
     * 1: 进攻, 2: 防御, 3: 移动. 默认 3
     */
    uint32_t sentry_mode : 2;

    /** * bit 23: 哨兵机器人是否确认使能能量机关进入激活状态 (1 bit)
     * 0: 默认, 1: 确认
     */
    uint32_t activate_power_rune : 1;

    /** * bit 24-31: 保留位 (8 bits)
     * 凑齐 32 位，通常置 0
     */
    uint32_t reserved : 8;
  } __attribute__((packed)) fields;
}__attribute__((packed)) Sentry_Cmd_t;
#pragma pack()

typedef enum {
  OFFENSE_POSE=1,
  DEFENSE_POSE,
  MOBILITY_POSE,
} Sentry_Mode_e;



typedef struct {
  Robot_Mode_e robot_mode;       // 机器人工作状态
  Control_Mode_e control_mode;   // 控制模式
  Sentry_Mode_e sentry_mode;    //哨兵姿态

  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
  referee_info_t* referee_data;     // 用于获取裁判系统的数据
  Vision_Receive_s* vision_recv_data;
  navigator_recv_t* navigator_data;    //从导航获取的控制指令

  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;

} RobotInstance;


/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void RobotTask();