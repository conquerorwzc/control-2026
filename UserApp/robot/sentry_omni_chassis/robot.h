#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "master_process.h"
#include "navigator.h"
#include "new_RC_VT13.h"
#include "remote_control.h"
#include "shoot.h"
#include "super_cap.h"
// todo: add vision_module

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON,
} Robot_Mode_e;

// 定义枚举体，包含自动模式和手动模式
typedef enum {
  MANUAL_MODE=0,   // 手动控制
  NAVIGATOR_MODE,    // 自动控制
} Control_Mode_e;

/* ===========================================================================
 * 双板 CAN2 载荷 (chassis 与 gimbal 两份定义必须逐字节一致, 见文末静态断言)
 *
 *   0x210  云台板 -> 底盘板  Send_Data_RC : 遥控通道 + 导航速度指令
 *   0x211  底盘板 -> 云台板  Referee_Data : 发射机构转发 + 底盘速度观测反馈
 *
 * 导航小电脑接在云台板 USART1, 因此导航指令由云台板解出后经 0x210 转给底盘板;
 * 底盘观测到的融合速度经 0x211 回传, 由云台板组装成 0x0010 反馈包发出。
 * =========================================================================== */
#pragma pack(1)

/* 底盘板 -> 云台板 */
typedef struct {
  // uint16_t projectile_allowance_17mm;//17mm弹丸允许发弹量
  uint16_t initial_speed; // 弹速
  uint16_t shooter_17mm_barrel_heat;//17mm发射机构的射击热量
  uint8_t robot_id;  // 本机器人ID（红蓝阵营）
  // uint16_t shooter_barrel_heat_limit;//机器人射击热量上限
  // uint16_t shooter_barrel_cooling_value;//机器人射击热量每秒冷却值
  // 底盘速度观测反馈, 供导航 0x0010 帧的 vx/vy/wz
  float fb_vx;   // 底盘系融合速度 m/s
  float fb_vy;   // 底盘系融合速度 m/s
  float fb_wz;   // 底盘系融合角速度 rad/s
} Referee_Data;

/* 云台板 -> 底盘板 */
#ifdef USE_DUAL_RC
typedef struct {
  int16_t Rc_vx;
  int16_t Rc_vy;
  float Rotate_speed;
  int16_t Spin_speed;
  float Yaw_motor_angle;
  uint8_t Switch_right;
  uint8_t Switch_left;
  // 导航速度指令, 由云台板 USART1 收包后转发; 底盘板在 NAVIGATOR_MODE 下使用
  float nav_vx;    // m/s
  float nav_vy;    // m/s
  float nav_wz;    // rad/s
  uint8_t nav_valid;  // 1: 500ms 看门狗内收到过有效帧; 0: 导航离线, 底盘按零速处理
} Send_Data_RC;
#elifdef USE_DUAL_RC_NEW
typedef struct {
  int16_t Rc_vx;
  int16_t Rc_vy;
  float Rotate_speed;
  int16_t Spin_speed;
  float Yaw_motor_angle;
  uint8_t Mode_switch;
  uint8_t Control_mode;
  uint8_t Pause_flag;
  float nav_vx;
  float nav_vy;
  float nav_wz;
  uint8_t nav_valid;
} Send_Data_RC_NEW;
#endif
#pragma pack()

/* 线上长度必须双方一致; 与 can_comm 的 send/recv_data_len 配套 */
_Static_assert(sizeof(Referee_Data) == 17, "下行包(底盘->云台)必须为 17 字节");
#ifdef USE_DUAL_RC
_Static_assert(sizeof(Send_Data_RC) == 29, "上行包(云台->底盘)必须为 29 字节");
#define Send_Data_Up                                    Send_Data_RC
#elifdef USE_DUAL_RC_NEW
_Static_assert(sizeof(Send_Data_RC_NEW) == 30, "上行包(云台->底盘)必须为 30 字节");
#define Send_Data_Up                                    Send_Data_RC_NEW
#endif

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

  #ifdef USE_DUAL_RC
    RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
  #elifdef USE_DUAL_RC_NEW
    VT13_RC_t *vt13_rc_data;
  #endif
  referee_info_t* referee_data;     // 用于获取裁判系统的数据
  Vision_Receive_s* vision_recv_data;
  // 导航小电脑接在云台板 USART1, 底盘板不再直接持有导航串口实例;
  // 导航速度指令经 CAN2 0x210 的 nav_vx/nav_vy/nav_wz/nav_valid 字段获取。
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