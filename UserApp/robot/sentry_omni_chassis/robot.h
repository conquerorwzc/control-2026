#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "master_process.h"
#include "navigator.h"
#include "new_RC_VT13.h"
#include "remote_control.h"
#include "rm_referee.h"
#include "shoot.h"
#include "super_cap.h"
// todo: add vision_module
#define CHASSIS_BOARD
typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON,
} Robot_Mode_e;

// 定义枚举体，包含自动模式和手动模式
typedef enum {
  MANUAL_MODE=0,   // 手动控制
  NAVIGATOR_MODE,    // 自动控制
} Control_Mode_e;

#pragma pack(1)
typedef struct {
  uint16_t initial_speed; // 弹速
  uint16_t shooter_17mm_barrel_heat;//17mm发射机构的射击热量
} Referee_Main_s;

typedef struct {
  float wz;
} Chassis_Motion_s;

typedef struct {
  // uint16_t projectile_allowance_17mm;//17mm弹丸允许发弹量
  // uint16_t shooter_barrel_cooling_value;//机器人射击热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;//机器人射击热量上限
  uint8_t robot_id;  // 本机器人ID（红蓝阵营）
} Referee_Game_State_s;

#ifdef USE_DUAL_RC
typedef struct {
  int16_t Rc_vx;
  int16_t Rc_vy;
  float Rotate_speed;
  int16_t Spin_speed;
  float Yaw_motor_angle;
  uint8_t rc_switch_left;
  uint8_t rc_switch_right;
  // uint8_t Control_mode;
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
} Send_Data_RC_NEW;
#endif

typedef struct {
   union{
    uint32_t RFID1;
    struct {
      uint32_t base_boost : 1;                        // bit0: 己方基地增益点
      uint32_t own_center_highland_boost : 1;         // bit1: 己方中央高地增益点
      uint32_t enemy_center_highland_boost : 1;       // bit2: 对方中央高地增益点
      uint32_t own_trapezoid_highland_boost : 1;      // bit3: 己方梯形高地增益点
      uint32_t enemy_trapezoid_highland_boost : 1;    // bit4: 对方梯形高地增益点
      uint32_t own_ramp_before_boost : 1;             // bit5: 己方地形跨越增益点(飞坡)(靠近己方一侧飞坡前)
      uint32_t own_ramp_after_boost : 1;              // bit6: 己方地形跨越增益点(飞坡)(靠近己方一侧飞坡后)
      uint32_t enemy_ramp_before_boost : 1;           // bit7: 对方地形跨越增益点(飞坡)(靠近对方一侧飞坡前)
      uint32_t enemy_ramp_after_boost : 1;            // bit8: 对方地形跨越增益点(飞坡)(靠近对方一侧飞坡后)
      uint32_t own_center_highland_lower_boost : 1;   // bit9: 己方地形跨越增益点(中央高地下方)
      uint32_t own_center_highland_upper_boost : 1;   // bit10: 己方地形跨越增益点(中央高地上方)
      uint32_t enemy_center_highland_lower_boost : 1; // bit11: 对方地形跨越增益点(中央高地下方)
      uint32_t enemy_center_highland_upper_boost : 1; // bit12: 对方地形跨越增益点(中央高地上方)
      uint32_t own_road_lower_boost : 1;              // bit13: 己方地形跨越增益点(公路下方)
      uint32_t own_road_upper_boost : 1;              // bit14: 己方地形跨越增益点(公路上方)
      uint32_t enemy_road_lower_boost : 1;            // bit15: 对方地形跨越增益点(公路下方)
      uint32_t enemy_road_upper_boost : 1;            // bit16: 对方地形跨越增益点(公路上方)
      uint32_t own_fortress_boost : 1;                // bit17: 己方堡垒增益点
      uint32_t own_outpost_boost : 1;                 // bit18: 己方前哨站增益点
      uint32_t own_supply_zone_boost : 1;             // bit19: 己方与资源区不重叠的补给区/RMUL补给区
      uint32_t own_overlap_supply_boost : 1;          // bit20: 己方与资源区重叠的补给区
      uint32_t own_assembly_boost : 1;                // bit21: 己方装配增益点
      uint32_t enemy_assembly_boost : 1;              // bit22: 对方装配增益点
      uint32_t center_boost_rmull : 1;                // bit23: 中心增益点(仅RMUL适用)
      uint32_t enemy_fortress_boost : 1;              // bit24: 对方堡垒增益点
      uint32_t enemy_outpost_boost : 1;               // bit25: 对方前哨站增益点
      uint32_t own_tunnel_boost_road_lower : 1;       // bit26: 己方地形跨越增益点(隧道)(靠近己方一侧公路区下方)
      uint32_t own_tunnel_boost_road_middle : 1;      // bit27: 己方地形跨越增益点(隧道)(靠近己方一侧公路区中间)
      uint32_t own_tunnel_boost_road_upper : 1;       // bit28: 己方地形跨越增益点(隧道)(靠近己方一侧公路区上方)
      uint32_t own_tunnel_boost_trapezoid_low : 1;    // bit29: 己方地形跨越增益点(隧道)(靠近己方梯形高地较低处)
      uint32_t own_tunnel_boost_trapezoid_mid : 1;    // bit30: 己方地形跨越增益点(隧道)(靠近己方梯形高地较中间)
      uint32_t own_tunnel_boost_trapezoid_high : 1;   // bit31: 己方地形跨越增益点(隧道)(靠近己方梯形高地较高处)
    } fields;
  } RFID1_t;
   union{
    uint8_t RFID2;
    struct {
      uint8_t enemy_tunnel_boost_road_lower : 1;   // bit0: 对方地形跨越增益点(隧道)(靠近对方公路一侧下方)
      uint8_t enemy_tunnel_boost_road_middle : 1;  // bit1: 对方地形跨越增益点(隧道)(靠近对方公路一侧中间)
      uint8_t enemy_tunnel_boost_road_upper : 1;   // bit2: 对方地形跨越增益点(隧道)(靠近对方公路一侧上方)
      uint8_t enemy_tunnel_boost_trapezoid_low : 1;// bit3: 对方地形跨越增益点(隧道)(靠近对方梯形高地较低处)
      uint8_t enemy_tunnel_boost_trapezoid_mid : 1;// bit4: 对方地形跨越增益点(隧道)(靠近对方梯形高地较中间)
      uint8_t enemy_tunnel_boost_trapezoid_high : 1;// bit5: 对方地形跨越增益点(隧道)(靠近对方梯形高地较高处)
      uint8_t reserved : 2;                         // 保留位(用于未来扩展)
    } fields;
  } RFID2_t;
} RFID_Status_t;

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

typedef enum {
  SAFETY_MODE=0,
  PASSIVE_MODE,
  ACTIVE_MODE,
  CHARGING_MODE,
  FORCED_CHARGING_MODE,
} SuperCapMode;

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
  navigator_recv_t* navigator_data;    //从导航获取的控制指令

  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;
} RobotInstance;

Sentry_Cmd_t *SentryUpdate();

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