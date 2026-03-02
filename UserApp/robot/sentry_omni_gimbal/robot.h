#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "remote_control.h"
#include "navigator.h"
#include "master_process.h"
// #include "rm_referee.h"
#include "super_cap.h"
#include "can_comm.h"
// todo: add vision_module

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

// 定义枚举体，包含自动模式和手动模式
typedef enum {
  MANUAL_MODE=0,   // 手动控制
  AUTO_MODE,    // 自动控制
} Control_Mode_e;
//联合体定义
typedef struct {
  uint16_t projectile_allowance_17mm;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_barrel_heat;
  uint16_t shooter_barrel_heat_limit;
  uint16_t shooter_barrel_cooling_value;
} Referee_Data;
#pragma pack(1)
#ifdef USE_DUAL_RC
typedef struct {
  int16_t Rc_vx;
  int16_t Rc_vy;
  float Rc_yaw;
  int16_t Rc_vw;
  float Yaw_single_round;
  uint8_t Switch_right;
} Send_Data_RC;
#elifdef USE_DUAL_RC_NEW
typedef struct {
  int16_t Rc_vx;
  int16_t Rc_vy;
  float Rc_yaw;
  int16_t Rc_vw;
  float Yaw_single_round;
  float Mode_switch;
  float Control_mode;
  float Pause_flag;
} Send_Data_RC_NEW;
#endif

#pragma pack()


typedef struct {
  Robot_Mode_e robot_mode;       // 机器人工作状态
  Control_Mode_e control_mode;   // 控制模式

  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
  // referee_info_t* referee_data;     // 用于获取裁判系统的数据
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