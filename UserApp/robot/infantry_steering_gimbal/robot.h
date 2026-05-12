#pragma once

#include <stdint.h>
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#ifdef USE_VT13
#include "new_RC_VT13.h"
#else
#include "remote_control.h"
#endif
#include "rm_referee.h"
#include "super_cap.h"
//#include "can_comm.h"

// todo: add vision_module

#define GIMBAL_BOARD

//联合体定义
typedef union {
  int16_t value;
  uint8_t bytes[2];
} Int16ToBytes;

//上传数据结构体，供master_process访问
//#pragma pack(1)
// 舵轮底盘控制命令
typedef struct {
  // 控制部分
  float vx;            // 前进方向速度
  float vy;            // 横移方向速度
  float wz;            // 旋转速度
  Chassis_Mode_e chassis_mode;
  float offset_angle;  // 底盘和归中位置的夹角
  uint8_t SuperCapBoost;
  //int chassis_speed_buff;
  //uint16_t max_power;  // 最大功率限制
  // UI部分
  //  ...
} Chassis_Ctrl_CanComm;
//#pragma pack()

#pragma pack(1)
typedef struct {
  Chassis_Ctrl_CanComm chassis_ctrl_can_comm;
  uint8_t gimbal_mode;
  uint8_t shoot_mode;
  uint8_t friction_mode;
  uint8_t load_mode;
  int16_t pitch;
  uint8_t rest_heat;
  uint8_t shoot_rate;
  uint16_t friction_speed1;
  uint16_t friction_speed2;
} CanComm_Pack;
#pragma pack()
typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

typedef struct {
  Robot_Mode_e robot_mode;       // 机器人整体工作状态



#ifdef USE_VT13
  VT13_RC_t *rc_data;               // VT13遥控器数据,初始化时返回
#else
  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
#endif
  referee_info_t* referee_data;     // 用于获取裁判系统的数据
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