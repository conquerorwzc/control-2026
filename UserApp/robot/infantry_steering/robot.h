#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "remote_control.h"
#include "super_cap.h"
// #include "rm_referee.h"
#include "rm_referee.h"
// todo: add vision_module
#
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

  RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
   referee_info_t* referee_data;     // 用于获取裁判系统的数据

  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;

} RobotInstance;
#include "can_comm.h"
#include "bsp_can.h"
//#define DEVICE_ROLE_TX 1 // 发送板：定义为1
#define DEVICE_ROLE_TX 0 // 接收板：注释上一行，定义为0
#if DEVICE_ROLE_TX
#define DEVICE_ROLE_STR "TX"
#else
#define DEVICE_ROLE_STR "RX"
#endif
//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 100
//can通信任务运行时间间隔 单位ms
#define CAN_COMM_TASK_TIME 50 //1
//云台can设备
#define GIMBAL_CAN hcan1
//双板can通信设备
#define BOARD_CAN hcan1
//发弹can通信设备
#define SHOOT_CAN hcan2
//裁判系统can通信
#define SHOOT_FLAGS_CAN hcan1
//发送云台pitch轴的相对角和绝对角can通信
#define PitchAngle_CAN hcan1
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