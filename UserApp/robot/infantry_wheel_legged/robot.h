#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"
#include "shoot.h"
#include "rm_referee.h"
#include "can_comm.h"
#include "master_process.h"
#include "navigator.h"
#include "super_cap.h"
#include "parallel_leg.h"
#include "vofa.h"

// todo: add vision_module

#ifndef ONE_BOARD
#pragma pack(1)
typedef struct {
  float Roll;           // 横滚角(绕X轴旋转) 单位: °
  float Pitch;          // 俯仰角(绕Y轴旋转) 单位: °
  float YawTotalAngle;  // Yaw轴累计转过的总角度，可用于多圈控制 单位: °
  float YawSpeed;       // yaw角速度，单位: rad/s
  // 后续增加底盘的真实速度
  // float real_vx;
  // float real_vy;
  // float real_wz;
  // uint8_t rest_heat;            // 剩余枪口热量
  // Bullet_Speed_e bullet_speed;  // 弹速限制
  // Enemy_Color_e enemy_color;    // 0 for blue, 1 for red
  // 裁判系统数据
  float bullet_speed;
} Chassis_Upload_Data_s;  // means the Chassis board, not the component

typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
} Chassis_Fetch_Data_s;  // means the Chassis board, not the component
#pragma pack()

#endif

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_CHASSIS_ROTATE,
  ROBOT_CHASSIS_FOLLOW,
  ROBOT_CHASSIS_FREE,
} Robot_Mode_e;

// 静态变量用于边沿检测
static struct {
  uint8_t q;      // 摩擦轮开关
  uint8_t space;  // 跳跃
  uint8_t v;      // 小陀螺模式切换
  uint8_t shift;  // 加速
} key_last_count;

typedef struct {
  Robot_Mode_e robot_mode;  // 机器人整体工作状态
  RC_ctrl_t* rc_data;       // 遥控器数据,初始化时返回
  referee_info_t* referee_data;     // 用于获取裁判系统的数据
  float offset_angle;
  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;
  Vision_Receive_s* vision_recv_data;
  navigator_recv_t* navigator_data;
  PIDInstance chassis_follow_PID;
  PIDInstance chassis_rotate_PID;
#ifndef ONE_BOARD
  Chassis_Upload_Data_s* chassis_upload_data;  // 此处chassis定义为chassis_board, 而非chassis模组, 故所有处理在robot中
  Chassis_Fetch_Data_s* chassis_fetch_data;
  CANCommInstance* can_comm;
#endif
  uint32_t DWT_CNT;
  float dt;
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