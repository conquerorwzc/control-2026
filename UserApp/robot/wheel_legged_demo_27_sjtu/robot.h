#pragma once

#include "can_comm.h"
#include "comm.h"
#include "chassis.h"
#include "gimbal.h"
#include "master_process.h"
#include "referee.h"
#include "remote_control.h"
#include "shoot.h"
#include "super_cap.h"

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_CHASSIS_ROTATE,
  ROBOT_CHASSIS_FOLLOW,
  ROBOT_CHASSIS_FREE,
  ROBOT_CHASSIS_PROSTRATE_ROTATE,
  ROBOT_CHASSIS_PROSTRATE_FOLLOW,
} Robot_Mode_e;

typedef struct RobotInstance {
  Robot_Mode_e robot_mode;  // 机器人整体工作状态

#ifdef USE_RC_CTRL
  RC_ctrl_t* rc_data;  // 遥控器数据,初始化时返回
#elifdef USE_OCD_CTRL
  VT13_RC_t* rc_data;
#endif
  referee_info_t* referee_data;  // 用于获取裁判系统的数据

  float offset_angle;

  SuperCapInstance* super_cap;
  ChassisInstance* chassis;
  GimbalInstance* gimbal;
  ShootInstance* shoot;

  Vision_Receive_s* vision_recv_data;

#ifndef ONE_BOARD
  Chassis_Upload_Data_s* chassis_upload_data;  // 此处chassis定义为chassis_board, 而非chassis模组, 故所有处理在robot中
  Chassis_Fetch_Data_s* chassis_fetch_data;
  CANCommInstance* main_comm;
  CANCommInstance* motion_comm;
  CANCommInstance* gamestate_comm;
#endif

  uint32_t DWT_CNT;
  float dt;

  struct {
    uint8_t is_gimbal_aligned : 1;  // 云台是否已与底盘正方向对齐；双板时由 DoubleBoardComms 在 CAN 上同步
  } update_flag;
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

RobotInstance* RobotGetInstance(void);
