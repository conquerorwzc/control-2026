//
// Created by zeg on 2025/12/3.
//

#pragma once

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "rm_referee.h"
#include "super_cap.h"
// todo: add vision_module

// ---------------- 条件编译遥控器头文件 ----------------
#if defined(USE_DUAL_RC_NEW)
#include "new_RC_VT13.h"
#elif defined(USE_DUAL_RC)
#include "remote_control.h"
#endif

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

typedef struct {
  Robot_Mode_e robot_mode;       // 机器人整体工作状态

  // ---------------- 条件编译遥控器数据指针类型 ----------------
#if defined(USE_DUAL_RC_NEW)
  VT13_RC_t *rc_data;               // VT13遥控器数据指针
#elif defined(USE_DUAL_RC)
  RC_ctrl_t *rc_data;               // 旧版DJI遥控器数据指针
#else
  void *rc_data;                    // 未定义宏时的缺省回退
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

RobotInstance* getRobot();