#pragma once
// #define USE_DUAL_RC_NEW
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#ifdef USE_DUAL_RC_NEW
#include "new_RC_VT13.h"
#else
#include "remote_control.h"
#endif
 #include "rm_referee.h"
#include "super_cap_HKUST/super_cap_HKUST.h"
// todo: add vision_module

typedef enum {
  ROBOT_POWER_OFF = 0,
  ROBOT_POWER_ON ,
} Robot_Mode_e;

typedef struct {
  Robot_Mode_e robot_mode;

#ifdef USE_DUAL_RC_NEW
  VT13_RC_t *vt13_rc_data;
#else
  RC_ctrl_t *rc_data;
#endif
  referee_info_t* referee_data;

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
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务/
 *
 */
void RobotTask();

RobotInstance* RobotGet();