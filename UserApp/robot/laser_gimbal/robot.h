//
// Created by yang6 on 2026/3/3.
//

#ifndef CONTROL_2026_LASER_GIMBAL_H
#define CONTROL_2026_LASER_GIMBAL_H
#include "gimbal.h"
#include "remote_control.h"
// #include "robot.h"
typedef enum {
    ROBOT_POWER_OFF = 0,
    ROBOT_POWER_ON ,
  } Robot_Mode_e;
typedef struct {
    Robot_Mode_e robot_mode;       // 机器人整体工作状态

    RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回

    GimbalInstance* gimbal;

} RobotInstance;
#endif // CONTROL_2026_LASER_GIMBAL_H
