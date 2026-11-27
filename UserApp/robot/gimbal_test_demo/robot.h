#pragma once

#include <stdint.h>

#include "can_comm.h"
#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"
#include "shoot.h"
#include "super_cap.h"

#define GIMBAL_BOARD

typedef union {
    int16_t value;
    uint8_t bytes[2];
} Int16ToBytes;

typedef enum {
    ROBOT_POWER_OFF = 0,
    ROBOT_POWER_ON,
  } Robot_Mode_e;

typedef struct {
    Robot_Mode_e robot_mode;  // 机器人整体工作状态

    RC_ctrl_t *rc_data;  // 遥控器数据,初始化时返回

    SuperCapInstance *super_cap;
    ChassisInstance *chassis;
    GimbalInstance *gimbal;
    ShootInstance *shoot;

    /*  新增: 时间基准 (与 infantry_wheel_legged_sjtu 一致) */
    float dt;          // 本周期时间间隔, 单位 s
    uint32_t DWT_CNT;  // DWT 周期计数, 供 DWT_GetDeltaT 使用
} RobotInstance;

void RobotInit();
void RobotTask();