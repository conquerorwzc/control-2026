#pragma once

/**
 ******************************************************************************
 * @file    robot.h
 * @brief   arm_rmcc —— Yaw-Pitch-Pitch 场地臂（两轴 pitch，DM-J4310）
 *
 * 本文件是整车的抽象。main 只需要包含它。
 * 布局与其它 robot 一致：robot.h 提供公共头 + RobotInstance，
 * robot_config.h 放全部可调参数，robot.c 放业务逻辑。
 *
 * 业务逻辑全部在 robot.c 的 RobotTask() 内：不新增任务、不改动其它层。
 ******************************************************************************
 */

#include "bsp_usart.h"
#include "dmmotor.h"
#include "general_def.h"
#include "usart.h"      /* huart1/huart5 的 extern 声明（CubeMX 生成） */

/* 全部可调参数与编译期模式开关都在这里。
 * 与其它 robot 的布局一致：robot.h 是公共入口，robot_config.h 放配置。 */
#include "robot_config.h"

/** 场地臂的运行模式。与 robot_config.h 里的编译期开关对应。 */
typedef enum {
  ARM_IDLE = 0,   /* 未启动 / 已结束：零力矩 */
  ARM_IDENT,      /* 辨识：跑位姿扫描并把数据经 USART 上报 */
  ARM_HOLD,       /* 部署：重力前馈 + 低刚度阻抗 */
} Arm_Mode_e;

typedef struct {
  Arm_Mode_e mode;

  DMMotorInstance *joint_motor[2]; /* [0] = 关节1（肩）, [1] = 关节2（肘） */
  USARTInstance *ident_usart;      /* J-Link VCOM 那一路，单向发送 */

  float q_zero[2];                 /* 契约零位（rad），见 robot_config.h */
  float q_sign[2];                 /* +1 / -1，必须实测 */
} RobotInstance;

/**
 * @brief 机器人初始化。由 os_task.c 的 StartROBOTTASK() 调用，
 *        且在 DMMotorTaskInit() 之前 —— 因此 DM 电机必须在这里注册完。
 */
void RobotInit();

/**
 * @brief 机器人任务。由 StartROBOTTASK() 以 500 Hz（osDelay(2)）运行。
 *        辨识扫描、力矩前馈、数据上报全部在这里，不另开线程。
 */
void RobotTask();
