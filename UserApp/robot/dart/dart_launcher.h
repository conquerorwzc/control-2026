/**
 * @file dart_launcher.h
 * @brief dart 发射架执行层: 4 电机(1xM2006 yaw + 2xM3508 同步带 + 1xM3508 扳机)的状态机与闭环控制
 *
 * 分步操作流程(由上层命令驱动):
 *   1. 上电使能后自动执行一次朝向释放方向的零位校准(堵转检测, 同步两带电机零点,
 *      校准完成前严格限制带电机力矩);
 *   2. 储能命令: 扳机先到卡位 -> 双带电机同步拉拽发射平台 -> 被扳机卡住 -> 挡块回撤;
 *   3. 就绪(READY)后可随时调射力(扳机卡位);
 *   4. 发射命令: 扳机移出卡位释放发射平台, 随后自动回卡位待下一发。
 */
#pragma once

#include "dji_motor.h"
#include <stdbool.h>
#include <stdint.h>

/* 发射架状态 */
typedef enum {
  DART_STATE_IDLE = 0,   // 待机: 挡块在回缩位, 扳机在卡位, 未储能(或刚发射完)
  DART_STATE_CALIBRATING,  // 释放方向零位校准中
  DART_STATE_CHARGING,     // 储能序列中
  DART_STATE_READY,        // 储能完成, 等待发射
  DART_STATE_FIRING,       // 发射序列中
  DART_STATE_FAULT,        // 故障(超时/异常), 电机停机, 需重新校准恢复
} Dart_State_e;

/* 上层命令 */
typedef enum {
  DART_CMD_NONE = 0,
  DART_CMD_CALIBRATE,  // 重新执行零位校准(故障恢复/手动重校)
  DART_CMD_CHARGE,     // 储能
  DART_CMD_FIRE,       // 发射
} Dart_Cmd_e;

/* 校准子步骤 */
typedef enum {
  CALI_STEP_DRIVE_TO_STOP = 0,  // 释放方向低力矩顶硬限位
  CALI_STEP_BACKOFF,            // 回撤到回缩位
} Dart_Cali_Step_e;

/* 储能子步骤 */
typedef enum {
  CHARGE_STEP_PREP_TRIGGER = 0,  // 扳机先到卡位(拉伸量决定射力)
  CHARGE_STEP_DRIVE,             // 双电机同步拉拽发射平台
  CHARGE_STEP_HOLD,              // 到位保持
  CHARGE_STEP_RETRACT,           // 挡块回撤出发射平台活动范围
} Dart_Charge_Step_e;

/* 发射子步骤 */
typedef enum {
  FIRE_STEP_RELEASE = 0,  // 扳机移出卡位, 释放发射平台
  FIRE_STEP_DWELL,        // 释放位保持
  FIRE_STEP_RESET,        // 扳机回卡位待下一发
} Dart_Fire_Step_e;

/* 堵转检测器(速度低于阈值持续指定时间) */
typedef struct {
  bool tracking;
  uint32_t start_ms;
} StallDetector_s;

/* 发射架实例 */
typedef struct {
  DJIMotorInstance* yaw_motor;      // M2006 发射架 yaw
  DJIMotorInstance* belt_motor[2];  // M3508 同步带电机 [0]左 [1]右, 同一位置目标严格同步
  DJIMotorInstance* trigger_motor;  // M3508 扳机

  Dart_State_e state;
  bool enabled;          // 遥控器使能标志(失能即全部停机)
  bool is_calibrated;    // 本次上电是否已完成零位校准
  bool zero_valid;       // 同步带零点是否有效(校准找到硬限位后置位)
  bool recovery_retract; // 储能中断后, 重新使能须先回撤挡块

  // 逻辑坐标零点
  float belt_zero_offset[2];  // 释放方向硬限位处的 total_angle
  float yaw_boot_angle;       // yaw 开机角度
  float trigger_boot_angle;   // 扳机开机角度

  // 控制目标(逻辑坐标)
  float belt_pos_target;     // 同步带目标 (deg, 正=储能方向)
  float yaw_angle_target;    // yaw 目标角 (deg, 相对开机)
  float trigger_catch_deg;   // 扳机卡位(射力, 可调)
  float trigger_target_deg;  // 扳机当前目标 (deg, 相对开机)
  float yaw_rate_cmd_dps;    // yaw 角速度指令 (deg/s)

  // 序列子步骤与计时
  Dart_Cali_Step_e cali_step;
  Dart_Charge_Step_e charge_step;
  Dart_Fire_Step_e fire_step;
  Dart_Cmd_e pending_cmd;
  uint32_t step_start_ms;
  uint32_t cali_start_ms;
  uint32_t last_ms;       // 上次任务时间戳, 用于 yaw 积分
  uint32_t sync_warn_ms;  // 同步偏差告警限频

  // 堵转检测
  StallDetector_s stall[2];
  bool stalled_flag[2];  // 校准/储能阶段各自的堵转记录

  // 调试点动(右开关上档)
  bool debug_jog;
  float debug_belt_speed_dps;
  float debug_trigger_speed_dps;
} DartLauncherInstance;

/**
 * @brief 初始化发射架: 注册 4 个电机并复位状态机
 */
DartLauncherInstance* DartLauncherInit(void);

/**
 * @brief 使能/失能发射架; 失能立即停机并中止进行中的序列,
 *        重新使能时若未校准则自动开始零位校准
 */
void DartLauncherSetEnable(DartLauncherInstance* inst, bool enable);

/**
 * @brief 下发一次性命令(校准/储能/发射), 非法状态下忽略
 */
void DartLauncherSetCommand(DartLauncherInstance* inst, Dart_Cmd_e cmd);

/**
 * @brief 扳机卡位(射力)微调, 自动限幅, 仅 IDLE/READY 生效
 */
void DartLauncherAdjustTrigger(DartLauncherInstance* inst, float delta_deg);

/**
 * @brief 设置 yaw 角速度指令, 由状态机积分成角度目标
 */
void DartLauncherSetYawRate(DartLauncherInstance* inst, float rate_dps);

/**
 * @brief 调试点动(仅 IDLE 态生效): 同步带双电机同速点动 + 扳机点动
 */
void DartLauncherSetDebugJog(DartLauncherInstance* inst, bool enable, float belt_speed_dps, float trigger_speed_dps);

/**
 * @brief 发射架周期任务, 由 RobotTask 以 ~1kHz 调用
 */
void DartLauncherTask(DartLauncherInstance* inst);
