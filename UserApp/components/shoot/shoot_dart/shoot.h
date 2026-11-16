/**
 * @file shoot.h
 * @brief dart 发射机构(shoot_dart): 2xM3508 同步带 + 1xM3508 扳机丝杆 + 1xPWM 舵机
 *
 * 执行器(发射架 yaw 在 components/gimbal/gimbal_dart, 本模块不涉及):
 *   - 同步带(M3508 x2): 对置驱动挡块, 把发射平台向后拉(储能)/复位(释放), 两侧严格同步;
 *   - 扳机丝杆(M3508): 扳机整体位置, 决定卡住平台时拉簧的拉伸量(射力, 可调);
 *   - 扳机舵机(PWM 50Hz, 开环): 卡位角扣住发射平台 / 释放角放开发射。
 *
 * 流程: 上电使能 -> 自动校准(同步带顶释放方向硬限位记零点 -> 回撤到释放位)
 *       -> 储能(丝杆到射力位 -> 双带拉平台 -> 回撤到释放位) -> 发射(舵机释放 -> 回卡位)。
 *
 * 蜂鸣器提示由本模块直接调用 dart_buzzer; 遥控失联/调试档/使能沿等机器人级提示由 robot.c 调用。
 */
#pragma once

#include "bsp_pwm.h"
#include "dji_motor.h"
#include <stdbool.h>
#include <stdint.h>

/* 发射机构状态 */
typedef enum {
  DART_STATE_IDLE = 0,    // 待机: 挡块在释放位, 扳机在卡位, 未储能(或刚发射完)
  DART_STATE_CALIBRATING, // 释放方向零位校准中
  DART_STATE_CHARGING,    // 储能序列中
  DART_STATE_READY,       // 储能完成, 等待发射
  DART_STATE_FIRING,      // 发射序列中
  DART_STATE_FAULT,       // 故障(超时/异常), 保持位置等重新校准
} Dart_State_e;

/* 上层命令 */
typedef enum {
  DART_CMD_NONE = 0,
  DART_CMD_CALIBRATE, // 重新执行零位校准(故障恢复/手动重校)
  DART_CMD_CHARGE,    // 储能
  DART_CMD_FIRE,      // 发射
} Dart_Cmd_e;

/* 校准子步骤(只做同步带: 顶限位记零点 -> 回撤到释放位) */
typedef enum {
  CALI_STEP_BELT_DRIVE_TO_STOP = 0,
  CALI_STEP_BELT_BACKOFF,
} Dart_Cali_Step_e;

/* 储能子步骤 */
typedef enum {
  CHARGE_STEP_PREP_SCREW = 0, // 丝杆先到射力位置(拉伸量决定射力)
  CHARGE_STEP_DRIVE,          // 双带把平台向后拉指定行程(扳机单向通道, 不堵转)
  CHARGE_STEP_RETRACT,        // 回撤到释放位(挡块脱离平台活动范围)
} Dart_Charge_Step_e;

/* 发射子步骤(舵机开环, 按时间推进) */
typedef enum {
  FIRE_STEP_RELEASE = 0, // 舵机到释放角
  FIRE_STEP_DWELL,       // 释放角保持
  FIRE_STEP_RESET,       // 舵机回卡位角
} Dart_Fire_Step_e;

/* 堵转检测器: 实测角速度持续低于阈值即判定堵转(调用侧负责起步宽限)
 * @note 不用 PID 的 PI_ErrorHandle 堵转标志位: 它置位后需手动清, 会把已过去的堵转带进后续阶段 */
typedef struct {
  bool tracking;
  uint32_t start_ms;
} StallDetector_s;

/* 发射机构实例 */
typedef struct {
  // 执行器
  DJIMotorInstance* belt_motor[2]; // 同步带 [0]左 [1]右
  DJIMotorInstance* screw_motor;   // 扳机丝杆
  PWMInstance* servo_pwm;          // 扳机舵机

  // 状态
  Dart_State_e state;
  bool enabled;          // 遥控器使能(失能即全部停机)
  bool is_calibrated;    // 本次上电同步带校准已完成(校准前力矩被严格限制)
  bool belt_zero_valid;  // 同步带零点有效(顶到释放方向硬限位后置位)
  bool recovery_retract; // 储能被中止后, 重新使能须先回撤挡块

  // 逻辑坐标
  float belt_zero_offset[2]; // 同步带: 释放方向硬限位处的 total_angle
  float belt_common_ref;     // 共模目标(两侧平均位置, total_angle 坐标)
  float belt_diff_base;      // 差模基准 = 两零点之差, 两侧"应有"的位置差(∝张力差基准)
  float belt_dev_f;          // 两侧位置差低通值(抑制两机 CAN 帧不同时到达的假误差)
  float fault_hold_pos;      // 故障保持目标(逻辑坐标)
  float screw_pos_deg;       // 丝杆位置(射力, 相对开机位置)
  float servo_angle_deg;     // 舵机当前目标角

  // 同步带控制状态
  PIDInstance belt_common_pid; // 共模位置环: 两侧平均位置 -> 速度给定
  float belt_sync_i;           // 差模积分(独立记忆, 不受速度环积分限幅影响)
  float belt_sync_u;           // 差模输出(电流偏置): 机0 前馈 −u / 机1 +u
  float belt_add_current[2];   // 电流前馈(电机配置的 current_feedforward_ptr 指向这里)
  float belt_ff_dps;           // 自动动作速度规划状态(梯形: 加速度限幅后的前馈速度)
  float belt_v_ff_dps;         // 本周期前馈(点动=摇杆速率, 自动动作=规划器输出)
  float belt_v_common_dps;     // 本周期共模速度给定(前馈+P, 已限幅), 供日志用

  // 序列子步骤与计时
  Dart_Cali_Step_e cali_step;
  Dart_Charge_Step_e charge_step;
  Dart_Fire_Step_e fire_step;
  Dart_Cmd_e pending_cmd;
  uint32_t step_start_ms;
  uint32_t cali_start_ms;
  uint32_t cali_log_ms;
  uint32_t last_ms;
  uint32_t sync_warn_ms;
  uint32_t sync_log_ms;
  float task_dt_s; // 实测任务周期(所有滤波器用它)

  // 堵转判据(速度阈值)
  StallDetector_s stall[2];
  bool stalled_flag[2];

  // 调试点动(右开关上档)
  bool debug_jog;
  float debug_belt_speed_dps;
  float debug_screw_speed_dps;
  float debug_belt_target;
  bool debug_belt_synced;
  float debug_belt_dev_base;
  float debug_screw_target;
  bool debug_screw_synced;
  float debug_belt_rate_last;
  float debug_screw_rate_last;
  uint32_t debug_log_ms;
} ShootDartInstance;

/** @brief 初始化: 注册同步带/丝杆电机与 PWM 舵机, 复位状态机 */
ShootDartInstance* ShootDartInit(void);

/** @brief 使能/失能; 失能立即停机并中止序列; auto_cali 且未校准时自动开始零位校准 */
void ShootDartSetEnable(ShootDartInstance* inst, bool enable, bool auto_cali);

/** @brief 下发一次性命令(校准/储能/发射), 非法状态忽略 */
void ShootDartSetCommand(ShootDartInstance* inst, Dart_Cmd_e cmd);

/** @brief 射力(丝杆位置)微调, 自动限幅, 仅 IDLE 生效 */
void ShootDartAdjustScrewPos(ShootDartInstance* inst, float delta_deg);

/** @brief 舵机角度增量(调试档用, 松手即停), 自动限幅 */
void ShootDartAdjustServoAngle(ShootDartInstance* inst, float delta_deg);

/** @brief 调试点动(IDLE/READY 生效): 同步带按摇杆速率点动 + 丝杆点动(READY 只动同步带) */
void ShootDartSetDebugJog(ShootDartInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps);

/** @brief 周期任务, 由 RobotTask 以 ~1kHz 调用 */
void ShootDartTask(ShootDartInstance* inst);
