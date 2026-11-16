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
 *
 * 实例字段按子组件分层: belt(同步带) / screw(丝杆) / servo(舵机) / cali / charge / fire / debug(调试档点动),
 * 顶层的 state/enabled/is_calibrated 等是整机状态机字段。
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

/* ================= 子组件 ================= */

/* 同步带(2xM3508): 共模/差模解耦 + 速度前馈; 两侧位置差 θ0−θ1 ∝ 两侧皮带张力差 */
typedef struct {
  DJIMotorInstance* motor[2]; // [0]左 [1]右
  PIDInstance common_pid;     // 共模位置环: 两侧平均位置 -> 速度给定(纯 P)
  float zero_offset[2];       // 释放方向硬限位处的 total_angle(校准得到)
  float pos_rel[2];           // 两侧相对零点的位置(deg): 0=释放方向硬限位, 校准后每周期更新
  bool zero_valid;            // 零点是否有效(有效才允许位置闭环)
  float common_ref;           // 共模目标(两侧平均位置, total_angle 坐标)
  float diff_base;            // 位置差标定基准 = 两零点之差(校准得到的"两侧应有位置差")
  float diff_ref;             // 位置差实际目标 = diff_base + tension_trim
  float tension_trim;         // 均张力慢外环修正(deg): 用两侧电流差微调位置差目标, 让出力一致
  float i_diff_filt;          // 两侧电流差低通值(raw): 均张力外环的输入
  float i_diff_fast;          // 两侧电流差快速低通(raw): 快速均流项(毫秒级)的输入
  float balance_deg;          // 快速均流项折合的等效位置差误差(deg), 日志用
  float pos_diff;             // 两侧位置差(低通后, ∝ 两侧张力差)
  float sync_i;               // 差模积分(独立记忆, 不受速度环积分限幅影响)
  float sync_u;               // 差模输出(电流偏置): 机0 前馈 −u / 机1 +u
  float add_current[2];       // 电流前馈(电机配置的 current_feedforward_ptr 指向这里)
  float ff_dps;               // 自动动作速度规划状态(加速度限幅后的前馈速度)
  float v_ff_dps;             // 本周期前馈(点动=摇杆速率, 自动动作=规划器输出)
  float v_common_dps;         // 本周期共模速度给定(前馈+P, 已限幅), 供日志用
  float hold_pos;             // 故障保持目标(逻辑坐标)
  StallDetector_s stall[2];   // 堵转检测器(速度阈值判据)
  bool stalled[2];            // 顶限位记录(两侧都置位才算找到零点)
  uint32_t sync_warn_ms;      // 同步偏差告警限频
  uint32_t sync_log_ms;       // 同步状态日志限频
} DartBelt_t;

/* 扳机丝杆(M3508): 扳机整体位置 -> 拉簧拉伸量 = 射力 */
typedef struct {
  DJIMotorInstance* motor; // M3508
  float pos_deg;           // 射力位置(相对开机位置)
} DartScrew_t;

/* 扳机舵机(PWM, 开环): 卡位角扣住平台 / 释放角放开 */
typedef struct {
  PWMInstance* pwm; // PWM 通道句柄
  float angle_deg;  // 当前目标角
} DartServo_t;

/* 校准流程(只做同步带) */
typedef struct {
  Dart_Cali_Step_e step; // 当前子步骤
  uint32_t start_ms;     // 顶限位步起始时刻(超时判据)
  uint32_t log_ms;       // 进度日志限频
} DartCali_t;

/* 储能流程 */
typedef struct {
  Dart_Charge_Step_e step; // 当前子步骤
} DartCharge_t;

/* 发射流程(舵机开环, 按时间推进) */
typedef struct {
  Dart_Fire_Step_e step; // 当前子步骤
} DartFire_t;

/* 调试档点动(右开关上档): 摇杆给速率 -> 虚拟位置目标, 同步带/丝杆各一套 */
typedef struct {
  bool enabled;            // 是否处于调试点动
  float belt_speed_dps;    // 同步带点动速率(摇杆缩放)
  float screw_speed_dps;   // 丝杆点动速率(摇杆缩放)
  float belt_target;       // 同步带虚拟位置目标(两侧共用)
  float screw_target;      // 丝杆虚拟位置目标
  float belt_diff_base;    // 进入点动时的两侧位置差基准
  bool belt_synced;        // 本次点动是否已把目标同步到实际位置
  bool screw_synced;       // 同上(丝杆)
  float belt_rate_last;    // 上周期速率(松手沿检测)
  float screw_rate_last;   // 同上(丝杆)
  uint32_t log_ms;         // 状态日志限频
} DartDebug_t;

/* ================= 发射机构实例 ================= */

typedef struct {
  DartBelt_t belt;     // 同步带
  DartScrew_t screw;   // 扳机丝杆
  DartServo_t servo;   // 扳机舵机
  DartCali_t cali;     // 校准流程
  DartCharge_t charge; // 储能流程
  DartFire_t fire;     // 发射流程
  DartDebug_t debug;   // 调试档点动

  // 整体状态机与任务
  Dart_State_e state;          // 当前状态
  Dart_Cmd_e pending_cmd;      // 待处理的一次性命令
  uint32_t step_start_ms;      // 当前序列子步骤起始时刻(序列共用)
  uint32_t last_ms;            // 上次任务时刻(用于计算 dt)
  float task_dt_s;             // 实测任务周期(所有滤波器用它)
  bool enabled;                // 遥控器使能(失能即全部停机)
  bool is_calibrated;          // 本次上电同步带校准已完成(校准前力矩被严格限制)
  bool recovery_retract;       // 储能被中止后, 重新使能须先回撤挡块
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

/** @brief 设置调试档点动速率(IDLE/READY 生效): 同步带按摇杆速率点动 + 丝杆点动(READY 只动同步带) */
void ShootDartSetDebug(ShootDartInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps);

/** @brief 周期任务, 由 RobotTask 以 ~1kHz 调用 */
void ShootDartTask(ShootDartInstance* inst);
