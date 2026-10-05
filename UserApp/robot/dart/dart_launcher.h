/**
 * @file dart_launcher.h
 * @brief dart 发射架执行层: 5 个执行器(1xM2006 yaw + 2xM3508 同步带 + 1xM3508 扳机丝杆 + 1xPWM 舵机)的状态机与闭环控制
 *
 * 扳机子系统含两个执行器:
 *   - 扳机丝杆(M3508, 位置环串级): 扳机整体在丝杆上的位置, 决定卡住发射平台时拉簧的拉伸量(射力, 可调);
 *   - 扳机舵机(PWM, 50Hz 占空比->角度, 开环): 扳机的发射动作, 卡位角扣住发射平台 / 释放角放开发射。
 *
 * 分步操作流程(由上层命令驱动):
 *   1. 上电使能后自动执行一次校准: 同步带顶释放方向硬限位(堵转检测, 同步两带电机零点)
 *      -> 扳机丝杆顶硬限位(丝杆零点), 完成后退回默认射力位置; 校准全程严格限制电机力矩;
 *   2. 储能命令: 扳机丝杆先到射力位置 -> 双带电机同步把平台向后拉指定行程
 *      (扳机锁定态是单向通道, 平台会滑过扳机, 不会堵转) -> 限速复位到释放位置;
 *   3. 就绪(READY)后可随时调射力(丝杆位置);
 *   4. 发射命令: 舵机转到释放角放开发射平台, 随后自动回卡位角待下一发。
 */
#pragma once

#include "bsp_pwm.h"
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

/* 校准子步骤(同步带 -> 丝杆, 顺序执行) */
typedef enum {
  CALI_STEP_BELT_DRIVE_TO_STOP = 0,  // 同步带顶释放方向硬限位(低力矩)
  CALI_STEP_BELT_BACKOFF,            // 同步带回撤到释放位置
  CALI_STEP_SCREW_DRIVE_TO_STOP,     // 扳机丝杆顶硬限位(低力矩)
  CALI_STEP_SCREW_BACKOFF,           // 丝杆退开限位到默认射力位置
} Dart_Cali_Step_e;

/* 储能子步骤 */
typedef enum {
  CHARGE_STEP_PREP_SCREW = 0,  // 扳机丝杆先到射力位置(拉伸量决定射力)
  CHARGE_STEP_DRIVE,           // 双电机同步把发射平台向后拉指定行程(单向扳机不阻挡, 不堵转)
  CHARGE_STEP_RETRACT,         // 限速复位到释放位置(挡块退出发射平台活动范围)
} Dart_Charge_Step_e;

/* 发射子步骤(舵机开环, 全部按时间推进) */
typedef enum {
  FIRE_STEP_RELEASE = 0,  // 舵机转到释放角, 放开发射平台
  FIRE_STEP_DWELL,        // 释放角保持
  FIRE_STEP_RESET,        // 舵机回卡位角待下一发
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
  DJIMotorInstance* screw_motor;    // M3508 扳机丝杆: 扳机整体位置(拉簧拉伸量=射力)
  PWMInstance* servo_pwm;           // PWM 舵机: 扳机发射动作(卡位角/释放角)

  Dart_State_e state;
  bool enabled;           // 遥控器使能标志(失能即全部停机)
  bool is_calibrated;     // 本次上电校准(同步带+丝杆)是否已完成
  bool belt_zero_valid;   // 同步带零点是否有效(顶到释放方向硬限位后置位)
  bool screw_zero_valid;  // 丝杆零点是否有效(顶到硬限位后置位)
  bool recovery_retract;  // 储能中断后, 重新使能须先回撤挡块

  // 逻辑坐标零点
  float belt_zero_offset[2];  // 同步带: 释放方向硬限位处的 total_angle
  float screw_zero_offset;    // 丝杆: 硬限位处的 total_angle
  float yaw_boot_angle;       // yaw 开机角度

  // 控制目标(逻辑坐标)
  float belt_pos_target;    // 同步带目标 (deg, 正=储能方向)
  float yaw_angle_target;   // yaw 目标角 (deg, 相对开机)
  float screw_pos_deg;      // 扳机丝杆位置(射力, 可调, deg 相对丝杆零点)
  float servo_angle_deg;    // 舵机当前目标角 (deg)
  float yaw_rate_cmd_dps;   // yaw 角速度指令 (deg/s)

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
  StallDetector_s stall[2];   // 两个同步带电机
  StallDetector_s screw_stall;  // 扳机丝杆
  bool stalled_flag[2];       // 同步带校准阶段各电机堵转记录

  // 调试点动(右开关上档)
  bool debug_jog;
  float debug_belt_speed_dps;
  float debug_screw_speed_dps;
} DartLauncherInstance;

/**
 * @brief 初始化发射架: 注册电机/PWM 舵机并复位状态机
 */
DartLauncherInstance* DartLauncherInit(void);

/**
 * @brief 使能/失能发射架; 失能立即停机并中止进行中的序列;
 *        重新使能时会重同步 yaw 目标并清 PID(防失能期间被手推动后跳变);
 *        auto_cali = true 且未校准时自动开始零位校准, 调试档传 false(先点动验方向, 手动校准)
 */
void DartLauncherSetEnable(DartLauncherInstance* inst, bool enable, bool auto_cali);

/**
 * @brief 下发一次性命令(校准/储能/发射), 非法状态下忽略
 */
void DartLauncherSetCommand(DartLauncherInstance* inst, Dart_Cmd_e cmd);

/**
 * @brief 扳机丝杆位置(射力)微调, 自动限幅, 仅 IDLE/READY 生效
 */
void DartLauncherAdjustScrewPos(DartLauncherInstance* inst, float delta_deg);

/**
 * @brief 直接设定舵机角度(自动限幅), 用于标定卡位角/释放角
 */
void DartLauncherSetServoAngle(DartLauncherInstance* inst, float angle_deg);

/**
 * @brief 舵机角度增量(自动限幅), 调试档用: 松手即停, 防止绝对映射松手跳变误触发
 */
void DartLauncherAdjustServoAngle(DartLauncherInstance* inst, float delta_deg);

/**
 * @brief 设置 yaw 角速度指令, 由状态机积分成角度目标
 */
void DartLauncherSetYawRate(DartLauncherInstance* inst, float rate_dps);

/**
 * @brief 调试点动(仅 IDLE 态生效): 同步带双电机同速点动 + 扳机丝杆点动
 */
void DartLauncherSetDebugJog(DartLauncherInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps);

/**
 * @brief 发射架周期任务, 由 RobotTask 以 ~1kHz 调用
 */
void DartLauncherTask(DartLauncherInstance* inst);
