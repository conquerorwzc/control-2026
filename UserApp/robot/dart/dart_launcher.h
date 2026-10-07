/**
 * @file dart_launcher.h
 * @brief dart 发射架执行层: 5 个执行器(1xM2006 yaw + 2xM3508 同步带 + 1xM3508 扳机丝杆 + 1xPWM 舵机)的状态机与闭环控制
 *
 * 扳机子系统含两个执行器:
 *   - 扳机丝杆(M3508, 位置环串级): 扳机整体在丝杆上的位置, 决定卡住发射平台时拉簧的拉伸量(射力, 可调);
 *   - 扳机舵机(PWM, 50Hz 占空比->角度, 开环): 扳机的发射动作, 卡位角扣住发射平台 / 释放角放开发射。
 *
 * 分步操作流程(由上层命令驱动):
 *   1. 上电使能后自动执行一次校准: **只做同步带**——顶释放方向硬限位(堵转检测, 同步两侧零点)
 *      -> 回撤到释放位置; 全程严格限制电机力矩。
 *      扳机丝杆**不自动校零**(校准代码已注释停用), 它以"开机位置"为位置零点;
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

/* 校准子步骤(当前只走同步带两步; 丝杆两步已停用, 枚举保留便于恢复) */
typedef enum {
  CALI_STEP_BELT_DRIVE_TO_STOP = 0,  // 同步带顶释放方向硬限位(低力矩)
  CALI_STEP_BELT_BACKOFF,            // 同步带回撤到释放位置 -> 校准结束
  CALI_STEP_SCREW_DRIVE_TO_STOP,     // [停用] 扳机丝杆顶硬限位(低力矩)
  CALI_STEP_SCREW_BACKOFF,           // [停用] 丝杆退开限位到默认射力位置
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

/* 速度阈值堵转检测器: 速度给定有效时, 实测角速度持续低于阈值即判定堵转
 * @note 不再使用 PID 的堵转标志位(PID_ErrorHandle): 它置位后不手动清就一直挂着,
 *       容易把"已经过去的堵转"带到后续阶段造成误故障 */
typedef struct {
  bool tracking;      // 是否正在累计"低速"时长
  uint32_t start_ms;  // 开始累计的时刻
} StallDetector_s;

/* 发射架实例 */
typedef struct {
  DJIMotorInstance* yaw_motor;      // M2006 发射架 yaw
  DJIMotorInstance* belt_motor[2];  // M3508 同步带电机 [0]左 [1]右, 同一位置目标严格同步
  DJIMotorInstance* screw_motor;    // M3508 扳机丝杆: 扳机整体位置(拉簧拉伸量=射力)
  PWMInstance* servo_pwm;           // PWM 舵机: 扳机发射动作(卡位角/释放角)

  Dart_State_e state;
  bool enabled;           // 遥控器使能标志(失能即全部停机)
  bool is_calibrated;     // 本次上电校准是否已完成(当前=同步带校准完成)
  bool belt_zero_valid;   // 同步带零点是否有效(顶到释放方向硬限位后置位)
  bool screw_zero_valid;  // 丝杆位置参考是否可用(不自动校零时上电即置位, 零点=开机位置)
  bool recovery_retract;  // 储能中断后, 重新使能须先回撤挡块

  // 逻辑坐标零点
  float belt_zero_offset[2];  // 同步带: 释放方向硬限位处的 total_angle
  float screw_zero_offset;    // 丝杆: 零点处的 total_angle(不自动校零时为 0, 即开机位置)
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
  uint32_t cali_log_ms;   // 校准进度日志限频

  // 堵转判据: 速度阈值(DART_CALI_STALL_SPEED_DPS/MS), 不用 PID 堵转标志位
  StallDetector_s stall[2];      // 两个同步带电机
  StallDetector_s screw_stall;   // 扳机丝杆(丝杆校准已 #if 0 停用, 保留供恢复)
  bool stalled_flag[2];          // 同步带两侧顶限位记录(两侧都置位才算找到零点)

  // 双带共模/差模解耦(均载 + 自动纠偏): 共模让两侧平均位置跟目标, 两电机得到**完全相同**的速度给定
  // (P 项一致, 稳态出力均担); 差模把两侧位置差收敛到基准, 输出为**力矩偏置**(经积分互融注入, 有上限)
  PIDInstance belt_common_pid;  // 共模位置环: measure=两侧平均位置, ref=belt_common_ref -> 速度给定
  PIDInstance belt_diff_pid;    // 差模纠偏环: measure=两侧位置差, ref=belt_diff_ref -> 力矩偏置
  float belt_common_ref;        // 共模目标 (转子侧 total_angle 坐标)
  float fault_hold_pos;         // 故障保持目标(逻辑坐标): 故障不断力矩, 保持当前位置防拉簧带飞
  float belt_diff_base;         // 差模基准(硬): 校准后=两零点之差, 点动=进入点动时的位置差
  float belt_diff_ref;          // 差模基准(当前): 位置同步模式=硬基准; 力矩同步模式=以 TAU 跟随实际偏差
  bool belt_torque_sync;        // 当前模式: true=力矩同步(带载, 软基准均流), false=位置同步(空载, 硬基准锁偏差)
  float belt_load_filt;         // 负载判据滤波值(两台 |real_current| 之和的低通)
  uint32_t belt_mode_switch_ms; // 上次模式切换时刻(最短驻留防翻转)
  float belt_diff_torque;       // 差模力矩偏置(±, DART_BELT_SYNC_TORQUE_MAX 钳幅), 由 BlendBeltIntegrals 注入

  // 调试点动(右开关上档)
  bool debug_jog;
  float debug_belt_speed_dps;   // 同步带点动速率(deg/s), 用于积分出两侧共用的位置目标
  float debug_screw_speed_dps;  // 丝杆点动速率(deg/s), 用于积分出虚拟位置目标
  float debug_belt_target;      // 同步带虚拟位置目标(转子侧 total_angle 坐标), 两侧共用; 反向指令时重同步到实际位置
  bool debug_belt_synced;       // 进入点动时是否已把目标同步到两侧当前位置
  float debug_belt_dev_base;    // 进入点动时的两侧位置差基准(用于纠偏与偏差告警)
  float debug_screw_target;     // 丝杆虚拟位置目标(转子侧 total_angle 坐标); 与同步带同机制, 顶限位暂停积分
  bool debug_screw_synced;      // 进入点动时是否已把丝杆目标同步到当前位置
  float debug_belt_rate_last;   // 上周期点动速率(松手沿检测: 松手瞬间收回目标, 防"松手后飘一段")
  float debug_screw_rate_last;
  uint32_t debug_log_ms;        // 调试档状态日志限频
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
 * @brief 扳机丝杆位置(射力)微调, 自动限幅, **仅 IDLE 生效**
 *        (READY 时平台被扳机锁住、拉簧带载, 动丝杆=带载推棘爪, 有误释放/卡滞风险)
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
 * @brief 调试点动(IDLE/READY 生效; READY 只动同步带——平台被扳机锁住, 丝杆锁定):
 *        同步带两侧同步点动 + 扳机丝杆点动
 * @param belt_speed_dps 同步带点动速率(deg/s): 两侧共用同一虚拟位置目标按此速率推进,
 *                       位置环保证两侧不累计偏差; 0 = 目标保持, 位置环继续收敛已有偏差
 * @param screw_speed_dps 丝杆点动速率(deg/s): 与同步带同机制积分出虚拟位置目标,
 *                       0 = 目标保持(位置环顶住不滑走), 顶限位时暂停积分不持续出力
 */
void DartLauncherSetDebugJog(DartLauncherInstance* inst, bool enable, float belt_speed_dps, float screw_speed_dps);

/**
 * @brief 发射架周期任务, 由 RobotTask 以 ~1kHz 调用
 */
void DartLauncherTask(DartLauncherInstance* inst);
