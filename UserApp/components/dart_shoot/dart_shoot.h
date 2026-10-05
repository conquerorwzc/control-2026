/**
 ******************************************************************************
 * @file    dart_shoot.h
 * @brief   飞镖发射机构(dart_shoot)组件
 ******************************************************************************
 * @attention
 * 机构组成:
 *   - 同步带电机 x2 (M3508): 通过同步带把装载飞镖的滑块拉下来, 拉到扳机的位置
 *   - 扳机调整电机 x1 (M3508): 调整扳机的机械位置
 *   - PWM 舵机 x1: 把扳机拉下来, 释放滑块, 滑块飞出并发射飞镖
 *
 * 三个 3508 均使用速度环控制:
 *   速度环参考值与反馈值单位均为 度/秒(deg/s), 即 dji_motor 中的 measure.speed_aps
 *
 * 完整发射流程(自动模式):
 *   HOME(回零) -> PULL(拉滑块到扳机位) -> READY(就位等待) -> FIRE/RELEASING(舵机拉扳机, 滑块飞出)
 *   -> RELOADING(舵机回位 + 同步带反向收回滑块) -> IDLE
 ******************************************************************************
 */
#pragma once

#include "dji_motor.h"
#include "servo_motor.h"

#define DART_SHOOT_PULL_MOTOR_NUM 2  // 同步带拉滑块电机数量(3508 x2)

/**
 * @brief 工作模式
 */
typedef enum {
  DART_SHOOT_MODE_STOPPED = 0,  // 停机: 电机停止输出, 舵机回到锁止位(上电默认, 安全状态)
  DART_SHOOT_MODE_MANUAL,       // 手动: 直接使用 ctrl_cmd 中的速度/舵机角度, 用于调试与测试
  DART_SHOOT_MODE_AUTO,         // 自动: 状态机完成 回零-拉滑块-击发-复位 流程
} DartShoot_Mode_e;

/**
 * @brief 状态机状态
 */
typedef enum {
  DART_SHOOT_STATE_STOPPED = 0,  // 停机
  DART_SHOOT_STATE_MANUAL,       // 手动
  DART_SHOOT_STATE_IDLE,         // 待机(滑块可自由移动)
  DART_SHOOT_STATE_HOMING,       // 回零(反向低速找机械零点)
  DART_SHOOT_STATE_PULLING,      // 拉滑块到扳机位
  DART_SHOOT_STATE_READY,        // 已到扳机位, 等待击发
  DART_SHOOT_STATE_RELEASING,    // 舵机拉下扳机, 滑块飞出
  DART_SHOOT_STATE_RELOADING,    // 复位(同步带收回滑块 + 舵机回锁止位)
  DART_SHOOT_STATE_ERROR,        // 异常(需要 CMD_RESET 清除)
} DartShoot_State_e;

/**
 * @brief 一次性指令, 由上层在需要时下发一次, 组件执行后会自动清为 DART_SHOOT_CMD_NONE
 * @note  请勿在每个控制周期都下发同一条指令, 否则会被重复执行
 */
typedef enum {
  DART_SHOOT_CMD_NONE = 0,  // 无指令
  DART_SHOOT_CMD_HOME,      // 回零: 反向低速直到堵转, 记录零点
  DART_SHOOT_CMD_PULL,      // 拉滑块到扳机位
  DART_SHOOT_CMD_FIRE,      // 击发: 舵机拉下扳机释放滑块
  DART_SHOOT_CMD_RELOAD,    // 复位: 舵机回锁止位并把滑块收回零点
  DART_SHOOT_CMD_ABORT,     // 中止当前动作, 回到待机
  DART_SHOOT_CMD_RESET,     // 清除异常, 回到待机
} DartShoot_Cmd_e;

/**
 * @brief 异常类型
 */
typedef enum {
  DART_SHOOT_ERROR_NONE = 0,     // 无异常
  DART_SHOOT_ERROR_PULL_TIMEOUT,   // 拉滑块超时
  DART_SHOOT_ERROR_HOME_TIMEOUT,   // 回零超时
  DART_SHOOT_ERROR_RELOAD_TIMEOUT, // 复位超时
  DART_SHOOT_ERROR_MOTOR_OFFLINE,  // 3508 离线
} DartShoot_Error_e;

/**
 * @brief 机构参数, 需要根据实车实测填写
 */
typedef struct {
  /* ---------------- 运动方向与速度(速度环参考值, 单位: 度/秒) ---------------- */
  int8_t pull_direction;          // 拉滑块时同步带电机的转向: 1 或 -1; 回零与复位方向取其反方向
  float pull_speed;               // 自动模式下拉滑块速度
  float reload_speed;             // 自动模式下复位(收回滑块)速度
  float home_speed;               // 回零速度(建议低速, 靠电流阈值判定机械零点)
  float pull_speed_limit;         // 手动模式下同步带电机的速度限幅
  float trigger_speed_limit;      // 扳机调整电机的速度限幅(自动/手动均生效)
  /* ---------------- 行程与到位判定 ---------------- */
  float pull_travel;              // 从起点到扳机位的行程, 单位: 电机总角度(measure.total_angle)
                                  // 换算: 电机总角度 = 滑块行程(mm) / 同步带轮周长(mm) * 360 * 减速比
  float pull_position_tolerance;  // 到位位置容差, 与 pull_travel 同单位
  float pull_current_threshold;   // 到位/堵转判定电流阈值(3508 反馈原始值, 约 ±16384 对应 ±20A)
  float pull_stall_time_ms;       // 电流(或 PID 堵转)需要连续保持的时间
  float pull_timeout_ms;          // 拉滑块超时
  float home_timeout_ms;          // 回零超时
  float reload_timeout_ms;        // 复位超时
  /* ---------------- PWM 舵机(拉扳机) ---------------- */
  float servo_lock_angle;         // 锁止位角度(°): 扳机未被拉下, 滑块被扳机挡住
  float servo_release_angle;      // 释放位角度(°): 扳机被拉下, 滑块飞出
  float servo_angle_min;          // 舵机机械最小角度(°), 用于角度->脉宽线性映射
  float servo_angle_max;          // 舵机机械最大角度(°)
  float servo_min_pulse_s;        // 最小角度对应脉宽(s), 常见 0.0005
  float servo_max_pulse_s;        // 最大角度对应脉宽(s), 常见 0.0025
  float servo_period_s;           // 舵机 PWM 周期(s), 模拟舵机通常为 0.02(50Hz)
  float servo_release_time_ms;    // 击发时舵机保持在释放位的时间(扳机拉下->滑块飞出的时间)
} DartShoot_Param_s;

/**
 * @brief 控制命令, 上层每个周期更新(手动模式直接生效)
 */
typedef struct {
  DartShoot_Mode_e mode;   // 工作模式
  DartShoot_Cmd_e cmd;     // 一次性指令, 组件执行后自动清为 DART_SHOOT_CMD_NONE
  float pull_speed;        // 手动模式下同步带电机速度参考(度/秒, 正负即方向, 会被 pull_speed_limit 限幅)
  float trigger_speed;     // 扳机调整电机速度参考(度/秒, 自动/手动模式均生效, 默认给 0)
  float servo_angle;       // 手动模式下舵机角度(°), 会被限制在 servo_angle_min~servo_angle_max
} DartShoot_Ctrl_Cmd_s;

/**
 * @brief 反馈数据, 供上层状态显示与测试使用
 */
typedef struct {
  DartShoot_State_e state;   // 当前状态
  DartShoot_Error_e error;   // 当前异常
  uint8_t is_homed;          // 是否已完成回零
  uint8_t is_ready;          // 是否已在扳机位(可以击发)
  uint8_t is_online;         // 三个 3508 是否都在线
  uint32_t fire_count;       // 累计击发次数
  float pull_position;       // 相对回零点的滑块位置(电机总角度单位)
  float pull_traveled;       // 本次拉滑块已经走过的行程
  float pull_speed;          // 同步带电机的平均速度(度/秒)
  float pull_time_ms;        // 上一次拉滑块耗时
  float fire_time_ms;        // 上一次击发(舵机释放)耗时
  float servo_angle;         // 当前舵机角度(°)
} DartShoot_Feed_s;

/**
 * @brief 初始化配置
 */
typedef struct {
  Motor_Init_Config_s pull_motor_config[DART_SHOOT_PULL_MOTOR_NUM];  // 同步带 3508 x2
  Motor_Init_Config_s trigger_motor_config;                          // 扳机调整 3508 x1
  Servo_Init_Config_s trigger_servo_config;                          // PWM 舵机 x1
  DartShoot_Param_s param;                                           // 机构参数
} DartShoot_Init_Config_s;

/**
 * @brief dart_shoot 实例
 */
typedef struct {
  DartShoot_Ctrl_Cmd_s ctrl_cmd;  // 上层写入的控制命令
  DartShoot_Feed_s feed;          // 反馈数据
  DartShoot_Param_s param;        // 机构参数(初始化时从 config 拷贝)

  DJIMotorInstance *pull_motor[DART_SHOOT_PULL_MOTOR_NUM];  // 同步带电机
  DJIMotorInstance *trigger_motor;                          // 扳机调整电机
  ServoInstance *trigger_servo;                             // 拉扳机的 PWM 舵机

  /* ------------- 以下为内部状态, 上层不需要读写 ------------- */
  DartShoot_Mode_e mode;        // 上一次的工作模式, 用于检测模式切换
  DartShoot_State_e state;      // 状态机当前状态
  DartShoot_Error_e error;      // 当前异常
  uint8_t is_homed;             // 是否已回零
  uint8_t online_latched;       // 是否曾经检测到全部电机在线(避免上电瞬间误报离线)
  uint8_t stall_timing;         // 电流/堵转计时是否已开始
  uint32_t fire_count;          // 累计击发次数
  float pull_zero_angle;        // 回零得到的零点(电机总角度)
  float pull_start_angle;       // 本次拉滑块起点(电机总角度)
  float servo_angle_cmd;        // 当前下发的舵机角度(°)
  float state_start_time;       // 进入当前状态的时间(DWT, ms)
  float stall_start_time;       // 电流/堵转开始的时间(DWT, ms)
  float pull_time_ms;           // 上一次拉滑块耗时
  float fire_time_ms;           // 上一次击发耗时
} DartShootInstance;

/**
 * @brief 初始化 dart_shoot, 由 RobotInit() 调用
 *
 * @param init_config 初始化配置
 * @return DartShootInstance* 实例指针
 */
DartShootInstance *DartShootInit(DartShoot_Init_Config_s *init_config);

/**
 * @brief dart_shoot 任务, 需要在 RobotTask() 中以 1kHz 调用
 *
 * @param instance 实例指针
 */
void DartShootTask(DartShootInstance *instance);

/**
 * @brief 设置工作模式
 *
 * @param instance 实例指针
 * @param mode 目标模式
 */
void DartShootSetMode(DartShootInstance *instance, DartShoot_Mode_e mode);

/**
 * @brief 下发一次性指令(请在需要的时刻只调用一次)
 *
 * @param instance 实例指针
 * @param cmd 指令
 */
void DartShootSendCmd(DartShootInstance *instance, DartShoot_Cmd_e cmd);

/**
 * @brief 手动模式下手动设置输出(调试/测试用)
 *
 * @param instance 实例指针
 * @param pull_speed 同步带电机速度(度/秒)
 * @param trigger_speed 扳机调整电机速度(度/秒)
 * @param servo_angle 舵机角度(°)
 */
void DartShootSetManualOutput(DartShootInstance *instance, float pull_speed, float trigger_speed, float servo_angle);

/**
 * @brief 直接设置舵机角度(°), 会被限制在 servo_angle_min~servo_angle_max 内
 *
 * @param instance 实例指针
 * @param angle 目标角度(°)
 */
void DartShootSetServoAngle(DartShootInstance *instance, float angle);
