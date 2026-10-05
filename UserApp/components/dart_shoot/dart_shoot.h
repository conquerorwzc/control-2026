/**
 ******************************************************************************
 * @file    dart_shoot.h
 * @brief   飞镖发射机构(dart_shoot)组件: 纯遥操作驱动
 ******************************************************************************
 * @attention
 * 机构组成(全部挂在 CAN1 上, 舵机为 PWM):
 *   - 同步带电机 x2 (M3508): 通过同步带把装载飞镖的滑块拉下来
 *   - yaw 电机 x1 (M2006): 瞄准轴
 *   - 扳机位置电机 x1 (M3508): 调整扳机的机械位置
 *   - PWM 舵机 x1: 把扳机拉下来, 释放滑块
 *
 * 控制方式(组件内没有自动发射流程, 全部由上层按周期下发):
 *   1. 同步带 / yaw 用「位置环 + 位置累加」: 上层只给累加方向(±1), 速率由 param 决定;
 *      同步带没有新指令时保持最后一个目标位置(位置环顶住), yaw 没有新指令时直接失能.
 *   2. 扳机位置电机用速度环: 上层只给方向(±1), 偏出阈值时按 param.trigger_speed 转,
 *      没有指令时直接失能(不发电流, 而不是速度环给 0).
 *   3. 舵机: 转到给定角度, 或停止 PWM 脉冲输出(失能).
 *
 * 方向: 累加的正方向与实车相反时, 改 robot_config.h 中对应电机配置的 _reverse 参数
 *       (同步带两个电机要一起改), 组件不需要额外的方向参数.
 *
 * 单位: 位置用 dji_motor 的 measure.total_angle(度), 速度用 measure.speed_aps(度/秒);
 *       M3508/M2006 的编码器都在减速箱前, 因此都是转子侧角度.
 ******************************************************************************
 */
#pragma once

#include "dji_motor.h"
#include "servo_motor.h"

#define DART_SHOOT_BELT_MOTOR_NUM 2  // 同步带拉滑块电机数量(3508 x2)

/**
 * @brief 工作档位
 */
typedef enum {
  DART_SHOOT_MODE_DISABLED = 0,  // 全部执行器失能(上电默认, 安全状态)
  DART_SHOOT_MODE_AIM,           // 同步带 + yaw: 位置累加
  DART_SHOOT_MODE_TRIGGER,       // 只允许扳机位置电机: 速度环
} DartShoot_Mode_e;

/**
 * @brief 舵机指令
 */
typedef enum {
  DART_SERVO_CMD_DISABLED = 0,  // 失能: 停止 PWM 脉冲输出
  DART_SERVO_CMD_ANGLE_MID,     // 转到中档角度(param.servo_angle_mid)
  DART_SERVO_CMD_ANGLE_UP,      // 转到上档角度(param.servo_angle_up)
} DartServo_Cmd_e;

/**
 * @brief 机构参数, 与实车有关的量都要实测确定
 */
typedef struct {
  float belt_pos_rate;        // 同步带位置累加速率, 单位: 电机总角度/秒
  float yaw_pos_rate;         // yaw 位置累加速率, 单位: 电机总角度/秒
  float trigger_speed;        // 扳机位置电机的速度参考(度/秒), 摇杆偏出阈值时按它转
  float servo_angle_mid;      // 左拨杆中档时的舵机角度(°)
  float servo_angle_up;       // 左拨杆上档时的舵机角度(°)
  float servo_min_pulse_s;    // 0° 对应的舵机脉宽(s), 常见 0.0005
  float servo_max_pulse_s;    // 行程上端对应的舵机脉宽(s): 180° 舵机常见 0.0025, 270° 舵机也是 0.0025
                              // 行程上端角度 = DART_SERVO_ANGLE_RANGE(见 dart_shoot.c)
} DartShoot_Param_s;

/**
 * @brief 控制命令, 上层每个控制周期更新一次
 */
typedef struct {
  DartShoot_Mode_e mode;      // 工作档位
  int8_t belt_dir;            // 同步带累加方向: -1 / 0 / +1, 0 表示保持当前目标位置
  int8_t yaw_dir;             // yaw 累加方向: -1 / 0 / +1, 0 表示失能
  int8_t trigger_dir;         // 扳机位置电机转向: -1 / 0 / +1, 0 表示失能
  DartServo_Cmd_e servo_cmd;  // 舵机指令
} DartShoot_Ctrl_Cmd_s;

/**
 * @brief 反馈数据
 */
typedef struct {
  DartShoot_Mode_e mode;    // 当前档位
  uint8_t online;           // 4 个电机是否都在线(离线时 dji_motor 会打印总线与 ID)
  uint8_t belt_enabled;     // 同步带是否使能
  uint8_t yaw_enabled;      // yaw 是否使能
  uint8_t trigger_enabled;  // 扳机位置电机是否使能
  uint8_t servo_enabled;    // 舵机是否使能(PWM 是否在输出脉冲)
  float belt_target;        // 同步带目标位置(电机总角度)
  float belt_position;      // 同步带当前位置(两个电机平均 total_angle)
  float yaw_target;         // yaw 目标位置(电机总角度)
  float yaw_position;       // yaw 当前位置(电机总角度)
  float trigger_speed;      // 扳机位置电机速度反馈(度/秒)
  float servo_angle;        // 当前舵机角度(°)
} DartShoot_Feed_s;

/**
 * @brief 初始化配置
 */
typedef struct {
  Motor_Init_Config_s belt_motor_config[DART_SHOOT_BELT_MOTOR_NUM];  // 同步带 3508 x2
  Motor_Init_Config_s yaw_motor_config;                              // yaw 2006 x1
  Motor_Init_Config_s trigger_motor_config;                          // 扳机位置 3508 x1
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

  DJIMotorInstance *belt_motor[DART_SHOOT_BELT_MOTOR_NUM];  // 同步带电机
  DJIMotorInstance *yaw_motor;                              // yaw 电机
  DJIMotorInstance *trigger_motor;                          // 扳机位置电机
  ServoInstance *trigger_servo;                             // 拉扳机的 PWM 舵机

  /* ------------- 以下为内部状态, 上层不需要读写 ------------- */
  DartShoot_Mode_e mode;      // 上一次的工作档位, 用于检测档位切换
  DartServo_Cmd_e servo_cmd;  // 上一次的舵机指令, 只在变化时动作
  uint8_t belt_enabled;       // 同步带当前是否使能
  uint8_t yaw_enabled;        // yaw 当前是否使能
  uint8_t trigger_enabled;    // 扳机位置电机当前是否使能
  uint8_t servo_enabled;      // 舵机当前是否使能
  float belt_target;          // 同步带累加得到的目标位置
  float yaw_target;           // yaw 累加得到的目标位置
  float servo_angle;          // 当前下发的舵机角度(°)
  uint32_t dt_cnt;            // 位置累加用的时间戳计数
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
 * @brief 设置工作档位
 *
 * @param instance 实例指针
 * @param mode 目标档位
 */
void DartShootSetMode(DartShootInstance *instance, DartShoot_Mode_e mode);

/**
 * @brief 设置同步带位置累加方向(仅在 DART_SHOOT_MODE_AIM 下生效)
 *
 * @param instance 实例指针
 * @param dir -1 / 0 / +1; 0 表示不再累加, 同步带保持当前目标位置不动
 */
void DartShootSetBeltDir(DartShootInstance *instance, int8_t dir);

/**
 * @brief 设置 yaw 位置累加方向(仅在 DART_SHOOT_MODE_AIM 下生效)
 *
 * @param instance 实例指针
 * @param dir -1 / 0 / +1; 0 表示没有新指令, yaw 直接失能
 */
void DartShootSetYawDir(DartShootInstance *instance, int8_t dir);

/**
 * @brief 设置扳机位置电机转向(仅在 DART_SHOOT_MODE_TRIGGER 下生效)
 *
 * @param instance 实例指针
 * @param dir -1 / 0 / +1; 0 表示没有指令, 电机直接失能
 */
void DartShootSetTriggerDir(DartShootInstance *instance, int8_t dir);

/**
 * @brief 设置舵机指令(失能 / 中档角度 / 上档角度)
 *
 * @note  整体失能档位(DART_SHOOT_MODE_DISABLED)下, 组件会强制让舵机失能
 * @param instance 实例指针
 * @param servo_cmd 舵机指令
 */
void DartShootSetServo(DartShootInstance *instance, DartServo_Cmd_e servo_cmd);

/**
 * @brief 直接设置舵机角度(°)(调试用): 会立刻使能舵机并转到该角度
 *
 * @note  角度按 0~DART_SERVO_ANGLE_RANGE(270°, 见 dart_shoot.c) 线性映射到 servo_min_pulse_s ~ servo_max_pulse_s
 * @param instance 实例指针
 * @param angle 目标角度(°)
 */
void DartShootSetServoAngle(DartShootInstance *instance, float angle);
