//
// Created by PC on 2025/11/18.
//
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dart_shoot.h"
#include "remote_control.h"

/* 机器人状态定义 */
typedef enum {
  ROBOT_POWER_ON = 0,      // 上电运行(遥控器左侧开关上档, 已解锁)
  ROBOT_EMERGENCY_STOP,    // 急停(遥控器左侧开关非上档, 或遥控器离线)
} Robot_Mode_e;

/* 飞镖模式定义, 由遥控器右侧开关选择 */
typedef enum {
  DART_MODE_STOPPED = 0,   // 下档: 关闭, 电机不输出
  DART_MODE_MANUAL,        // 中档: 手动测试, 逐个执行器单独给定
  DART_MODE_AUTO,          // 上档: 自动测试, 状态机驱动 + 自动循环测试
} DartMode_e;

/* 自动循环测试步骤 */
typedef enum {
  TEST_STEP_IDLE = 0,      // 等待开始 / 上一周期已结束
  TEST_STEP_HOMING,        // 等待回零完成
  TEST_STEP_PULLING,       // 等待滑块拉到扳机位
  TEST_STEP_FIRING,        // 等待击发与复位完成
} DartTestStep_e;

/* 自动循环测试统计 */
typedef struct {
  uint32_t pass;                 // 通过次数
  uint32_t fail;                 // 失败次数
  uint32_t fail_pull_timeout;    // 拉滑块超时次数
  uint32_t fail_home_timeout;    // 回零超时次数
  uint32_t fail_reload_timeout;  // 复位超时次数
  uint32_t fail_offline;         // 电机离线次数
  uint32_t arrive_by_encoder;    // 靠编码器行程判定到位
  uint32_t arrive_by_stall;      // 靠堵转/电流判定到位
  float pull_time_sum_ms;        // 拉滑块耗时累加
  float pull_time_min_ms;        // 拉滑块耗时最小值
  float pull_time_max_ms;        // 拉滑块耗时最大值
  float fire_time_sum_ms;        // 击发耗时累加
  float cycle_time_sum_ms;       // 整周期耗时累加
} DartTest_Stat_s;

/* 机器人主实例结构体 */
typedef struct {
  DartShootInstance* dart_shoot;  // 发射机构组件
  RC_ctrl_t* rc_data;             // 遥控器数据
  Robot_Mode_e robot_mode;        // 机器人状态
  DartMode_e dart_mode;           // 飞镖(测试)模式
  DartTest_Stat_s test_stat;      // 自动循环测试统计
} RobotInstance;

extern RobotInstance* robot;  // 全局变量声明

/*=======核心机器人函数=======*/
/**
 * @brief 机器人系统初始化
 */
void RobotInit(void);

/**
 * @brief 机器人主任务循环
 */
void RobotTask(void);

/*=======飞镖函数=======*/
/**
 * @brief 飞镖发射机构初始化
 */
void DartInit(void);

/**
 * @brief 机器人状态与飞镖模式更新
 */
void DartStateMachineUpdate(void);

/**
 * @brief 飞镖模式切换处理
 */
void DartModeChangeHandler(void);

/**
 * @brief 关闭模式: 电机不输出
 */
void DartStoppedHandler(void);

/**
 * @brief 手动测试模式: 逐个执行器单独给定
 */
void DartManualModeHandler(void);

/**
 * @brief 自动测试模式: 状态机驱动 + 自动循环测试
 */
void DartAutoTestHandler(void);

/**
 * @brief 急停处理
 */
void DartEmergencyHandler(void);

/*=======工具函数=======*/
bool IsInDeadzone(int16_t value);
float MapStickToSpeed(int16_t stick_value, float max_speed);
