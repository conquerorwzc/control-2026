#pragma once

#include "dji_motor.h"

#define SHOOT_CNT_MAX 1

// 发射模式设置
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} Shoot_Mode_e;

typedef enum {
  FRICTION_OFF = 0,  // 摩擦轮关闭
  FRICTION_ON,       // 摩擦轮开启
} Friction_Mode_e;

typedef enum {
  LOAD_STOP = 0,   // 停止发射
  LOAD_REVERSE,    // 反转
  LOAD_1_BULLET,   // 单发
  LOAD_3_BULLET,   // 三发
  LOAD_BURSTFIRE,  // 连发
} Loader_Mode_e;

typedef enum {
  BULLET_SPEED_NONE = 0,
  BIG_AMU_10 = 10,
  SMALL_AMU_15 = 15,
  BIG_AMU_16 = 16,
  SMALL_AMU_18 = 18,
  SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  float one_bullet_delta_angle;  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
  float reduction_ratio_loader;  // 2006拨盘电机的减速比,英雄拨弹盘的3508减速比为100
  float num_per_circle;          // 拨盘一圈的装  载量
  int loader_direction;          // 拨弹盘方向
  int friction_num;              // 摩擦轮数量
  float friction_speed;          // 摩擦轮速度
  float target_speed;
  float friction_coefficients[FRICTION_NUM];  // 摩擦轮的系数
  float deadtime_onebullet;                   // 单发死时间
  float deadtime_burstfire;                   // 连发死时间
  float bullet_speed_adjustment;
} Shoot_Param_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct {
  Shoot_Mode_e shoot_mode;
  Loader_Mode_e load_mode;
  Friction_Mode_e friction_mode;
  Bullet_Speed_e bullet_speed;  // 弹速枚举
  uint8_t rest_heat;
  float shoot_rate;  // 连续发射的射频,unit per s,发/秒

} Shoot_Ctrl_Cmd_s;

typedef struct {
  Motor_Init_Config_s loader_motor_config;
  Shoot_Param_s shoot_param;
  Motor_Init_Config_s friction_motor_config[FRICTION_NUM];
} Shoot_Init_Config_s;

typedef struct {
  Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
  DJIMotorInstance* loader_motor;  // 拨盘电机
  DJIMotorInstance* friction_motor[FRICTION_NUM];
} ShootInstance;

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
ShootInstance* ShootInit(Shoot_Init_Config_s* shoot_init_config);
/**
 * @brief 云台任务
 *
 */
void ShootTask();