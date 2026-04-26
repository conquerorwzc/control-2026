//
// Created by zeg on 2025/12/3.
//

/**
******************************************************************************
* @file    chassis.h
* @author  NeoZeng
* @author  Annotation and Modification By SRM-Control 2026
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2026 all rights reserved
* @brief   Mecanum Chassis Module
******************************************************************************
* @attention
* Mecanum Chassis Motor Layout:
*
*          motor[0]     motor[1]
*        Left Front   Right Front
*             ╭─────────────╮
*             │      ↑      │
*             │    Front    │
*             ╰─────────────╯
*          motor[2]     motor[3]
*        Left Rear    Right Rear
*
* @note    Motor Index:
*          motor[0] - Left Front Wheel
*          motor[1] - Right Front Wheel
*          motor[2] - Left Rear Wheel
*          motor[3] - Right Rear Wheel
*          腿：0左 1右
*          (车尾方向)                                                (车头方向)

 (主电机轴心)                                          (机架固定支点)
      A ==================================================== D
       \               [ L1 机架 / 固定轴距 ]               /
        \                                                  /
  [ L2   \                                                /  [ L4
  主动杆] \                                              /   从动杆 ]
           \                                            /
            \                                          /
             \                                        /
(后轮轴心) W ======= B ---------------------------------- C
   |    [ L_ext 延长段 ]           [ L3 连杆 ]
   |
  (O) 后轮
******************************************************************************
*/
#pragma once

#include "dji_motor.h"
#include "dmmotor.h"
#include "external_imu/external_imu.h"
#include "super_cap.h"
#include "tofsense.h"
typedef enum {
  CHASSIS_POWER_OFF = 0,     // 电流零输入
  CHASSIS_ROTATE,            // 小陀螺模式
  CHASSIS_FOLLOW,            // 跟随模式，底盘叠加角度环控制
  CHASSIS_FOLLOW_REAR_END,   // 掉头模式,跟随车辆尾部
} Chassis_Mode_e;
typedef enum {
  LEG_DISABLE = 0,        // 腿部电机失能
  LEG_NORMAL,             // 常规位置
  LEG_RAISE,              // 抬起位置
  LEG_KIKE,
  LEG_MANUAL_UP,          // 手动上升
  LEG_MANUAL_DOWN,        // 手动下降
  LEG_HOLD,
  LEG_CRUISE,
  LEG_IN_AIR,
} Leg_Mode_e;
typedef struct {
  // 控制部分
  float vx;            // 前进方向速度
  float vy;            // 横移方向速度
  float wz;            // 旋转速度
  Chassis_Mode_e chassis_mode;
  Leg_Mode_e leg_mode;
  float offset_angle;  // 底盘和归中位置的夹角
  int chassis_speed_buff;
  uint16_t max_power;  // 最大功率限制
  float power_distribute;  //前后轮功率分配系数
  float chassis_direction; //底盘运动朝向
  // UI部分
  //  ...
  uint8_t SuperCapBoost;
} Chassis_Ctrl_Cmd_s;

#pragma pack()
//超级电容策略结构体

typedef enum {
  SAFETY_MODE=0,//安全模式，超电电压低于8伏时进入，大于18伏退出，底盘限制30W
  PASSIVE_MODE,//被动模式，超电电压正常时的工作模式
  ACTIVE_MODE,//，主动模式，主动使用超电能量
  CHARGING_MODE,//充电模式，衰减底盘功率，保障电容电压健康
  FORCED_CHARGING_MODE,//强制充电模式，更极端的功率衰减，强制超电快速充电
} SuperCapMode;

typedef struct {
  float k0;
  float k1;
  float k2;
  float k3;
  float k4;
  float k5;
}Power_Param_3508_s ;

// 机器人底盘修改的参数,单位为mm(毫米)
typedef struct {
  float wheel_base;                     // 纵向轴距(前进后退方向)
  float track_width;                    // 横向轮距(左右平移方向)
  float center_gimbal_offset_x;         // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
  float center_gimbal_offset_y;         // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
  float wheel_radius;                   // 轮子半径
  float wheel_reduction_ratio;          // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
  Power_Param_3508_s power_param;       //3508功率模型参数，采用中科大的模型
} Chassis_Param_s;

typedef struct {
  Chassis_Param_s chassis_param;
  Motor_Init_Config_s wheel_motor_config[4];
  Motor_Init_Config_s leg_motor_config[2];
  PID_Init_Config_s follow_pid;
  struct {
    uint8_t can_id;
    uint8_t mst_id;
    FDCAN_HandleTypeDef *can_handle;
  } external_imu; // External IMU sensor configuration
  SuperCap_Init_Config_s super_cap_config;
  TOFSense_Init_Config_s tof_sense_config;
} Chassis_Init_Config_s;

typedef struct {
  Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
  DJIMotorInstance* wheel_motor[4];// left right forward back
  DMMotorInstance* leg_motor[2];
  external_imu_t* chassis_external_imu;  // 底盘外部IMU数据
  SuperCapInstance* super_cap;
  SuperCapMode super_cap_mode;
  TOFSenseInstance* tof_sense;
} ChassisInstance;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 *
 */
void ChassisTask();