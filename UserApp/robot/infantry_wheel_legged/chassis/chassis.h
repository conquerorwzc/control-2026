/**
 ******************************************************************************
 * @file    chassis.h
 * @author  Enhao Zhang
 * @date    2025/8/8
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "parallel_leg.h"
#include "controller.h"
#include "message_center.h"
#include "robot_def.h"

typedef struct {
  Leg_Init_Config_s leg_init_config[2];
  PID_Init_Config_s delta_theta_PID_config;
  PID_Init_Config_s roll_PID_config;
  PID_Init_Config_s yaw_PID_config;
} Chassis_Init_Config_s;

typedef struct {
  LegInstance* leg[2];
  attitude_t* chassis_IMU_data;

  PIDInstance delta_theta_PID; // Only use PD
  PIDInstance roll_PID; // Only use P
  PIDInstance yaw_PID; // Only use PD

  float delta_theta_comp;
  float roll_comp;
  float yaw_comp;

  Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
  Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

  Publisher_t* chassis_pub; // 用于发布,底盘的数据
  Subscriber_t* chassis_sub; // 用于订阅,底盘的控制命令
} ChassisInstance;

void ChassisInit();

void ChassisTask();
#endif //CHASSIS_CONTROL_H