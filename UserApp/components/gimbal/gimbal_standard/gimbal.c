/**
******************************************************************************
* @file    gimbal.h
* @author  NeoZeng
* @author  Annotation and Modification By Enhao Zhang
* @date    2025/10/10
* @copyright Copyright (c) SHU SRM 2025 all rights reserved
* @brief   Standard Gimbal Module
******************************************************************************
* @attention
******************************************************************************
*/

#include "gimbal.h"
#include "user_lib.h"
#include "ins_task.h"

static GimbalInstance* gimbal;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;  // 声明但不初始化
static Gimbal_Mode_e gimbal_mode_last;
static float last_yaw_cmd=0.0f;

//保证云台每一次旋转不会超过180度
float wrap180(float now, float last) {
  float diff = now - last;
  // 1. 先把diff限制在 -360 到 360 之间 (其实fmod这一步可以省略，直接用while或if更直观)
  // 2. 核心逻辑：找最短路径
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return last + diff;
}

  // static BMI088Instance *bmi088; // 云台IMU
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config) {
  GimbalInstance* gimbal_instance = (GimbalInstance*)zmalloc(sizeof(GimbalInstance));
  gimbal_instance->gimbal_IMU_data = INS_Init(&gimbal_init_config->imu_init_config);  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
  // gimbal_instance->gimbal_hi05_data = HI05_Init(gimbal_init_config->hi05_uart_handle);

  // YAW控制器参数配置
   gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];
  // gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr=&gimbal_instance->gimbal_hi05_data->YawTotalAngle;
  //  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr=&gimbal_instance->gimbal_hi05_data->gyr[2];

  // YAW控制器设置配置
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
  smc_init(&gimbal_instance->YawSMC, 20.0f, 120.0f, 0.0f, 0.001f, 25000.0f, 0.8f, 0.5f);
  // PITCH控制器参数配置
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];

  // gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr=&gimbal_instance->gimbal_hi05_data->pitch;
  // gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr=&gimbal_instance->gimbal_hi05_data->gyr[0];


  // PITCH控制器设置配置
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  gimbal_instance->yaw_motor = DJIMotorInit(&gimbal_init_config->yaw_motor_config);
  gimbal_instance->pitch_motor = DJIMotorInit(&gimbal_init_config->pitch_motor_config);

  gimbal = gimbal_instance;
  gimbal_ctrl_cmd = &gimbal->gimbal_ctrl_cmd;  // 在运行时初始化指针
  return gimbal_instance;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
  // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    // 停止
    DJIMotorStop(gimbal->yaw_motor);
    DJIMotorStop(gimbal->pitch_motor);
    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
  } else {
    DJIMotorEnable(gimbal->yaw_motor);
    DJIMotorEnable(gimbal->pitch_motor);
    if (gimbal_mode_last==GIMBAL_POWER_OFF){
        gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
        gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
    }
    gimbal_ctrl_cmd->yaw = wrap180(gimbal_ctrl_cmd->yaw, last_yaw_cmd);
    DJIMotorSetPIDRef(gimbal->yaw_motor, gimbal_ctrl_cmd->yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈
    smc_tick(&gimbal->YawSMC,gimbal->gimbal_IMU_data->YawTotalAngle,gimbal->gimbal_IMU_data->Gyro[2],gimbal_ctrl_cmd->yaw);
    gimbal->yaw_motor->motor_controller.final_output = 0.3f*gimbal->YawSMC.u;
    DJIMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);
    last_yaw_cmd = gimbal_ctrl_cmd->yaw;
  }
  gimbal_mode_last = gimbal_ctrl_cmd->gimbal_mode;

  // 在合适的地方添加pitch重力补偿前馈力矩
  // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
  // ...
}