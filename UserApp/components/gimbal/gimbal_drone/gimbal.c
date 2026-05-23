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

// static BMI088Instance *bmi088; // 云台IMU
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config) {
  GimbalInstance* gimbal_instance = (GimbalInstance*)zmalloc(sizeof(GimbalInstance));
  gimbal_instance->gimbal_IMU_data = INS_Init(&gimbal_init_config->imu_init_config);  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

  /*如果yaw要用imu控制*/
  // YAW控制器参数配置
  // gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr =
  //     &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  // gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr =
  //     &gimbal_instance->gimbal_IMU_data->Gyro[2];

  // YAW控制器设置配置
  // gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  // gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  // gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  // gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  /*如果yaw要用电机自身回传值控制*/
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // PITCH控制器参数配置
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];

  // PITCH控制器设置配置
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  gimbal_instance->yaw_motor = DJIMotorInit(&gimbal_init_config->yaw_motor_config);
  gimbal_instance->pitch_motor = DMMotorInit(&gimbal_init_config->pitch_motor_config);

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
    DMMotorStop(gimbal->pitch_motor);
    //gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle; //如果你要imu控制
    gimbal_ctrl_cmd->yaw = gimbal->yaw_motor->measure.total_angle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
  } else {
    DJIMotorEnable(gimbal->yaw_motor);
    DMMotorEnable(gimbal->pitch_motor);
    //pid调参测试用
    //gimbal_ctrl_cmd->yaw=40*sin(DWT_GetTimeline_s()*2.5f);
    DJIMotorSetPIDRef(gimbal->yaw_motor, gimbal_ctrl_cmd->yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DMMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);


    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...
    // --- A. 获取欧拉角 (弧度制) ---
    // float roll_rad = gimbal->gimbal_IMU_data->Roll * (3.14159265f / 180.0f);
    // float pitch_rad = gimbal->gimbal_IMU_data->Pitch * (3.14159265f / 180.0f);
    //
    // // --- B. 角度环 PID 计算 (得到地球坐标系下的期望角速度) ---
    // float yaw_v_earth = PIDCalculate(&gimbal->yaw_motor->motor_controller.angle_PID,
    //                                  gimbal->gimbal_IMU_data->YawTotalAngle,
    //                                  gimbal_ctrl_cmd->yaw);
    //
    // float pitch_v_earth = PIDCalculate(&gimbal->pitch_motor->motor_controller.angle_PID,
    //                                    gimbal->gimbal_IMU_data->Pitch,
    //                                    gimbal_ctrl_cmd->pitch);
    //
    // // --- C. Roll 补偿与坐标变换 ---
    // float cos_r = cosf(roll_rad);
    // float sin_r = sinf(roll_rad);
    // float cos_p = cosf(pitch_rad);
    //
    // // 安全防线：防止 Pitch 接近 90 度时分母为 0 导致飞车
    // if (cos_p < 0.1f) cos_p = 0.1f;
    //
    // // Roll 补偿公式 (省略 pitch 补偿项/cos_p，提高稳定性)
    // float yaw_v_motor = (yaw_v_earth * cos_r - pitch_v_earth * sin_r);
    // float pitch_v_motor = pitch_v_earth * cos_r + yaw_v_earth * sin_r;
    //
    // // --- D. 下发速度指令给电机 ---
    // // 因为电机已经被配置为双闭环模式，此时传入的是速度目标值
    // DJIMotorSetPIDRef(gimbal->yaw_motor, yaw_v_motor);
    // DMMotorSetPIDRef(gimbal->pitch_motor, pitch_v_motor);
  }
}