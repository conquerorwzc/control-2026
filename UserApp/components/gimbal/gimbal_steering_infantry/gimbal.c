#include "gimbal.h"
#include "user_lib.h"
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

static GimbalInstance* gimbal;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;  // 声明但不初始化

static int disable_2_enable_flag;

// static BMI088Instance *bmi088; // 云台IMU
GimbalInstance* GimbalInit(Gimbal_Init_Config_s* gimbal_init_config) {
  GimbalInstance* gimbal_instance = (GimbalInstance*)zmalloc(sizeof(GimbalInstance));
  gimbal_instance->gimbal_IMU_data = INS_Init(&gimbal_init_config->imu_init_config);  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
  gimbal_instance->gimbal_hi05_data = HI05_Init(gimbal_init_config->hi05_uart_handle);

  // YAW控制器参数配置
#if defined(BMI088_CTRL)
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];

#elif defined(HI05_CTRL)
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr=&gimbal_instance->gimbal_hi05_data->YawTotalAngle;
  gimbal_init_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr=&gimbal_instance->gimbal_hi05_data->gyr[2];
#endif

  // YAW控制器设置配置
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

#if defined(BMI088_CTRL)
  // PITCH控制器参数配置
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];
#elif defined(HI05_CTRL)
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr=&gimbal_instance->gimbal_hi05_data->pitch;
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr=&gimbal_instance->gimbal_hi05_data->gyr[0];
#endif
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
  // if (!gimbal) return;
  // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    // 停止
    DJIMotorStop(gimbal->yaw_motor);
    DMMotorStop(gimbal->pitch_motor);
    disable_2_enable_flag=0;
  }else {
    DJIMotorEnable(gimbal->yaw_motor);
    DMMotorEnable(gimbal->pitch_motor);

    // if (gimbal->pitch_motor->stop_flag==MOTOR_STOP) {
    //   DMMotorEnable(gimbal->pitch_motor);
    // }

    if (disable_2_enable_flag==0) {
      //重新使能后yaw屏蔽失能时的控制，pitch回中
      gimbal_ctrl_cmd->yaw=gimbal->gimbal_IMU_data->YawTotalAngle;
      gimbal_ctrl_cmd->pitch=0;
      disable_2_enable_flag=1;
    }

    // 调用核心控制函数
    DJIMotorSetPIDRef(gimbal->yaw_motor, gimbal_ctrl_cmd->yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈GimbalMotorAbsoluteAngleControl(gimbal);
    DMMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);
    //gimbal_ctrl_cmd->pitch=10.0f*sinf((float)HAL_GetTick()/100.0f);
    //DMMotorSetPIDRef(gimbal->pitch_motor, 20.0f*powf(sinf((float)HAL_GetTick()/100.0f),3)-5.0f);
  }
  // 在合适的地方添加pitch重力补偿前馈力矩
  // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
  // ...
}
