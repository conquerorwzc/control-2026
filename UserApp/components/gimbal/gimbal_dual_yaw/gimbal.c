/**
 ******************************************************************************
 * @file    gimbal.c
 * @author  SRM-Control 2026
 * @brief   Dual Yaw Gimbal Module - 双yaw串联云台模块实现
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "gimbal.h"
#include "user_lib.h"
#include "ins_task.h"

static GimbalDualYawInstance* gimbal;
static Gimbal_Ctrl_Cmd_s* gimbal_ctrl_cmd;
static Gimbal_Mode_e gimbal_mode_last;
static float last_yaw_cmd = 0.0f;

/**
 * @brief 保证云台每一次旋转不会超过180度
 */
static float wrap180(float now, float last) {
  float diff = now - last;
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return last + diff;
}

/**
 * @brief 初始化双yaw云台
 */
GimbalDualYawInstance* GimbalDualYawInit(Gimbal_Dual_Yaw_Init_Config_s* gimbal_init_config) {
  GimbalDualYawInstance* gimbal_instance = (GimbalDualYawInstance*)zmalloc(sizeof(GimbalDualYawInstance));

  // 1. 初始化IMU
  gimbal_instance->gimbal_IMU_data = INS_Init(&gimbal_init_config->imu_init_config);

  // 2. 配置主yaw电机（GM6020，使用IMU反馈）
  gimbal_init_config->yaw_master_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_master_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];
  gimbal_init_config->yaw_master_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_master_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_master_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_master_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // 3. 配置副yaw电机（GM6020，使用主yaw角度+偏移量作为反馈）
  // 副yaw电机同样使用IMU反馈，但控制目标会加上偏移量
  gimbal_init_config->yaw_slave_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->YawTotalAngle;
  gimbal_init_config->yaw_slave_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[2];
  gimbal_init_config->yaw_slave_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_slave_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->yaw_slave_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->yaw_slave_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // 4. 配置pitch电机（J4310，使用IMU反馈）
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Pitch;
  gimbal_init_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr =
      &gimbal_instance->gimbal_IMU_data->Gyro[0];
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  gimbal_init_config->pitch_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  // 5. 初始化电机
  gimbal_instance->yaw_master_motor = DJIMotorInit(&gimbal_init_config->yaw_master_motor_config);
  gimbal_instance->yaw_slave_motor = DJIMotorInit(&gimbal_init_config->yaw_slave_motor_config);
  gimbal_instance->pitch_motor = DMMotorInit(&gimbal_init_config->pitch_motor_config);

  // 6. 初始化副yaw偏移量为0
  gimbal_instance->yaw_slave_offset = 0.0f;

  gimbal = gimbal_instance;
  gimbal_ctrl_cmd = &gimbal->gimbal_ctrl_cmd;
  return gimbal_instance;
}

/**
 * @brief 双yaw云台控制核心任务
 */
void GimbalDualYawTask() {
  if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
    // 停止所有电机
    DJIMotorStop(gimbal->yaw_master_motor);
    DJIMotorStop(gimbal->yaw_slave_motor);
    DMMotorStop(gimbal->pitch_motor);
    gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
    gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
  } else {
    // 使能所有电机
    DJIMotorEnable(gimbal->yaw_master_motor);
    DJIMotorEnable(gimbal->yaw_slave_motor);
    DMMotorEnable(gimbal->pitch_motor);

    // 从POWER_OFF模式切换回来时，同步当前角度
    if (gimbal_mode_last == GIMBAL_POWER_OFF) {
      gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
      gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
    }

    // 处理yaw角度wrap
    gimbal_ctrl_cmd->yaw = wrap180(gimbal_ctrl_cmd->yaw, last_yaw_cmd);

    // 计算主yaw控制量
    DJIMotorSetPIDRef(gimbal->yaw_master_motor, gimbal_ctrl_cmd->yaw);

    // 计算副yaw控制量（主yaw + 偏移量）
    float yaw_slave_ref = gimbal_ctrl_cmd->yaw + gimbal->yaw_slave_offset;
    DJIMotorSetPIDRef(gimbal->yaw_slave_motor, yaw_slave_ref);

    // 计算pitch控制量
    DMMotorSetPIDRef(gimbal->pitch_motor, gimbal_ctrl_cmd->pitch);

    last_yaw_cmd = gimbal_ctrl_cmd->yaw;
  }
  gimbal_mode_last = gimbal_ctrl_cmd->gimbal_mode;
}
