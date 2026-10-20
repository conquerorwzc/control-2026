//
// Created by Dell on 2026/9/24.
//

#include "gimbal.h"
#include "general_def.h"
#include "ins_task.h"
#include <math.h>
#include <stdlib.h>

static GimbalInstance *gimbal; //云台实例
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd; // 声明但不初始化
static uint8_t gimbal_enabled;  //使能标志位，上电保持原有姿态有关

static int UpdateGimbalGravity(void)
{
    DMMotorInstance *big_motor = gimbal->big_pitch_motor;
    DMMotorInstance *small_motor = gimbal->small_pitch_motor;
    GravityFeedforwardInstance *instance = &gimbal->gravity_feedforward;
    const GravityFeedforward_Init_Config_s *config = &instance->config;

    float big_angle = config->big_pitch_direction *
        (big_motor->measure.total_angle - config->big_pitch_zero_rad);

    float small_relative_angle = config->small_pitch_direction *
        (small_motor->measure.total_angle - config->small_pitch_zero_rad);

    float end_angle = big_angle + small_relative_angle;
    float end_gravity_nm =
        config->small_pitch_scale_nm * cosf(end_angle);

    float big_feedforward = config->big_pitch_direction *
        (config->big_pitch_scale_nm * cosf(big_angle) + end_gravity_nm);

    float small_feedforward =
        config->small_pitch_direction * end_gravity_nm;

    // 统一安全检查，失败时清零两轴补偿。
    if (!DaemonIsOnline(big_motor->daemon) ||
        !DaemonIsOnline(small_motor->daemon) ||
        !(big_motor->dt > 0.0f) ||
        !(small_motor->dt > 0.0f) ||
        !isfinite(big_feedforward) ||
        !isfinite(small_feedforward)) {
        instance->big_pitch_feedforward = 0.0f;
        instance->small_pitch_feedforward = 0.0f;
        return 0;
    }

    instance->big_pitch_feedforward = big_feedforward;
    instance->small_pitch_feedforward = small_feedforward;
    return 1;
}
static void ResetController(Motor_Controller_s *controller)
{
    PIDClear(&controller->angle_PID);
    PIDClear(&controller->speed_PID);
    PIDClear(&controller->current_PID);
    controller->pid_ref = 0.0f;
    controller->final_output = 0.0f;
}

static void StopGimbal(void)
{
    gimbal->gravity_feedforward.big_pitch_feedforward = 0.0f;
    gimbal->gravity_feedforward.small_pitch_feedforward = 0.0f;
    DMMotorStop(gimbal->big_pitch_motor);
    DMMotorStop(gimbal->small_pitch_motor);
    DMMotorStop(gimbal->big_yaw_motor);
    DJIMotorStop(gimbal->small_yaw_motor);
    if (gimbal_enabled) {
        ResetController(&gimbal->big_pitch_motor->motor_controller);
        ResetController(&gimbal->small_pitch_motor->motor_controller);
        ResetController(&gimbal->big_yaw_motor->motor_controller);
        ResetController(&gimbal->small_yaw_motor->motor_controller);
    }
    gimbal_enabled = 0;
}

GimbalInstance *GimbalInit(Gimbal_Init_Config_s *gimbal_init_config)
{
    gimbal = calloc(1, sizeof(*gimbal));

    gimbal_ctrl_cmd = &gimbal->gimbal_ctrl_cmd;
    gimbal_ctrl_cmd->gimbal_mode = GIMBAL_POWER_OFF;

    gimbal->gimbal_IMU_data = INS_Init(&gimbal_init_config->imu_init_config);

    gimbal->gravity_feedforward.config = gimbal_init_config->gravity_config;

    Motor_Init_Config_s *small_pitch_config = &gimbal_init_config->small_pitch_motor_config;
    small_pitch_config->controller_param_init_config.other_angle_feedback_ptr = &gimbal->gimbal_IMU_data->Pitch;
    small_pitch_config->controller_param_init_config.other_speed_feedback_ptr = &gimbal->gimbal_IMU_data->Gyro[0];
    small_pitch_config->controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    small_pitch_config->controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    small_pitch_config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    small_pitch_config->controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;

    Motor_Init_Config_s *big_pitch_config = &gimbal_init_config->big_pitch_motor_config;
    big_pitch_config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    big_pitch_config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    big_pitch_config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    big_pitch_config->controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;

    Motor_Init_Config_s *big_yaw_config = &gimbal_init_config->big_yaw_motor_config;
    big_yaw_config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    big_yaw_config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    big_yaw_config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    big_yaw_config->controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;


    Motor_Init_Config_s *small_yaw_config = &gimbal_init_config->small_yaw_motor_config;
    small_yaw_config->controller_param_init_config.other_angle_feedback_ptr = &gimbal->gimbal_IMU_data->YawTotalAngle;
    small_yaw_config->controller_param_init_config.other_speed_feedback_ptr = &gimbal->gimbal_IMU_data->Gyro[2];
    small_yaw_config->controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    small_yaw_config->controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    small_yaw_config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    small_yaw_config->controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;

    PIDInit(&gimbal->yaw_follow,&gimbal_init_config->Yaw_Follow_PID);

    gimbal->big_pitch_motor = DMMotorInit(big_pitch_config);
    DMMotorStop(gimbal->big_pitch_motor);

    gimbal->small_pitch_motor = DMMotorInit(small_pitch_config);
    DMMotorStop(gimbal->small_pitch_motor);

    // 重力补偿前馈指针绑定
    // CURRENT_FEEDFORWARD 在速度 PID 后叠加 N*m；SPEED_FEEDFORWARD 的单位是 rad/s。
    gimbal->big_pitch_motor->motor_controller.current_feedforward_ptr =
        &gimbal->gravity_feedforward.big_pitch_feedforward;
    gimbal->big_pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
    gimbal->small_pitch_motor->motor_controller.current_feedforward_ptr =
        &gimbal->gravity_feedforward.small_pitch_feedforward;
    gimbal->small_pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;

    gimbal->big_yaw_motor = DMMotorInit(big_yaw_config);
    DMMotorStop(gimbal->big_yaw_motor);

    gimbal->small_yaw_motor = DJIMotorInit(small_yaw_config);
    DJIMotorStop(gimbal->small_yaw_motor);

    return gimbal;
}
void GimbalTask(void)
{
    if (gimbal_ctrl_cmd->gimbal_mode == GIMBAL_POWER_OFF) {
        StopGimbal();
        return;
    }

    if (!gimbal_enabled) {        //保持当前姿态
        ResetController(&gimbal->big_pitch_motor->motor_controller);
        ResetController(&gimbal->small_pitch_motor->motor_controller);
        ResetController(&gimbal->big_yaw_motor->motor_controller);
        ResetController(&gimbal->small_yaw_motor->motor_controller);
        gimbal_ctrl_cmd->yaw = gimbal->gimbal_IMU_data->YawTotalAngle;
        gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->Pitch;
        gimbal_ctrl_cmd->big_yaw = gimbal->big_yaw_motor->measure.total_angle / DEGREE_2_RAD;
        gimbal_enabled = 1;
        return;
    }
    // 必须先更新前馈，再计算两轴 PID。
    UpdateGimbalGravity();
    // 小轴角度反馈为 deg，速度反馈为 rad/s；角度 PID 的输出须按 rad/s 整定。
    DJIMotorSetPIDRef(gimbal->small_yaw_motor, gimbal_ctrl_cmd->yaw);
    DMMotorSetPIDRef(gimbal->small_pitch_motor, gimbal_ctrl_cmd->pitch);
    if (gimbal_ctrl_cmd->big_pitch_pose == GIMBAL_BIG_PITCH_VERTICAL)
    {
        DMMotorSetPIDRef(gimbal->big_pitch_motor, UP_ANGLE);
    }
    else
    {
        DMMotorSetPIDRef(gimbal->big_pitch_motor, DOWN_ANGLE);
    }
    DMMotorSetPIDRef(gimbal->big_yaw_motor, gimbal_ctrl_cmd->big_yaw * DEGREE_2_RAD);
    DMMotorEnable(gimbal->big_pitch_motor);
    DMMotorEnable(gimbal->small_pitch_motor);
    DMMotorEnable(gimbal->big_yaw_motor);
    DJIMotorEnable(gimbal->small_yaw_motor);
}
