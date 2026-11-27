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
static float big_pitch_target_rad;  // 大 Pitch 编码器目标，单位 rad，由 UpdateGimbalLimits() 计算

static float PitchTorqueToPID(const DMMotorInstance *motor, float direction, float torque_nm)
{
    float output = direction * torque_nm;
    return motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -output : output;
}

static void ClearPitchFeedforward(void)
{
    gimbal->gravity_feedforward.big_pitch_feedforward = 0.0f;
    gimbal->gravity_feedforward.small_pitch_feedforward = 0.0f;
    gimbal->big_pitch_burst_feedforward = 0.0f;
    gimbal->small_pitch_burst_feedforward = 0.0f;
    gimbal->big_pitch_total_feedforward = 0.0f;
    gimbal->small_pitch_total_feedforward = 0.0f;
}

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

    // 上述力矩按电机编码器方向计算，先转换到 PID 输出坐标。
    // DMMotorTask 发帧前会按 motor_reverse_flag 再次反向。
    if (big_motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        big_feedforward = -big_feedforward;
    if (small_motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        small_feedforward = -small_feedforward;

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

static void UpdatePitchFeedforward(void)
{
    if (!UpdateGimbalGravity()) {
        ClearPitchFeedforward();
        return;
    }
    const GravityFeedforward_Init_Config_s *config = &gimbal->gravity_feedforward.config;
    // 与重力补偿相同：物理关节方向 -> 编码器坐标 -> PID坐标。
    // 连发补偿是独立恒定力矩，不乘姿态余弦，不随控制周期累加。
    gimbal->big_pitch_burst_feedforward = gimbal_ctrl_cmd->burst_feedforward_enabled ?
        PitchTorqueToPID(gimbal->big_pitch_motor, config->big_pitch_direction,
                         gimbal->burst_config.big_pitch_nm) : 0.0f;
    gimbal->small_pitch_burst_feedforward = gimbal_ctrl_cmd->burst_feedforward_enabled ?
        PitchTorqueToPID(gimbal->small_pitch_motor, config->small_pitch_direction,
                         gimbal->burst_config.small_pitch_nm) : 0.0f;
    gimbal->big_pitch_total_feedforward = gimbal->gravity_feedforward.big_pitch_feedforward +
        gimbal->big_pitch_burst_feedforward;
    gimbal->small_pitch_total_feedforward = gimbal->gravity_feedforward.small_pitch_feedforward +
        gimbal->small_pitch_burst_feedforward;
}
static void ResetController(Motor_Controller_s *controller)
{
    PIDClear(&controller->angle_PID);
    PIDClear(&controller->speed_PID);
    PIDClear(&controller->current_PID);
    controller->pid_ref = 0.0f;
    controller->final_output = 0.0f;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 通用编码器限位，每个轴调用一次。
// limit:        该轴的限位参数（方向、软限位、硬故障余量）
// feedback_rad: 指令域当前角度，单位 rad；编码器闭环轴直接传 total_angle
// fault:        该轴硬限位锁存标志
// target_rad:   指令域目标，单位 rad；返回时夹到允许窗口
static AxisLimit_State_e ApplyAxisLimit(DMMotorInstance *motor, const AxisLimit_Config_s *limit,
                                        float feedback_rad, AxisLimit_State_e *fault,
                                        float *target_rad)
{
    // 反馈还没就绪时不做判断
    if (!(motor->dt > 0.0f) || !isfinite(motor->measure.total_angle))
        return AXIS_LIMIT_OK;

    // 编码器实测角，单位 rad，与 gimbal.h 中填写的 MIN/MAX 同一坐标，不减任何零位
    const float joint_rad = motor->measure.total_angle;

    // 硬限位：超过软限位 margin 后锁存零力矩，回到软限位内才恢复
    if (joint_rad > limit->max_rad + limit->margin_rad ||
        joint_rad < limit->min_rad - limit->margin_rad) {
        if (*fault == AXIS_LIMIT_OK) {
            ResetController(&motor->motor_controller);
            DMMotorStop(motor);
        }
        *fault = AXIS_LIMIT_FAULT;
        return AXIS_LIMIT_FAULT;
    }

    if (*fault == AXIS_LIMIT_FAULT) {
        if (joint_rad <= limit->max_rad && joint_rad >= limit->min_rad && isfinite(feedback_rad)) {
            ResetController(&motor->motor_controller);
            *fault = AXIS_LIMIT_OK;
            return AXIS_LIMIT_RECOVERED;
        }
        return AXIS_LIMIT_FAULT;
    }

    // 软限位需要有效的指令域反馈才能把关节窗口映射过去
    if (!isfinite(feedback_rad) || !isfinite(*target_rad))
        return AXIS_LIMIT_OK;

    // 关节窗口 -> 指令域窗口；每周期重算，自动抵消上游关节/底盘转动。
    const float offset_rad = feedback_rad - joint_rad;
    float cmd_min_rad, cmd_max_rad;
    if (limit->direction >= 0.0f) {
        cmd_min_rad = limit->min_rad + offset_rad;
        cmd_max_rad = limit->max_rad + offset_rad;
    } else {
        cmd_min_rad = feedback_rad + joint_rad - limit->max_rad;
        cmd_max_rad = feedback_rad + joint_rad - limit->min_rad;
    }

    // 直接夹指令，防止在限位外继续累积
    *target_rad = ClampFloat(*target_rad, cmd_min_rad, cmd_max_rad);
    return AXIS_LIMIT_OK;
}

static void UpdateGimbalLimits(void)
{
    INS_t *imu = gimbal->gimbal_IMU_data;

    // 小 Yaw：指令域是 IMU Yaw，deg -> rad 后统一用 rad 限位
    float target_rad = gimbal_ctrl_cmd->yaw * DEGREE_2_RAD;
    AxisLimit_State_e state = ApplyAxisLimit(
        gimbal->small_yaw_motor, &gimbal->small_yaw_limit, imu->YawTotalAngle * DEGREE_2_RAD,
        &gimbal->small_yaw_hard_fault, &target_rad);
    if (state == AXIS_LIMIT_RECOVERED)
        gimbal_ctrl_cmd->yaw = imu->YawTotalAngle;  // 恢复时重同步，避免目标跳变
    else if (state == AXIS_LIMIT_OK)
        gimbal_ctrl_cmd->yaw = target_rad * RAD_2_DEGREE;

    // 小 Pitch：指令域是 IMU Pitch
    target_rad = gimbal_ctrl_cmd->pitch * DEGREE_2_RAD;
    state = ApplyAxisLimit(gimbal->small_pitch_motor, &gimbal->small_pitch_limit,
                           imu->PitchTotalAngle * DEGREE_2_RAD, &gimbal->small_pitch_hard_fault, &target_rad);
    if (state == AXIS_LIMIT_RECOVERED)
        gimbal_ctrl_cmd->pitch = imu->PitchTotalAngle;
    else if (state == AXIS_LIMIT_OK)
        gimbal_ctrl_cmd->pitch = target_rad * RAD_2_DEGREE;

    // 大 Pitch：指令域就是编码器本身，目标直接由姿态宏决定，在这里做限位。
    big_pitch_target_rad = gimbal_ctrl_cmd->big_pitch_pose == GIMBAL_BIG_PITCH_VERTICAL
                               ? UP_ANGLE
                               : DOWN_ANGLE;
    ApplyAxisLimit(gimbal->big_pitch_motor, &gimbal->big_pitch_limit,
                   gimbal->big_pitch_motor->measure.total_angle, &gimbal->big_pitch_hard_fault,
                   &big_pitch_target_rad);
}

static void StopGimbal(void)
{
    ClearPitchFeedforward();
    DMMotorStop(gimbal->big_pitch_motor);
    DMMotorStop(gimbal->small_pitch_motor);
    DMMotorStop(gimbal->big_yaw_motor);
    DMMotorStop(gimbal->small_yaw_motor);
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
    gimbal->burst_config = gimbal_init_config->burst_config;

    // IMU 闭环轴的指令域方向：编码器 total_angle 增大时，IMU 角度增大填 +1，减小填 -1。
    // 标定：POWER_OFF 后手动转动关节，同时观察编码器 total_angle 与 IMU 角度；
    //       两者同向变化填 +1.0f，反向填 -1.0f。填反会导致软限位窗口镜像。
    // 大 Pitch 的指令域就是编码器本身，方向固定 +1，不需要此宏。

    AxisLimit_Config_s *small_yaw_limit_config = &gimbal->small_yaw_limit;
    small_yaw_limit_config->direction = (-1.0f);//优先标定确保IMU和电机方向对应
    small_yaw_limit_config->min_rad = (-1.2f);
    small_yaw_limit_config->max_rad = (0.0f);
    small_yaw_limit_config->margin_rad = (0.10f);// 约 5.7°，超软限位多少判定硬故障

    AxisLimit_Config_s *small_pitch_limit_config = &gimbal->small_pitch_limit;
    small_pitch_limit_config->direction = (-1.0f);
    small_pitch_limit_config->min_rad = (0.0f);
    small_pitch_limit_config->max_rad = (2.6f);
    small_pitch_limit_config->margin_rad = (0.10f);

    AxisLimit_Config_s *big_pitch_limit_config = &gimbal->big_pitch_limit;
    big_pitch_limit_config->direction = 1.0f;
    big_pitch_limit_config->min_rad = UP_ANGLE;    // 标定的上位置，编码器值较小
    big_pitch_limit_config->max_rad = DOWN_ANGLE;  // 标定的下位置，编码器值较大
    big_pitch_limit_config->margin_rad = (0.10f);
    gimbal->small_yaw_hard_fault = AXIS_LIMIT_OK;
    gimbal->small_pitch_hard_fault = AXIS_LIMIT_OK;
    gimbal->big_pitch_hard_fault = AXIS_LIMIT_OK;

    Motor_Init_Config_s *small_pitch_config = &gimbal_init_config->small_pitch_motor_config;
    small_pitch_config->controller_param_init_config.other_angle_feedback_ptr =
        &gimbal->gimbal_IMU_data->PitchTotalAngle;
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

    // 重力+连发补偿前馈指针绑定；重力单独保留供原VOFA通道观察。
    // CURRENT_FEEDFORWARD 在速度 PID 后叠加 N*m；SPEED_FEEDFORWARD 的单位是 rad/s。
    gimbal->big_pitch_motor->motor_controller.current_feedforward_ptr =
        &gimbal->big_pitch_total_feedforward;
    gimbal->big_pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
    gimbal->small_pitch_motor->motor_controller.current_feedforward_ptr =
        &gimbal->small_pitch_total_feedforward;
    gimbal->small_pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;

    gimbal->big_yaw_motor = DMMotorInit(big_yaw_config);
    DMMotorStop(gimbal->big_yaw_motor);

    gimbal->small_yaw_motor = DMMotorInit(small_yaw_config);
    DMMotorStop(gimbal->small_yaw_motor);

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
        gimbal_ctrl_cmd->pitch = gimbal->gimbal_IMU_data->PitchTotalAngle;
        gimbal_ctrl_cmd->big_yaw = gimbal->big_yaw_motor->measure.total_angle / DEGREE_2_RAD;
        gimbal_enabled = 1;
        return;
    }
    // 必须先更新前馈，再计算两轴 PID。
    UpdatePitchFeedforward();
    
    // 各轴编码器机械限位；超过硬限位时锁存零力矩。
    UpdateGimbalLimits();
    // 小轴角度反馈为 deg，速度反馈为 rad/s；角度 PID 的输出须按 rad/s 整定。
    if (gimbal->small_yaw_hard_fault == AXIS_LIMIT_OK)
        DMMotorSetPIDRef(gimbal->small_yaw_motor, gimbal_ctrl_cmd->yaw);
    if (gimbal->small_pitch_hard_fault == AXIS_LIMIT_OK)
        DMMotorSetPIDRef(gimbal->small_pitch_motor, gimbal_ctrl_cmd->pitch);
    // 大 Pitch 的目标已在 UpdateGimbalLimits() 中按机械限位夹紧。
    if (gimbal->big_pitch_hard_fault == AXIS_LIMIT_OK)
        DMMotorSetPIDRef(gimbal->big_pitch_motor, big_pitch_target_rad);

    DMMotorSetPIDRef(gimbal->big_yaw_motor, gimbal_ctrl_cmd->big_yaw * DEGREE_2_RAD);

    if (gimbal->big_pitch_hard_fault == AXIS_LIMIT_OK)
        DMMotorEnable(gimbal->big_pitch_motor);
    if (gimbal->small_pitch_hard_fault == AXIS_LIMIT_OK)
        DMMotorEnable(gimbal->small_pitch_motor);
    DMMotorEnable(gimbal->big_yaw_motor);
    if (gimbal->small_yaw_hard_fault == AXIS_LIMIT_OK)
        DMMotorEnable(gimbal->small_yaw_motor);
}
