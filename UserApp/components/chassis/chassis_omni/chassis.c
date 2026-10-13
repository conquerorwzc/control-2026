/**
 * @file    chassis.c
 * @author  NeoZeng
 * @author  Annotation and Modification By SRM-Control 2026
 * @date    2025/10/10
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   Omni Chassis Module
 */
#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "referee.h"
#include "user_lib.h"
#include "FreeRTOS.h"
#include "task.h"

static ChassisInstance *chassis;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd; // 声明但不初始化
static Chassis_Param_s chassis_param;        // 声明为静态局部变量
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_wz;
static referee_info_t *referee_data;
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static PIDInstance follow_pid;
static uint8_t yaw_hold_active;
static float power;
static float k0, k1, k2, k3, k4, k5;     // 中科大的功率模型
static uint8_t super_cap_error_flag = 0; // 电容故障标志
static uint32_t observer_clock;

// 与速度观测器一致，避免 -Ofast 将 NaN/Inf 检查优化掉。
static uint8_t SpeedFinite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static void ResetSpeedHold()
{
    PIDClear(&chassis->vx_hold_pid);
    PIDClear(&chassis->vy_hold_pid);
    PIDClear(&chassis->wz_hold_pid);
    chassis->speed_hold_correction = (Speed_s){0};
    chassis->speed_hold_active = 0u;
}

/** @brief 三轴速度外环，目标前馈 + PID 修正；只修改局部逆解输入。 */
static void SpeedHoldControl()
{
    Speed_s target = {.vx = chassis_vx, .vy = chassis_vy, .wz = chassis_wz};
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF ||
        !SpeedFinite(target.vx) || !SpeedFinite(target.vy) || !SpeedFinite(target.wz))
    {
        target = (Speed_s){0};
        ResetSpeedHold();
    }
    else if (!chassis->enable_speed_hold || !chassis->speed_observer.valid)
        ResetSpeedHold(); // 暂退为原有速度前馈；不改变已有电机急停策略。
    else if (!chassis->speed_hold_active)
    {
        // 刚使能/恢复测速时先预置历史，下一周期再计算，避免长间隔积分和微分冲击。
        ResetSpeedHold();
        chassis->vx_hold_pid.Last_Measure = chassis->speed_observer.fused.vx;
        chassis->vy_hold_pid.Last_Measure = chassis->speed_observer.fused.vy;
        chassis->wz_hold_pid.Last_Measure = chassis->speed_observer.fused.wz;
        chassis->vx_hold_pid.Last_Err = target.vx - chassis->speed_observer.fused.vx;
        chassis->vy_hold_pid.Last_Err = target.vy - chassis->speed_observer.fused.vy;
        chassis->wz_hold_pid.Last_Err = target.wz - chassis->speed_observer.fused.wz;
        chassis->speed_hold_active = 1u;
    }
    else
    {
        Speed_s correction = {
            .vx = PIDCalculate(&chassis->vx_hold_pid, chassis->speed_observer.fused.vx, target.vx),
            .vy = PIDCalculate(&chassis->vy_hold_pid, chassis->speed_observer.fused.vy, target.vy),
            .wz = PIDCalculate(&chassis->wz_hold_pid, chassis->speed_observer.fused.wz, target.wz),
        };
        chassis->speed_hold_updated = 1u;
        if (SpeedFinite(correction.vx) && SpeedFinite(correction.vy) && SpeedFinite(correction.wz))
            chassis->speed_hold_correction = correction;
        else
            ResetSpeedHold();
    }
    chassis->target_velocity = target;
    chassis_vx = target.vx + chassis->speed_hold_correction.vx;
    chassis_vy = target.vy + chassis->speed_hold_correction.vy;
    chassis_wz = target.wz + chassis->speed_hold_correction.wz;
    chassis->control_velocity = (Speed_s){.vx = chassis_vx, .vy = chassis_vy, .wz = chassis_wz};
}

/** @brief SI 底盘速度逆解为电机轴速度参考，单位 deg/s。 */
static void OmniCalculate()
{
    const Speed_s body = {.vx = chassis_vx, .vy = chassis_vy, .wz = chassis_wz};
    float motor_speed[4];
    OmniInverse(&chassis->speed_observer.model, &body, motor_speed);
    vt_lf = motor_speed[0];
    vt_rf = motor_speed[1];
    vt_lb = motor_speed[2];
    vt_rb = motor_speed[3];
}

/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl()
{
    // 获取电机速度反馈，deg/s 转 rpm（下方功率公式再转 rad/s）
    float motor_speed_fdb[4];
    for (int i = 0; i < 4; i++)
    {
        motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
    }

    // 获取当前电机参考电流，单位为协议码值；下方功率公式内转换为 A。
    float motor_current_list[4];
    float motor_real_current_list[4];
    for (int i = 0; i < 4; i++)
    {
        motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
        chassis->observer_telemetry.motor_requested_current[i] = motor_current_list[i];
        motor_real_current_list[i] = (float)chassis->wheel_motor[i]->measure.real_current;
    }

    float initial_give_power[4] = {0.0f};      // 每个电机的初始估计功率
    float initial_give_real_power[4] = {0.0f}; // 每个电机的初始估计功率
    float initial_total_power = 0.0f;          // 估计初始总功率
    power = 0;

    // 计算每个电机的功率贡献
    for (int i = 0; i < 4; i++)
    {
        initial_give_power[i] =
            k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
            k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

        // 只累加正向功率
        if (initial_give_power[i] > 0)
        {
            initial_total_power += initial_give_power[i];
        }
    }

    // 计算每个电机的功率贡献
    for (int i = 0; i < 4; i++)
    {
        initial_give_real_power[i] =
            k0 + k1 * motor_real_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k3 * motor_real_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k4 * motor_real_current_list[i] / (16384.0f / 20.0f) * motor_real_current_list[i] / (16384.0f / 20.0f) +
            k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

        power += initial_give_real_power[i];
    }

    // 同周期诊断快照，不改变功率分配算法。
    chassis->observer_telemetry.requested_power = initial_total_power;
    chassis->observer_telemetry.power_limit_scale = 1.0f;
    // 功率超限时进行动态调整
    if (initial_total_power > (float)chassis_ctrl_cmd->max_power)
    {
        float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power; // 削减功率比例
        chassis->observer_telemetry.power_limit_scale = power_scale;
        float scaled_give_power[4];
        // 计算缩放后的功率目标
        for (int i = 0; i < 4; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale;
            chassis->wheel_motor[i]->scaled_give_power = scaled_give_power[i];
        }

        // 重新计算每个电机的电流参考值
        for (int i = 0; i < 4; i++)
        {
            // 二次方程系数计算，参数
            float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
            float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
            float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                      k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                      scaled_give_power[i] + k0;
            float discriminant = b * b - 4 * a * c; // 判别式
            if (discriminant >= 0)
            {
                float sqrt_disc = sqrtf(discriminant);
                float temp1 = (-b + sqrt_disc) / (2 * a);
                float temp2 = (-b - sqrt_disc) / (2 * a);

                // 选择最接近当前电流的解
                if (motor_current_list[i] > 0)
                {
                    motor_current_list[i] =
                        (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                            ? fminf(16000.f, temp1)
                            : fminf(16000.f, temp2);
                }
                else
                {
                    motor_current_list[i] =
                        (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                            ? fmaxf(-16000.f, temp1)
                            : fmaxf(-16000.f, temp2);
                }
            }
            else
            {
                // 无解时归零
                motor_current_list[i] = 0.0f;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)(motor_current_list[i]);
        chassis->observer_telemetry.motor_limited_current[i] =
            chassis->wheel_motor[i]->motor_controller.final_output;
    }
}

/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput()
{
    DJIMotorSetPIDRef(chassis->wheel_motor[0], vt_lf);
    DJIMotorSetPIDRef(chassis->wheel_motor[1], vt_rf);
    DJIMotorSetPIDRef(chassis->wheel_motor[2], vt_lb);
    DJIMotorSetPIDRef(chassis->wheel_motor[3], vt_rb);
    PowerControl();
}

/** @brief 编码器正解与 EKF；短临界区只复制传感器，矩阵运算在区外执行。 */
static void EstimateSpeed()
{
    const float dt = DWT_GetDeltaT(&observer_clock);
    SpeedObserverInput_s input = {0};
    ChassisObserverTelemetry_s *telemetry = &chassis->observer_telemetry;
    telemetry->sample_time = DWT_GetTimeline_s();
    telemetry->dt = dt;
    // 临界区只复制同一轮传感器快照，解算与融合在区外完成。
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i)
    {
        input.motor_speed[i] = chassis->wheel_motor[i]->measure.speed_aps;
        input.wheel_stamp[i] = chassis->wheel_motor[i]->feed_cnt;
        input.wheel_online[i] = DaemonIsOnline(chassis->wheel_motor[i]->daemon);
        telemetry->motor_speed[i] = input.motor_speed[i];
    }
    if (chassis->chassis_IMU != NULL && chassis->chassis_IMU->init)
    {
        input.imu_ready = 1u;
        input.imu_stamp = QEKF_INS.UpdateCount;
        input.gyro_z = chassis->chassis_IMU->Gyro[2];
        input.accel_x = chassis->chassis_IMU->MotionAccel_b[0];
        input.accel_y = chassis->chassis_IMU->MotionAccel_b[1];
        input.accel_lpf = chassis->chassis_IMU->AccelLPF;
        telemetry->imu_count = (uint32_t)(input.imu_stamp & 0x00ffffffu);
        telemetry->temperature = BMI088.Temperature;
        for (int i = 0; i < 3; ++i)
        {
            telemetry->gyro[i] = chassis->chassis_IMU->Gyro[i];
            telemetry->motion_accel[i] = chassis->chassis_IMU->MotionAccel_b[i];
        }
    }
    taskEXIT_CRITICAL();

    SpeedObserverUpdate(&chassis->speed_observer, &input, dt);
    telemetry->center_accel[0] = chassis->speed_observer.center_accel[0];
    telemetry->center_accel[1] = chassis->speed_observer.center_accel[1];
    telemetry->status = chassis->speed_observer.status;
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF) telemetry->status |= (1u << 7);
}
static float YawHoldCompensate()
{
    if (chassis == NULL || chassis_ctrl_cmd == NULL || chassis->chassis_IMU == NULL || !chassis->chassis_IMU->init)
    {
        yaw_hold_active = 0;
        return 0.0f;
    }

    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_HOLD)
    {
        yaw_hold_active = 0;
        return 0.0f;
    }

    if (!yaw_hold_active)
    {
        yaw_hold_active = 1;
        PIDClear(&chassis->yaw_hold_pid); // 同时重置 DWT 时间戳，避免模式切换后的长 dt。
    }

    chassis->yaw_hold_updated = 1u;
    return PIDCalculate(&chassis->yaw_hold_pid, chassis->chassis_IMU->YawTotalAngle, chassis_ctrl_cmd->yaw_hold_ref);
}

ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config)
{
    ChassisInstance *chassis_instance = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));

    chassis_param = chassis_init_config->chassis_param; // 在运行时赋值

    k0 = chassis_param.power_param.k0;
    k1 = chassis_param.power_param.k1;
    k2 = chassis_param.power_param.k2;
    k3 = chassis_param.power_param.k3;
    k4 = chassis_param.power_param.k4;
    k5 = chassis_param.power_param.k5;

    PIDInit(&follow_pid, &chassis_init_config->follow_pid);
    PIDInit(&chassis_instance->vx_hold_pid, &chassis_init_config->vx_hold_pid);
    PIDInit(&chassis_instance->vy_hold_pid, &chassis_init_config->vy_hold_pid);
    PIDInit(&chassis_instance->wz_hold_pid, &chassis_init_config->wz_hold_pid);
    chassis_instance->enable_speed_hold = chassis_init_config->enable_speed_hold;

    if (chassis_init_config->enable_yaw_hold)
    {
        PIDInit(&chassis_instance->yaw_hold_pid, &chassis_init_config->yaw_hold_pid);
    }

    SpeedObserver_Init_Config_s observer_config = {
        .filter = chassis_init_config->observer_config,
        .wheel_base = chassis_param.wheel_base * 0.001f,
        .track_width = chassis_param.track_width * 0.001f,
        .wheel_radius = chassis_param.wheel_radius * 0.001f,
        .reduction = chassis_param.wheel_reduction_ratio,
        .imu_offset = {chassis_init_config->observer_imu_offset[0], chassis_init_config->observer_imu_offset[1]},
        .sensor_timeout = chassis_init_config->observer_sensor_timeout,
        .use_imu = chassis_init_config->observer_imu_calibrated,
        .use_accel = chassis_init_config->observer_accel_enabled,
    };
    SpeedObserverInit(&chassis_instance->speed_observer, &observer_config);
    if (chassis_init_config->enable_imu_sampling || chassis_init_config->observer_imu_calibrated || chassis_init_config->enable_yaw_hold)
        chassis_instance->chassis_IMU = INS_Init(&chassis_init_config->imu_init_config);
    DWT_GetDeltaT(&observer_clock);
    for (int i = 0; i < 4; i++)
    {
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.outer_loop_type = SPEED_LOOP;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.close_loop_type = SPEED_LOOP;
        chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
    }

    chassis = chassis_instance;
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd; // 在运行时初始化指针
    chassis_instance->super_cap_mode = SAFETY_MODE;
    return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    chassis->yaw_hold_updated = 0u;
    chassis->speed_hold_updated = 0u;
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_HOLD)
        yaw_hold_active = 0; // 非 HOLD 时不会调用补偿函数，必须在这里清除进入状态。
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF)
    {
        // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        for (int i = 0; i < 4; i++)
            DJIMotorStop(chassis->wheel_motor[i]);
    }
    else
    {
        // 正常工作
        for (int i = 0; i < 4; i++)
            DJIMotorEnable(chassis->wheel_motor[i]);
    }
    chassis_wz = chassis_ctrl_cmd->wz; // 局部叠加角度环，避免修改并累加外部目标
    // 根据控制模式设定旋转速度
    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW: // offset_angle 为云台相对底盘的逆时针偏角，PID 输出同向 wz
        chassis_wz += PIDCalculate(&follow_pid, chassis_ctrl_cmd->offset_angle, 0);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        break;
    case CHASSIS_HOLD:
        chassis_wz += YawHoldCompensate();
        break;
    default:
        break;
    }
    // 根据控制量所在的坐标系决定是否需要映射:
    //  - 云台系(遥控器指令): 指令以云台指向为 x, 按 offset_angle 旋转到底盘坐标系
    //  - 底盘系(导航/上位机指令): 指令已经是底盘正前方为 x, 直接使用, 不随云台旋转
    if (chassis_ctrl_cmd->cmd_frame == CHASSIS_CMD_CHASSIS_FRAME)
    {
        chassis_vx = chassis_ctrl_cmd->vx;
        chassis_vy = chassis_ctrl_cmd->vy;
    }
    else
    {
        // 两坐标系均为前 x、左 y；offset_angle 为云台相对底盘逆时针偏角，单位度。
        // 云台系向量转换到底盘系: v_body = R(offset_angle) * v_gimbal。
        static float sin_theta, cos_theta;
        cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
        sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
        chassis_vx = chassis_ctrl_cmd->vx * cos_theta - chassis_ctrl_cmd->vy * sin_theta;
        chassis_vy = chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;
    }

    // 更新轮速正解与速度观测器，再在底盘机体系完成三轴速度外环。
    EstimateSpeed();
    SpeedHoldControl();
    if (chassis->speed_observer.valid) chassis->observer_telemetry.status |= (1u << 12);
    if (chassis->speed_hold_active) chassis->observer_telemetry.status |= (1u << 13);
    if (chassis->yaw_hold_updated) chassis->observer_telemetry.status |= (1u << 14);

    // 逆运动学解算，计算四轮速度参考
    OmniCalculate();

    // SuperCapSendMessage(chassis->super_cap, (int16_t)300, referee_data->PowerHeatData.buffer_energy,
    //                       referee_data->GameRobotState.power_management_chassis_output);

    // 功率控制与输出限幅
    LimitChassisOutput();
}
