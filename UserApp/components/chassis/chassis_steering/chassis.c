//
// 由 yang6 创建于 2025/11/17.
//
/**
 * @file    chassis.c
 * @author  YZC
 * @author  注释与修改：SRM-Control 2026
 * @date    2026/1/1
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   舵轮底盘模块
 */
#include "chassis.h"

#include <math.h>

#include "arm_math.h"
#include "rm_referee.h"
#include "user_lib.h"

#define SQRT2_OVER_2 0.7071067811865476f
#define STEERING_CMD_DEADBAND 0.5f
#define STEERING_FLIP_ENTER_ANGLE 100.0f
#define STEERING_FLIP_EXIT_ANGLE 80.0f
#define STEERING_NO_ATTENUATION_ANGLE 8.0f
#define FOLLOW_START_DEADBAND 5.0f
#define FOLLOW_STOP_DEADBAND 2.0f
#define WHEEL_CURRENT_OUT_PER_AMP (16384.0f / 20.0f)
#define RUDDER_CURRENT_OUT_PER_AMP (16384.0f / 3.0f)
#define MOTOR_OUTPUT_LIMIT 16000.0f
#define SUPERCAP_SAFE_POWER 35U
#define SUPERCAP_ACTIVE_POWER 150U
#define SUPERCAP_FULL_VOLTAGE 18.0f
#define SUPERCAP_CHARGE_VOLTAGE 14.0f
#define SUPERCAP_FORCE_CHARGE_VOLTAGE 13.5f
#define SUPERCAP_SAFETY_VOLTAGE 13.0f

/**
 * @brief 单个舵轮模块的目标地面速度向量。
 *
 * x/y 为底盘坐标系下的分量，后续会转换成舵角和轮速。
 */
typedef struct
{
    float x;
    float y;
} WheelVector_s;

static referee_info_t *referee_data;
static ChassisInstance *chassis;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Power_Param_3508_s wheel_power_param;
static PIDInstance follow_pid;

static float chassis_vx;
static float chassis_vy;
static float wheel_speed_ref[WHEEL_COUNT] = {0.0f};
static float rudder_angle_ref[WHEEL_COUNT] = {0.0f};
static int8_t wheel_direction[WHEEL_COUNT] = {1, 1, 1, 1};
static float omega_x[WHEEL_COUNT] = {0.0f};
static float omega_y[WHEEL_COUNT] = {0.0f};
static uint8_t follow_enabled;

/* 调试/观测变量保持为全局，方便在 watch 窗口里查看。 */
float FK_Vx;
float FK_Vy;
float FK_Wz;
float initial_rudder_total_power = 0.0f;
float initial_rudder_give_power[WHEEL_COUNT] = {0.0f};

/* 小型数学 helper 用来提高功率模型公式的可读性。 */
static float Square(float value)
{
    return value * value;
}

/**
 * @brief 将角度归一化到 [-180, 180]，单位：度。
 */
static float NormalizeAngle180(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    else if (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 将角度归一化到 [0, 360)，单位：度。
 */
static float NormalizeAngle360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 带下限保护地扣除功率降额，避免无符号数下溢。
 */
static uint16_t PowerLimitMinus(uint16_t limit, uint16_t margin)
{
    return limit > margin ? (uint16_t)(limit - margin) : 0U;
}

/**
 * @brief 将裁判系统功率上限裁剪到超电协议字段可表示范围。
 */
static int16_t ClampPowerForSuperCap(uint16_t power_limit)
{
    return (int16_t)(power_limit > UINT8_MAX ? UINT8_MAX : power_limit);
}

/* DJI 电流指令值 -> 功率拟合模型使用的物理电流估计值。 */
static float CurrentOutputTo3508Amp(float current_output)
{
    return current_output / WHEEL_CURRENT_OUT_PER_AMP;
}

/* DJI 电流指令值 -> GM6020 功率预算使用的电流估计值。 */
static float CurrentOutputTo6020Amp(float current_output)
{
    return current_output / RUDDER_CURRENT_OUT_PER_AMP;
}

/* 电机速度反馈单位是度/秒，功率模型使用弧度/秒。 */
static float WheelSpeedToRadPerSec(float speed_aps)
{
    return speed_aps * DEGREE_2_RAD;
}

/**
 * @brief 将目标舵角转换为最近的等效控制指令。
 *
 * 如果直接打舵需要转过 90 度以上，则反向驱动轮电机，并给舵电机
 * 下发等效的反向舵角，让模块始终走短弧。
 */
static float AngleToOptimalAngle(float target_angle, float current_angle, int8_t *direction)
{
    float diff = NormalizeAngle180(target_angle - current_angle);

    if (*direction > 0)
    {
        if (diff > STEERING_FLIP_ENTER_ANGLE)
        {
            *direction = -1;
            return current_angle + diff - 180.0f;
        }
        if (diff < -STEERING_FLIP_ENTER_ANGLE)
        {
            *direction = -1;
            return current_angle + diff + 180.0f;
        }

        return current_angle + diff;
    }

    // 已经反向驱动时，必须等直接角度误差回到较小范围再恢复正向。
    if (fabsf(diff) < STEERING_FLIP_EXIT_ANGLE)
    {
        *direction = 1;
        return current_angle + diff;
    }

    if (diff >= 0.0f)
    {
        return current_angle + diff - 180.0f;
    }

    return current_angle + diff + 180.0f;
}

/**
 * @brief 以底盘整体为单位使能或停止所有轮电机和舵电机。
 */
static void EnableChassisMotors(uint8_t enable)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (enable)
        {
            DJIMotorEnable(chassis->rudder_motor[i]);
            DJIMotorEnable(chassis->wheel_motor[i]);
        }
        else
        {
            DJIMotorStop(chassis->rudder_motor[i]);
            DJIMotorStop(chassis->wheel_motor[i]);
        }
    }
}

/**
 * @brief 将轮电机强制配置为速度环，同时保留 PID/CAN 配置。
 */
static void ConfigureWheelMotor(Motor_Init_Config_s *config)
{
    config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    config->controller_setting_init_config.close_loop_type = SPEED_LOOP;
}

/**
 * @brief 将舵电机强制配置为角度-速度串级环，同时保留 PID/CAN 配置。
 */
static void ConfigureRudderMotor(Motor_Init_Config_s *config)
{
    config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config->controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
}

/**
 * @brief 根据舵角和轮速估计底盘速度。
 *
 * FK_* 仅作为观测/调试输出，本模块不会把它反馈到底盘控制链路中。
 */
static void ForwardKinematicCal(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float rudder_angle = chassis->rudder_motor[i]->measure.angle_single_round -
                             (float)chassis->rudder_offset[i] * ECD_ANGLE_COEF_DJI;
        rudder_angle = NormalizeAngle360(rudder_angle) * DEGREE_2_RAD;

        omega_x[i] = chassis->wheel_motor[i]->measure.speed_aps * arm_sin_f32(rudder_angle);
        omega_y[i] = chassis->wheel_motor[i]->measure.speed_aps * arm_cos_f32(rudder_angle);
    }

    FK_Vx = (omega_x[LF] + omega_x[RF] + omega_x[LB] + omega_x[RB]) / 4.0f;
    FK_Vy = (omega_y[LF] + omega_y[RF] + omega_y[LB] + omega_y[RB]) / 4.0f;
    FK_Wz = (+omega_x[LF] - omega_y[LF] + omega_x[RF] + omega_y[RF] - omega_x[LB] + omega_y[LB] -
             omega_x[RB] - omega_y[RB]) *
            SQRT2_OVER_2 / 4.0f;
}

/**
 * @brief 当任意轮速超过上限时，按比例整体缩放四个轮速参考值。
 */
static void LimitWheelSpeed(void)
{
    float max_speed = 0.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float speed_abs = fabsf(wheel_speed_ref[i]);
        if (max_speed < speed_abs)
        {
            max_speed = speed_abs;
        }
    }

    if (max_speed > MAX_WHEEL_SPEED)
    {
        float scale = MAX_WHEEL_SPEED / max_speed;
        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_speed_ref[i] *= scale;
        }
    }
}

static void LimitTranslationForSpin(void)
{
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_ROTATE)
    {
        return;
    }

    const float wz = chassis_ctrl_cmd->wz;
    const float translation_sq = Square(chassis_vx) + Square(chassis_vy);
    const float limit_sq = Square(MAX_WHEEL_SPEED);
    float translation_scale = 1.0f;

    if (translation_sq < 1.0e-6f)
    {
        return;
    }

    const WheelVector_s rotation[WHEEL_COUNT] = {
        [LF] = {.x = +wz * SQRT2_OVER_2, .y = -wz * SQRT2_OVER_2},
        [LB] = {.x = +wz * SQRT2_OVER_2, .y = +wz * SQRT2_OVER_2},
        [RB] = {.x = -wz * SQRT2_OVER_2, .y = +wz * SQRT2_OVER_2},
        [RF] = {.x = -wz * SQRT2_OVER_2, .y = -wz * SQRT2_OVER_2},
    };

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        // |alpha * translation + rotation_i|^2 <= MAX_WHEEL_SPEED^2
        const float b = 2.0f * (chassis_vx * rotation[i].x +
                                chassis_vy * rotation[i].y);
        const float c = Square(rotation[i].x) + Square(rotation[i].y) - limit_sq;

        // 纯自转已经超过轮速能力，平移无法解决。
        if (c > 0.0f)
        {
            chassis_vx = 0.0f;
            chassis_vy = 0.0f;
            return;
        }

        const float discriminant = b * b - 4.0f * translation_sq * c;
        const float positive_root =
            (-b + sqrtf(fmaxf(discriminant, 0.0f))) / (2.0f * translation_sq);

        translation_scale = fminf(translation_scale, positive_root);
    }

    translation_scale = fmaxf(0.0f, translation_scale);
    chassis_vx *= translation_scale;
    chassis_vy *= translation_scale;
}
/**
 * @brief 在下发角度指令前补偿 GM6020 机械零位偏移。
 */
static void ApplyRudderOffset(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        rudder_angle_ref[i] -= (float)chassis->rudder_offset[i] * ECD_ANGLE_COEF_DJI;
    }
}

/**
 * @brief 根据舵角跟随误差统一门控四个轮子的驱动速度。
 *
 * 8 度以内不衰减；误差从 8 度增加到 90 度时，公共门控系数从 1 平滑降到 0。
 * 使用公共系数可保持四轮扭矩比例，不额外改变底盘偏航力矩。
 */
static void ApplyRudderPriority(void)
{
    float common_scale = 1.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float angle_error = fabsf(NormalizeAngle180(
            rudder_angle_ref[i] - chassis->rudder_motor[i]->measure.total_angle));
        float wheel_scale = 1.0f;

        if (angle_error > STEERING_NO_ATTENUATION_ANGLE)
        {
            if (angle_error >= 90.0f)
            {
                wheel_scale = 0.0f;
            }
            else
            {
                float normalized_error =
                    (angle_error - STEERING_NO_ATTENUATION_ANGLE) /
                    (90.0f - STEERING_NO_ATTENUATION_ANGLE);
                wheel_scale = arm_cos_f32(normalized_error * PI / 2.0f);
            }
        }

        if (wheel_scale < common_scale)
        {
            common_scale = wheel_scale;
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] *= common_scale;
    }
}

/**
 * @brief GM6020 舵电机拟合功率估计。
 *
 * 输入电流单位为 A，角速度单位为 rad/s；只有正功率会计入底盘功率预算。
 */
static float RudderPowerForecast(float current_amp, float omega_rad_per_sec)
{
    const float k0 = 0.8130f;
    const float k1 = -0.0005f;
    const float k2 = 6.0021f;
    const float a = 1.3715f;

    return k0 * current_amp * omega_rad_per_sec + k1 * Square(omega_rad_per_sec) + k2 * Square(current_amp) + a;
}

/**
 * @brief 3508 轮电机拟合功率估计。
 */
static float WheelPowerForecast(float current_output, float speed_aps)
{
    float current_amp = CurrentOutputTo3508Amp(current_output);
    float omega = WheelSpeedToRadPerSec(speed_aps);

    return wheel_power_param.k0 + wheel_power_param.k1 * current_amp + wheel_power_param.k2 * omega +
           wheel_power_param.k3 * current_amp * omega + wheel_power_param.k4 * Square(current_amp) +
           wheel_power_param.k5 * Square(omega);
}

/**
 * @brief 根据四轮功率模型直接反解公共电流缩放系数。
 *
 * 令每个轮子的电流为 I_i * s，则四轮总功率可写成：
 *
 *     A * s^2 + B * s + C = available_power
 *
 * 公共缩放系数 s 作用于所有轮子，因此保持四轮电流/扭矩比例不变。
 */
static float SolveWheelCurrentScaleByPower(const float speed_aps[WHEEL_COUNT],
                                           const float current_output[WHEEL_COUNT],
                                           float available_power)
{
    float coefficient_a = 0.0f;
    float coefficient_b = 0.0f;
    float coefficient_c = 0.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        const float current_amp = CurrentOutputTo3508Amp(current_output[i]);
        const float omega = WheelSpeedToRadPerSec(speed_aps[i]);
        const float reference_power = WheelPowerForecast(current_output[i], speed_aps[i]);

        // 与 PowerControl 的总功率统计保持一致，不让负功率项抵消正功率项。
        if (reference_power <= 0.0f)
        {
            continue;
        }

        coefficient_a += wheel_power_param.k4 * Square(current_amp);
        coefficient_b += wheel_power_param.k1 * current_amp +
                         wheel_power_param.k3 * current_amp * omega;
        coefficient_c += wheel_power_param.k0 + wheel_power_param.k2 * omega +
                         wheel_power_param.k5 * Square(omega);
    }

    // 即使电流缩放到零，速度相关功率仍然超出预算，无法靠削减电流解决。
    if (coefficient_c >= available_power)
    {
        return 0.0f;
    }

    const float equation_c = coefficient_c - available_power;
    float current_scale;

    if (fabsf(coefficient_a) < 1.0e-6f)
    {
        // 退化为一次方程。
        if (coefficient_b <= 1.0e-6f)
        {
            return 1.0f;
        }
        current_scale = -equation_c / coefficient_b;
    }
    else
    {
        const float discriminant = coefficient_b * coefficient_b -
                                    4.0f * coefficient_a * equation_c;

        if (discriminant <= 0.0f)
        {
            return 0.0f;
        }

        // 取较大的根，得到不超功率时最大的可用电流比例。
        current_scale = (-coefficient_b + sqrtf(discriminant)) /
                         (2.0f * coefficient_a);
    }

    return fmaxf(0.0f, fminf(1.0f, current_scale));
}

/**
 * @brief 预测电机功率并限制轮电机电流输出。
 *
 * 舵电机功率优先计入预算；当预测轮电机功率超过 chassis_ctrl_cmd->max_power
 * 时，搜索一个公共电流缩放系数并作用于四个轮电机。
 */
static void PowerControl(void)
{
    float wheel_power[WHEEL_COUNT] = {0.0f};
    float wheel_current[WHEEL_COUNT] = {0.0f};
    float wheel_speed[WHEEL_COUNT] = {0.0f};
    float total_wheel_power = 0.0f;

    initial_rudder_total_power = 0.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        initial_rudder_give_power[i] =
            RudderPowerForecast(CurrentOutputTo6020Amp((float)chassis->rudder_motor[i]->measure.real_current),
                                WheelSpeedToRadPerSec(chassis->rudder_motor[i]->measure.speed_aps));

        if (initial_rudder_give_power[i] > 0.0f)
        {
            initial_rudder_total_power += initial_rudder_give_power[i];
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed[i] = chassis->wheel_motor[i]->measure.speed_aps;
        wheel_current[i] = chassis->wheel_motor[i]->motor_controller.final_output;
        wheel_power[i] = WheelPowerForecast(wheel_current[i], wheel_speed[i]);

        if (wheel_power[i] > 0.0f)
        {
            total_wheel_power += wheel_power[i];
        }
    }

    float available_power = (float)chassis_ctrl_cmd->max_power;//新规则下不再需要控制舵电机功率 - initial_rudder_total_power;
    if (available_power < 0.0f)
    {
        available_power = 0.0f;
    }

    if (total_wheel_power > available_power && total_wheel_power > 0.0f)
    {
        float current_scale = SolveWheelCurrentScaleByPower(wheel_speed, wheel_current, available_power);

        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_current[i] *= current_scale;
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)wheel_current[i];
    }
}

/**
 * @brief 写入轮电机/舵电机 PID 参考值，然后执行功率限制。
 */
static void LimitChassisOutput(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        DJIMotorSetPIDRef(chassis->wheel_motor[i], wheel_speed_ref[i]);
        DJIMotorSetPIDRef(chassis->rudder_motor[i], rudder_angle_ref[i]);
    }

    PowerControl();
}

/**
 * @brief 计算四个舵轮模块的逆运动学。
 *
 * 每个模块先由底盘 vx/vy/wz 得到目标速度向量，再转换为舵角和轮速；
 * 必要时通过轮向反转把舵角约束到短弧路径。
 */
static void SteeringCalculate(void)
{
    LimitTranslationForSpin();
    WheelVector_s wheel_vector[WHEEL_COUNT] = {
        [LF] = {.x = chassis_vx + chassis_ctrl_cmd->wz * SQRT2_OVER_2,
                .y = chassis_vy - chassis_ctrl_cmd->wz * SQRT2_OVER_2},
        [LB] = {.x = chassis_vx + chassis_ctrl_cmd->wz * SQRT2_OVER_2,
                .y = chassis_vy + chassis_ctrl_cmd->wz * SQRT2_OVER_2},
        [RB] = {.x = chassis_vx - chassis_ctrl_cmd->wz * SQRT2_OVER_2,
                .y = chassis_vy + chassis_ctrl_cmd->wz * SQRT2_OVER_2},
        [RF] = {.x = chassis_vx - chassis_ctrl_cmd->wz * SQRT2_OVER_2,
                .y = chassis_vy - chassis_ctrl_cmd->wz * SQRT2_OVER_2},
    };

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] = sqrtf(Square(wheel_vector[i].x) + Square(wheel_vector[i].y));
    }

    if (fabsf(chassis_ctrl_cmd->vx) > STEERING_CMD_DEADBAND ||
        fabsf(chassis_ctrl_cmd->vy) > STEERING_CMD_DEADBAND ||
        fabsf(chassis_ctrl_cmd->wz) > STEERING_CMD_DEADBAND)
    {
        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            rudder_angle_ref[i] = RAD_2_DEGREE * atan2f(wheel_vector[i].y, wheel_vector[i].x);
        }

        ApplyRudderOffset();

        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            float current_angle = chassis->rudder_motor[i]->measure.total_angle;
            rudder_angle_ref[i] =
                AngleToOptimalAngle(rudder_angle_ref[i], current_angle, &wheel_direction[i]);
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] *= (float)wheel_direction[i];
    }

    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_ROTATE)
    {
        LimitWheelSpeed();
    }
}

/**
 * @brief 超级电容状态机和底盘功率预算选择。
 */
static void UpdateSuperCapMode(void)
{
    uint16_t referee_power_limit = referee_data->GameRobotState.chassis_power_limit;
    float cap_voltage = chassis->super_cap->cap_msg.cap_v;

    switch (chassis->super_cap_mode)
    {
    case SAFETY_MODE:
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_ctrl_cmd->max_power = SUPERCAP_SAFE_POWER;
        break;

    case FORCED_CHARGING_MODE:
        if (cap_voltage < SUPERCAP_SAFETY_VOLTAGE)
        {
            chassis->super_cap_mode = SAFETY_MODE;
        }
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_ctrl_cmd->max_power = PowerLimitMinus(referee_power_limit, 30U);
        break;

    case CHARGING_MODE:
        if (cap_voltage < SUPERCAP_FORCE_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = FORCED_CHARGING_MODE;
        }
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_ctrl_cmd->max_power = PowerLimitMinus(referee_power_limit, 20U);
        break;

    case PASSIVE_MODE:
        if (chassis_ctrl_cmd->SuperCapBoost & 1U)
        {
            chassis->super_cap_mode = ACTIVE_MODE;
        }
        if (cap_voltage < SUPERCAP_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = CHARGING_MODE;
        }
        chassis_ctrl_cmd->max_power = (uint16_t)(0.9f * (float)referee_power_limit);
        break;

    case ACTIVE_MODE:
        if (cap_voltage < SUPERCAP_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = CHARGING_MODE;
        }
        if (!(chassis_ctrl_cmd->SuperCapBoost & 1U))
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_ctrl_cmd->max_power = SUPERCAP_ACTIVE_POWER;
        break;

    default:
        chassis->super_cap_mode = SAFETY_MODE;
        chassis_ctrl_cmd->max_power = SUPERCAP_SAFE_POWER;
        break;
    }
}

/**
 * @brief 围绕目标角度加入带滞回的 yaw 跟随反馈。
 */
static void ApplyFollowControl(float target_angle)
{
    float angle_error = NormalizeAngle180(chassis_ctrl_cmd->offset_angle - target_angle);

    if (fabsf(angle_error) > FOLLOW_START_DEADBAND)
    {
        follow_enabled = 1U;
    }
    if (fabsf(angle_error) < FOLLOW_STOP_DEADBAND)
    {
        follow_enabled = 0U;
    }

    if (follow_enabled)
    {
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, angle_error, 0.0f);
    }
}

/**
 * @brief 在运动学解算前处理高层底盘模式逻辑。
 */
static void ApplyChassisMode(void)
{
    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW:
        ApplyFollowControl(0.0f);
        break;

    case CHASSIS_FOLLOW_DIAGONAL:
        ApplyFollowControl(45.0f);
        break;

    case CHASSIS_ROTATE:
        chassis_ctrl_cmd->offset_angle -= 0.001f * chassis_ctrl_cmd->wz;
        break;

    default:
        break;
    }
}

/**
 * @brief 将云台坐标系下的平移指令旋转到底盘坐标系。
 */
static void TransformCommandToChassisFrame(void)
{
    float cos_theta = arm_cos_f32(-chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(-chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);

    chassis_vx = -chassis_ctrl_cmd->vx * cos_theta + chassis_ctrl_cmd->vy * sin_theta;
    chassis_vy = -chassis_ctrl_cmd->vx * sin_theta - chassis_ctrl_cmd->vy * cos_theta;
}

/**
 * @brief 将裁判系统功率状态转发给超电模块。
 */
static void SendSuperCapCommand(void)
{
    SuperCapSendMessage(chassis->super_cap, ClampPowerForSuperCap(referee_data->GameRobotState.chassis_power_limit),
                        referee_data->PowerHeatData.buffer_energy,
                        referee_data->GameRobotState.power_management_chassis_output);
}

/**
 * @brief 分配并连接舵轮底盘运行时实例。
 *
 * 该函数保存共享的裁判系统指针，初始化超电模块和四组轮/舵电机，
 * 并设置 ChassisTask() 使用的模块级指针。初始化配置仍由调用方持有；
 * 返回的实例由 zmalloc 分配，并作为本模块的单例保存。
 *
 * @return 初始化成功时返回 ChassisInstance 指针；内存分配失败时返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config)
{
    ChassisInstance *chassis_instance = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));
    if (chassis_instance == NULL)
    {
        return NULL;
    }

    wheel_power_param = chassis_init_config->chassis_param.power_param;
    referee_data = GetReferee();
    PIDInit(&follow_pid, &chassis_init_config->follow_pid);

    chassis_instance->super_cap = SuperCapInit(&chassis_init_config->super_cap_config);
    chassis_instance->super_cap_mode = SAFETY_MODE;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        ConfigureWheelMotor(&chassis_init_config->wheel_motor_config[i]);
        chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        ConfigureRudderMotor(&chassis_init_config->rudder_motor_config[i]);
        chassis_instance->rudder_motor[i] = DJIMotorInit(&chassis_init_config->rudder_motor_config[i]);
        chassis_instance->rudder_offset[i] = chassis_init_config->chassis_param.rudder_motor_offset[i];
    }

    chassis = chassis_instance;
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;
    return chassis_instance;
}

/**
 * @brief 舵轮底盘周期控制任务。
 *
 * 该任务需要在 ChassisInit() 之后运行。每个周期会更新超电功率模式、
 * 应用底盘模式、把云台系指令转换到底盘系、执行舵轮逆运动学解算，
 * 最后写入电机参考值并进行功率限制。
 */
void ChassisTask(void)
{
    if (chassis == NULL || chassis_ctrl_cmd == NULL)
    {
        return;
    }

    UpdateSuperCapMode();
    EnableChassisMotors(chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF);
    ApplyChassisMode();
    TransformCommandToChassisFrame();

    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF)
    {
        SteeringCalculate();
        ForwardKinematicCal();
        ApplyRudderPriority();
    }

    SendSuperCapCommand();
    LimitChassisOutput();
}
