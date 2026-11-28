#include "chassis.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "task.h"
#include "user_lib.h"

static const float kRpmToRadPerSec = 0.1047197551f;
static const float kC620MaxCurrentA = 20.0f;
static const float kC620CurrentCountsPerAmp = 16384.0f / 20.0f;
static ChassisInstance instance;

#if defined(STM32H723xx)
static bool PrepareCanFilters(const Chassis_Init_Config_s *config)
{
    FDCAN_HandleTypeDef *const buses[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    unsigned needed[DEVICE_CAN_CNT] = {0};
    for (unsigned bus = 0; bus < DEVICE_CAN_CNT; ++bus)
    {
        FDCAN_HandleTypeDef *can = buses[bus];
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
            if ((config->joint_mask & (1u << i)) && config->hip[i].motor_config.can_init_config.can_handle == can)
                ++needed[bus];
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
            if ((config->wheel_mask & (1u << i)) && config->wheel[i].motor_config.can_init_config.can_handle == can)
                ++needed[bus];
        if (needed[bus] <= can->Init.StdFiltersNbr)
            continue;

        // The first CANRegister starts all FDCAN buses. Re-init only CAN1/2 and only before that call.
        if (can == &hfdcan3 || HAL_FDCAN_GetState(can) != HAL_FDCAN_STATE_READY)
            return false;
        const uint32_t start = can->msgRam.StandardFilterSA;
        const uint32_t end = can->msgRam.EndAddress + 4u * (needed[bus] - can->Init.StdFiltersNbr);
        // Keep the same SRAMCAN end bound as the H723 HAL. Each standard filter occupies one word.
        if (end > SRAMCAN_BASE + 0x2800u - 4u)
            return false;
        for (unsigned other = 0; other < DEVICE_CAN_CNT; ++other)
        {
            const FDCAN_HandleTypeDef *neighbor = buses[other];
            if (other != bus && start < neighbor->msgRam.EndAddress && end > neighbor->msgRam.StandardFilterSA)
                return false;
        }
    }
    for (unsigned bus = 0; bus < DEVICE_CAN_CNT; ++bus)
    {
        FDCAN_HandleTypeDef *can = buses[bus];
        if (needed[bus] > can->Init.StdFiltersNbr)
        {
            can->Init.StdFiltersNbr = needed[bus];
            if (HAL_FDCAN_Init(can) != HAL_OK)
                return false;
        }
    }
    return true;
}
#endif

ChassisInstance *ChassisInit(Chassis_Init_Config_s *config)
{
#if defined(STM32H723xx)
    if (!PrepareCanFilters(config))
        return NULL;
#endif

    instance.config = config;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &instance.legs[side];
        const unsigned front = 2u * side + CHASSIS_FRONT;
        const unsigned rear = 2u * side + CHASSIS_REAR;
        if (config->joint_mask & (1u << front))
        {
            leg->front_motor = DMMotorInit(&config->hip[front].motor_config);
            DMMotorStop(leg->front_motor);
        }
        if (config->joint_mask & (1u << rear))
        {
            leg->rear_motor = DMMotorInit(&config->hip[rear].motor_config);
            DMMotorStop(leg->rear_motor);
        }
        if (config->wheel_mask & (1u << side))
        {
            leg->wheel_motor = DJIMotorInit(&config->wheel[side].motor_config);
            DJIMotorStop(leg->wheel_motor);
        }
    }
    return &instance;
}

static bool FeedbackFresh(uint32_t counter, float timeout_s, uint32_t *previous, bool *seen, bool *updated,
                          uint64_t *feedback_time_us)
{
    *updated = counter != *previous;
    uint32_t sample = counter;
    taskENTER_CRITICAL();
    const float age_s = DWT_GetDeltaT(&sample);
    const uint64_t now_us = DWT_GetTimeline_us();
    taskEXIT_CRITICAL();
    if (*updated)
    {
        *previous = counter;
        *seen = true;
        const uint64_t age_us = (uint64_t)(age_s * 1000000.0f);
        *feedback_time_us = age_us <= now_us ? now_us - age_us : 0u;
    }

    // Expired data stays expired until a new frame, including across a DWT counter wrap.
    *seen = *seen && age_s <= timeout_s;
    return *seen;
}

static void UpdateJoint(const DMMotorInstance *motor, const HipMotor_Init_Config_s *config,
                        ChassisJointFeedback *feedback, float timeout_s)
{
    DM_Motor_Measure_s measure;
    uint32_t counter;
    uint8_t dlc;
    uint16_t torque_raw;
    const CANInstance *can = motor->motor_can_instance;
    taskENTER_CRITICAL();
    measure = motor->measure;
    counter = motor->feed_cnt;
    dlc = can->rx_len;
    torque_raw = ((uint16_t)(can->rx_buff[4] & 0x0fu) << 8) | can->rx_buff[5];
    taskEXIT_CRITICAL();
    const bool fresh = FeedbackFresh(counter, timeout_s, &feedback->feed_counter, &feedback->seen, &feedback->updated,
                                     &feedback->feedback_time_us);
    feedback->q_rad = (measure.total_angle - config->zero_rad) * config->direction;
    feedback->qd_rad_s = measure.velocity * config->direction;
    feedback->tau_nm = measure.torque * config->direction;
    feedback->torque_raw = torque_raw;
    feedback->temperature_c = fmaxf(measure.T_Mos, measure.T_Rotor);
    // The CAN driver already dispatches by bus and Master ID.
    feedback->valid = fresh && dlc == 8u && measure.state == 1u;
}

static void UpdateWheel(const DJIMotorInstance *motor, const WheelMotor_Init_Config_s *config,
                        ChassisWheelFeedback *feedback, float timeout_s)
{
    uint32_t counter;
    uint8_t dlc;
    uint8_t temperature;
    int16_t rotor_rpm;
    int16_t current_counts;
    const CANInstance *can = motor->motor_can_instance;
    taskENTER_CRITICAL();
    counter = motor->feed_cnt;
    dlc = can->rx_len;
    rotor_rpm = (int16_t)((uint16_t)can->rx_buff[2] << 8 | can->rx_buff[3]);
    current_counts = (int16_t)((uint16_t)can->rx_buff[4] << 8 | can->rx_buff[5]);
    temperature = can->rx_buff[6];
    taskEXIT_CRITICAL();
    const bool fresh = FeedbackFresh(counter, timeout_s, &feedback->feed_counter, &feedback->seen, &feedback->updated,
                                     &feedback->feedback_time_us);

    // The driver smooths both fields. Use the original frame for identification.
    feedback->speed_rad_s = (float)rotor_rpm * kRpmToRadPerSec * config->direction / config->reduction;
    // The C620 manual specifies A/count for commands only.
    feedback->current_raw = (float)current_counts * config->direction;
    feedback->temperature_c = temperature;
    feedback->valid = fresh && dlc == 8u;
}

static void UpdateGeometry(ChassisLeg *leg, const ChassisGeometryConfig *geometry, float center_delta)
{
    leg->geometry_valid = false;
    if (!leg->joint[CHASSIS_FRONT].valid || !leg->joint[CHASSIS_REAR].valid)
        return;

    // HKU concentric ACE model: virtual active rods are scaled by CE/HJ.
    const float front = leg->joint[CHASSIS_FRONT].q_rad;
    const float rear = leg->joint[CHASSIS_REAR].q_rad;
    const float scale = geometry->virtual_second_m / geometry->second_m;
    const float active = scale * geometry->first_m;
    leg->hip_delta_rad = center_delta + remainderf(front - rear - center_delta, 2.0f * PI);
    const float half_diff = 0.5f * leg->hip_delta_rad;
    const float normal = active * sinf(half_diff);
    const float discriminant = geometry->virtual_second_m * geometry->virtual_second_m - normal * normal;
    const float epsilon = geometry->singular_epsilon_m;
    if (!isfinite(discriminant) || discriminant <= epsilon * epsilon)
        return;

    const float root = sqrtf(discriminant);
    const float axial = active * cosf(half_diff);
    const float radius = axial + root > epsilon ? axial + root : axial - root;
    if (radius <= epsilon)
        return;

    const float length = radius / scale;
    const float mean = remainderf(rear, 2.0f * PI) + half_diff;
    const float angle = atan2f(-cosf(mean), sinf(mean));
    if (isfinite(length) && isfinite(angle))
    {
        leg->length_m = length;
        leg->angle_rad = angle;
        leg->geometry_valid = true;
    }
}

void ChassisFeedbackUpdate(ChassisInstance *chassis)
{
    const Chassis_Init_Config_s *config = chassis->config;
    const float timeout_s = (float)config->feedback_timeout_ms * 0.001f;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &chassis->legs[side];
        if (leg->front_motor)
            UpdateJoint(leg->front_motor, &config->hip[2u * side + CHASSIS_FRONT], &leg->joint[CHASSIS_FRONT],
                        timeout_s);
        if (leg->rear_motor)
            UpdateJoint(leg->rear_motor, &config->hip[2u * side + CHASSIS_REAR], &leg->joint[CHASSIS_REAR], timeout_s);
        const float center_delta =
            config->hip[2u * side + CHASSIS_FRONT].center_rad - config->hip[2u * side + CHASSIS_REAR].center_rad;
        UpdateGeometry(leg, &config->geometry, center_delta);
        if (leg->wheel_motor)
            UpdateWheel(leg->wheel_motor, &config->wheel[side], &leg->wheel_feedback, timeout_s);
    }
}

void ChassisTask(ChassisInstance *chassis, const ChassisRequest *request)
{
    const Chassis_Init_Config_s *config = chassis->config;
    bool enabled = request->enabled;
    bool clipped = false;

    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &chassis->legs[side];
        for (unsigned joint = 0; joint < 2u; ++joint)
        {
            ChassisJointFeedback *fb = &leg->joint[joint];
            fb->command_torque_preclip_nm = fb->command_torque_nm = 0.0f;
            const unsigned i = 2u * side + joint;
            if (!request->enabled || !(config->joint_mask & (1u << i)))
                continue;

            const HipMotor_Init_Config_s *hip = &config->hip[i];
            const float error = remainderf(request->hip_position_rad[i] - fb->q_rad, 2.0f * PI);
            float output = request->hip_torque_nm[i];
            if (!(request->hip_torque_mask & (1u << i)))
                output += hip->kp_nm_per_rad * error - hip->kd_nm_per_rad_s * fb->qd_rad_s;
            if (!fb->valid || !isfinite(output))
            {
                enabled = false;
                continue;
            }
            float limit = fminf(hip->max_torque_nm, DM_T_MAX_J8009P);
            if (request->hip_torque_limit_nm > 0.0f)
                limit = fminf(limit, request->hip_torque_limit_nm);
            fb->command_torque_preclip_nm = output;
            fb->command_torque_nm = float_constrain(output, -limit, limit);
            clipped |= output != fb->command_torque_nm;
        }

        ChassisWheelFeedback *fb = &leg->wheel_feedback;
        fb->command_current_preclip_a = fb->command_current_a = 0.0f;
        if (!request->enabled || !leg->wheel_motor)
            continue;
        const WheelMotor_Init_Config_s *wheel = &config->wheel[side];
        const float ref = request->wheel_reference[side];
        const float output = request->wheel_current_mode ? ref : wheel->kp_a_per_rad_s * (ref - fb->speed_rad_s);
        if (!fb->valid || fabsf(fb->speed_rad_s) > wheel->max_speed_rad_s ||
            (!request->wheel_current_mode && fabsf(ref) > wheel->max_speed_rad_s) || !isfinite(output))
        {
            enabled = false;
            continue;
        }
        const float limit = fminf(wheel->max_current_a, kC620MaxCurrentA);
        fb->command_current_preclip_a = output;
        fb->command_current_a = float_constrain(output, -limit, limit);
        clipped |= output != fb->command_current_a;
    }

    chassis->enabled = enabled;
    chassis->clipped = enabled && clipped;
    taskENTER_CRITICAL();
    chassis->command_time_us = DWT_GetTimeline_us();
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &chassis->legs[side];
        DMMotorInstance *motors[] = {leg->front_motor, leg->rear_motor};
        for (unsigned joint = 0; joint < 2u; ++joint)
        {
            if (!motors[joint])
                continue;
            ChassisJointFeedback *fb = &leg->joint[joint];
            if (!enabled)
                fb->command_torque_nm = 0.0f;
            DMMotorSetRef(motors[joint], fb->command_torque_nm * config->hip[2u * side + joint].direction);
            if (enabled)
                DMMotorEnable(motors[joint]);
            else
                DMMotorStop(motors[joint]);
        }
        if (leg->wheel_motor)
        {
            ChassisWheelFeedback *fb = &leg->wheel_feedback;
            const int16_t counts = enabled ? (int16_t)(fb->command_current_a * kC620CurrentCountsPerAmp) : 0;
            fb->command_current_a = (float)counts / kC620CurrentCountsPerAmp;
            DJIMotorSetRef(leg->wheel_motor, (float)counts * config->wheel[side].direction);
            if (enabled)
                DJIMotorEnable(leg->wheel_motor);
            else
                DJIMotorStop(leg->wheel_motor);
        }
    }
    taskEXIT_CRITICAL();
}
