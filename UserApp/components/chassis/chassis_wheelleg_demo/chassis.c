#include "chassis.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "task.h"

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
            if ((config->joint_mask & (1u << i)) && config->joint[i].can_init_config.can_handle == can)
                ++needed[bus];
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
            if ((config->wheel_mask & (1u << i)) && config->wheel[i].can_init_config.can_handle == can)
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
            leg->front_motor = DMMotorInit(&config->joint[front]);
            DMMotorStop(leg->front_motor);
        }
        if (config->joint_mask & (1u << rear))
        {
            leg->rear_motor = DMMotorInit(&config->joint[rear]);
            DMMotorStop(leg->rear_motor);
        }
        if (config->wheel_mask & (1u << side))
        {
            leg->wheel_motor = DJIMotorInit(&config->wheel[side]);
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

static void UpdateJoint(const DMMotorInstance *motor, const Chassis_Init_Config_s *config, unsigned i,
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
    const float transmission = config->joint_direction[i] / config->motor_per_joint[i];
    feedback->q_rad = (measure.total_angle - config->joint_zero_rad[i]) * transmission;
    feedback->qd_rad_s = measure.velocity * transmission;
    feedback->tau_nm = measure.torque * config->joint_direction[i] * config->motor_per_joint[i];
    feedback->torque_raw = torque_raw;
    feedback->temperature_c = fmaxf(measure.T_Mos, measure.T_Rotor);
    feedback->valid = fresh && dlc == 8u && measure.state == 1u && measure.id == config->joint_feedback_id[i] &&
                      isfinite(feedback->q_rad) && isfinite(feedback->qd_rad_s) && isfinite(feedback->tau_nm);
}

static void UpdateWheel(const DJIMotorInstance *motor, const Chassis_Init_Config_s *config, unsigned i,
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
    feedback->speed_rad_s =
        (float)rotor_rpm * kRpmToRadPerSec * config->wheel_direction[i] / config->motor_per_wheel[i];
    // The C620 manual specifies A/count for commands only.
    feedback->current_raw = (float)current_counts * config->wheel_direction[i];
    feedback->temperature_c = temperature;
    feedback->valid = fresh && dlc == 8u && isfinite(feedback->speed_rad_s);
}

static void UpdateGeometry(ChassisLeg *leg, const ChassisGeometryConfig *geometry)
{
    leg->geometry_valid = false;
    if (!leg->joint[CHASSIS_FRONT].valid || !leg->joint[CHASSIS_REAR].valid)
        return;

    // HKU concentric ACE model: virtual active rods are scaled by CE/HJ.
    const float front = leg->joint[CHASSIS_FRONT].q_rad;
    const float rear = leg->joint[CHASSIS_REAR].q_rad;
    const float scale = geometry->virtual_second_m / geometry->second_m;
    const float active = scale * geometry->first_m;
    const float half_diff = 0.5f * (front - rear);
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
    const float mean = 0.5f * (front + rear);
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
            UpdateJoint(leg->front_motor, config, 2u * side + CHASSIS_FRONT, &leg->joint[CHASSIS_FRONT], timeout_s);
        if (leg->rear_motor)
            UpdateJoint(leg->rear_motor, config, 2u * side + CHASSIS_REAR, &leg->joint[CHASSIS_REAR], timeout_s);
        UpdateGeometry(leg, &config->geometry);
        if (leg->wheel_motor)
            UpdateWheel(leg->wheel_motor, config, side, &leg->wheel_feedback, timeout_s);
    }
}

static float Clip(float value, float limit, bool *clipped)
{
    *clipped |= fabsf(value) > limit;
    return fmaxf(-limit, fminf(limit, value));
}

void ChassisTask(ChassisInstance *chassis, const ChassisRequest *request)
{
    const Chassis_Init_Config_s *config = chassis->config;
    float torque[CHASSIS_JOINT_COUNT] = {0};
    float torque_preclip[CHASSIS_JOINT_COUNT] = {0};
    int16_t current_counts[CHASSIS_LEG_COUNT] = {0};
    float current_preclip[CHASSIS_LEG_COUNT] = {0};
    bool clipped = false;
    bool fault = false;

    for (unsigned side = 0; side < CHASSIS_LEG_COUNT && !fault; ++side)
    {
        const ChassisLeg *leg = &chassis->legs[side];
        for (unsigned joint = 0; joint < 2 && !fault; ++joint)
        {
            const unsigned i = 2u * side + joint;
            if (!(request->joint_enable_mask & (1u << i)))
                continue;
            const ChassisJointFeedback *feedback = &leg->joint[joint];
            const bool direct = request->joint_direct_torque_mask & (1u << i);
            const float reference = direct ? request->joint_torque_ref_nm[i] : request->q_ref_rad[i];
            const float base = direct ? reference
                                      : config->joint_kp_nm_per_rad[i] * (reference - feedback->q_rad) -
                                            config->joint_kd_nm_per_rad_s[i] * feedback->qd_rad_s;
            const float output = base + request->joint_torque_offset_nm[i];
            fault = !feedback->valid || !isfinite(output);
            if (!fault)
            {
                torque_preclip[i] = output;
                const float motor_limit = DM_T_MAX_J8009P * config->motor_per_joint[i];
                torque[i] = Clip(output, fminf(config->joint_max_torque_nm[i], motor_limit), &clipped);
            }
        }
        if (!fault && (request->wheel_enable_mask & (1u << side)))
        {
            const bool direct = request->wheel_current_mode_mask & (1u << side);
            const float output = direct ? request->wheel_current_ref_a[side]
                                        : config->wheel_kp_a_per_rad_s[side] *
                                              (request->wheel_speed_ref_rad_s[side] - leg->wheel_feedback.speed_rad_s);
            fault = !leg->wheel_feedback.valid || !isfinite(output);
            if (!fault)
            {
                current_preclip[side] = output;
                const float limited =
                    Clip(output, fminf(config->wheel_max_current_a[side], kC620MaxCurrentA), &clipped);
                current_counts[side] = (int16_t)(limited * kC620CurrentCountsPerAmp);
            }
        }
    }

    chassis->fault = fault;
    chassis->clipped = clipped && !fault;
    chassis->joint_enabled_mask = fault ? 0u : request->joint_enable_mask;
    chassis->wheel_enabled_mask = fault ? 0u : request->wheel_enable_mask;
    taskENTER_CRITICAL();
    chassis->command_time_us = DWT_GetTimeline_us();
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &chassis->legs[side];
        DMMotorInstance *motors[2] = {leg->front_motor, leg->rear_motor};
        for (unsigned joint = 0; joint < 2; ++joint)
        {
            if (!motors[joint])
                continue;
            const unsigned i = 2u * side + joint;
            const bool enable = chassis->joint_enabled_mask & (1u << i);
            const float applied = enable ? torque[i] : 0.0f;
            leg->joint[joint].command_torque_preclip_nm = torque_preclip[i];
            leg->joint[joint].command_torque_nm = applied;
            if (enable)
            {
                DMMotorSetRef(motors[joint], applied * config->joint_direction[i] / config->motor_per_joint[i]);
                DMMotorEnable(motors[joint]);
            }
            else
            {
                DMMotorSetRef(motors[joint], 0.0f);
                DMMotorStop(motors[joint]);
            }
        }
        if (leg->wheel_motor)
        {
            const bool enable = chassis->wheel_enabled_mask & (1u << side);
            const int16_t applied = enable ? current_counts[side] : 0;
            leg->wheel_feedback.command_current_preclip_a = current_preclip[side];
            leg->wheel_feedback.command_current_a = (float)applied / kC620CurrentCountsPerAmp;
            DJIMotorSetRef(leg->wheel_motor, (float)applied * config->wheel_direction[side]);
            if (enable)
                DJIMotorEnable(leg->wheel_motor);
            else
                DJIMotorStop(leg->wheel_motor);
        }
    }
    taskEXIT_CRITICAL();
}
