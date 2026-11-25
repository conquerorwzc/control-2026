#include "chassis.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "task.h"

static const float kRpmToRadPerSec = 0.1047197551f;
static const float kC620MaxCurrentA = 20.0f;
static const float kC620CurrentCountsPerAmp = 16384.0f / 20.0f;
static ChassisInstance instance;
static bool initialized;

static bool Positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool Nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static bool DirectionValid(float direction)
{
    return isfinite(direction) && (direction == 1.0f || direction == -1.0f);
}

static int CanBus(const CAN_Init_Config_s *can)
{
    if (can->can_handle == &hcan1)
        return 0;
    if (can->can_handle == &hcan2)
        return 1;
#if defined(STM32H723xx)
    if (can->can_handle == &hcan3)
        return 2;
#endif
    return -1;
}

static bool CheckCan(const Chassis_Init_Config_s *config, const CAN_Init_Config_s *can, uint32_t rx_id,
                     unsigned used[DEVICE_CAN_CNT], uint32_t feedback_ids[DEVICE_CAN_CNT][6])
{
    const int bus = CanBus(can);
    if (bus < 0 || !rx_id || rx_id > 0x7ffu || !can->tx_id || can->tx_id > 0x7ffu || can->tx_id == rx_id)
        return false;
    unsigned capacity = 14;
    unsigned reserved = 0;
#if defined(STM32H723xx)
    // CAN1/2 can grow before the first register. CAN3 keeps its generated layout.
    capacity = bus < 2 ? 128u : can->can_handle->Init.StdFiltersNbr;
    reserved = config->reserved_std_filters[bus];
#else
    (void)config;
#endif
    if (reserved >= capacity || used[bus] >= capacity - reserved)
        return false;
    for (unsigned i = 0; i < used[bus]; ++i)
    {
        if (feedback_ids[bus][i] == rx_id)
            return false;
    }
    feedback_ids[bus][used[bus]++] = rx_id;
    return true;
}

#if defined(STM32H723xx)
static bool ConfigureCanFilters(const Chassis_Init_Config_s *config, const unsigned used[DEVICE_CAN_CNT])
{
    FDCAN_HandleTypeDef *const buses[DEVICE_CAN_CNT] = {&hfdcan1, &hfdcan2, &hfdcan3};
    // STM32H723 HAL's SRAMCAN end address. HAL checks against this address when it lays out the blocks.
    const uint32_t ram_end = SRAMCAN_BASE + 0x2800u - sizeof(uint32_t);
    unsigned required[2];

    for (unsigned bus = 0; bus < 2; ++bus)
    {
        FDCAN_HandleTypeDef *can = buses[bus];
        required[bus] = used[bus] ? used[bus] + config->reserved_std_filters[bus] : 0;
        const unsigned existing = can->Init.StdFiltersNbr;
        if (required[bus] <= existing)
            continue;

        // The first BSP registration starts every handle. READY and no reservations are required for re-init.
        if (can->State != HAL_FDCAN_STATE_READY || config->reserved_std_filters[bus])
            return false;

        const uint32_t start = can->msgRam.StandardFilterSA;
        const uint32_t end = can->msgRam.EndAddress;
        uint32_t limit = ram_end;
        if (start < SRAMCAN_BASE || end < start || end > ram_end)
            return false;
        for (unsigned other = 0; other < DEVICE_CAN_CNT; ++other)
        {
            if (other == bus)
                continue;
            const FDCAN_HandleTypeDef *neighbor = buses[other];
            const uint32_t neighbor_start = neighbor->msgRam.StandardFilterSA;
            const uint32_t neighbor_end = neighbor->msgRam.EndAddress;
            if (neighbor->State != HAL_FDCAN_STATE_READY || neighbor_start < SRAMCAN_BASE ||
                neighbor_end < neighbor_start || neighbor_end > ram_end)
                return false;
            if (neighbor_start > start)
            {
                if (neighbor_start < limit)
                    limit = neighbor_start;
            }
            else if (neighbor_end > start)
                return false;
        }
        if (end > limit || (required[bus] - existing) * sizeof(uint32_t) > limit - end)
            return false;
    }

    for (unsigned bus = 0; bus < 2; ++bus)
    {
        FDCAN_HandleTypeDef *can = buses[bus];
        if (required[bus] > can->Init.StdFiltersNbr)
        {
            can->Init.StdFiltersNbr = required[bus];
            if (HAL_FDCAN_Init(can) != HAL_OK)
                return false;
        }
    }
    return true;
}
#endif

static bool ConfigValid(const Chassis_Init_Config_s *config, unsigned used[DEVICE_CAN_CNT])
{
    if (!config || !config->feedback_timeout_ms || (config->joint_mask & ~15u) || (config->wheel_mask & ~3u) ||
        !(config->joint_mask | config->wheel_mask))
        return false;

    const ChassisGeometryConfig *geometry = &config->geometry;
    if (config->joint_mask && (!Positive(geometry->first_m) || !Positive(geometry->second_m) ||
                               !Positive(geometry->virtual_second_m) || !Positive(geometry->singular_epsilon_m)))
        return false;

    uint32_t feedback_ids[DEVICE_CAN_CNT][6] = {0};
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        const unsigned pair = (config->joint_mask >> (2u * side)) & 3u;
        if (pair != 0u && pair != 3u)
            return false;
        for (unsigned joint = 0; joint < 2 && pair; ++joint)
        {
            const ChassisJointConfig *entry = &config->joint[2u * side + joint];
            const Motor_Control_Setting_s *settings = &entry->motor.controller_setting_init_config;
            if (entry->feedback_id > 15u || !DirectionValid(entry->direction) || !Positive(entry->motor_per_joint) ||
                !isfinite(entry->zero_rad) || !Nonnegative(entry->kp_nm_per_rad) ||
                !Nonnegative(entry->kd_nm_per_rad_s) || !Positive(entry->max_torque_nm) ||
                entry->motor.motor_type != J8009P ||
                (unsigned)settings->motor_reverse_flag != (unsigned)settings->feedback_reverse_flag ||
                !CheckCan(config, &entry->motor.can_init_config, entry->motor.can_init_config.rx_id, used,
                          feedback_ids))
                return false;
        }
        if (config->wheel_mask & (1u << side))
        {
            const ChassisWheelConfig *entry = &config->wheel[side];
            const CAN_Init_Config_s *can = &entry->motor.can_init_config;
            const Motor_Control_Setting_s *settings = &entry->motor.controller_setting_init_config;
            if (entry->motor.motor_type != M3508 || !DirectionValid(entry->direction) ||
                !Positive(entry->motor_per_wheel) || !Nonnegative(entry->kp_a_per_rad_s) ||
                !Positive(entry->max_current_a) || settings->motor_reverse_flag != MOTOR_DIRECTION_NORMAL ||
                settings->feedback_reverse_flag != FEEDBACK_DIRECTION_NORMAL || can->tx_id > 8u ||
                !CheckCan(config, can, 0x200u + can->tx_id, used, feedback_ids))
                return false;
        }
    }
    return true;
}

ChassisInstance *ChassisInit(const Chassis_Init_Config_s *config)
{
    unsigned used[DEVICE_CAN_CNT] = {0};
    if (initialized || !ConfigValid(config, used))
        return NULL;
#if defined(STM32H723xx)
    if (!ConfigureCanFilters(config, used))
        return NULL;
#endif

    instance.config = *config;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &instance.legs[side];
        if (config->joint_mask & (3u << (2u * side)))
        {
            Motor_Init_Config_s front = config->joint[2u * side + CHASSIS_FRONT].motor;
            Motor_Init_Config_s rear = config->joint[2u * side + CHASSIS_REAR].motor;
            leg->front_motor = DMMotorInit(&front);
            DMMotorStop(leg->front_motor);
            leg->rear_motor = DMMotorInit(&rear);
            DMMotorStop(leg->rear_motor);
        }
        if (config->wheel_mask & (1u << side))
        {
            Motor_Init_Config_s wheel = config->wheel[side].motor;
            leg->wheel_motor = DJIMotorInit(&wheel);
            DJIMotorStop(leg->wheel_motor);
        }
    }
    initialized = true;
    return &instance;
}

static bool FeedbackFresh(uint32_t counter, float timeout_s, uint32_t *previous, bool *seen, bool *updated,
                          uint64_t *feedback_time_us)
{
    *updated = counter != *previous;
    uint32_t sample = counter;
    taskENTER_CRITICAL();
    const double age_s = DWT_GetDeltaT64(&sample);
    const uint64_t now_us = DWT_GetTimeline_us();
    taskEXIT_CRITICAL();
    if (*updated)
    {
        *previous = counter;
        *seen = true;
        const uint64_t age_us = (uint64_t)(age_s * 1000000.0);
        *feedback_time_us = age_us <= now_us ? now_us - age_us : 0u;
    }

    // Expired data stays expired until a new frame, including across a DWT counter wrap.
    *seen = *seen && age_s <= timeout_s;
    return *seen;
}

static void UpdateJoint(const DMMotorInstance *motor, const ChassisJointConfig *config, ChassisJointFeedback *feedback,
                        float timeout_s)
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
    const float transmission = config->direction / config->motor_per_joint;
    feedback->q_rad = (measure.total_angle - config->zero_rad) * transmission;
    feedback->qd_rad_s = measure.velocity * transmission;
    feedback->tau_nm = measure.torque * config->direction * config->motor_per_joint;
    feedback->torque_raw = torque_raw;
    feedback->temperature_c = fmaxf(measure.T_Mos, measure.T_Rotor);
    feedback->valid = fresh && dlc == 8u && measure.state == 1u && measure.id == config->feedback_id &&
                      isfinite(feedback->q_rad) && isfinite(feedback->qd_rad_s) && isfinite(feedback->tau_nm);
}

static void UpdateWheel(const DJIMotorInstance *motor, const ChassisWheelConfig *config, ChassisWheelFeedback *feedback,
                        float timeout_s)
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
    feedback->speed_rad_s = (float)rotor_rpm * kRpmToRadPerSec * config->direction / config->motor_per_wheel;
    // The C620 manual specifies A/count for commands only.
    feedback->current_raw = (float)current_counts * config->direction;
    feedback->temperature_c = temperature;
    feedback->valid = fresh && dlc == 8u && isfinite(feedback->speed_rad_s);
}

static void UpdateGeometry(ChassisLeg *leg, const ChassisGeometryConfig *geometry)
{
    leg->geometry_valid = false;
    leg->geometry_updated = false;
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
        leg->geometry_updated = leg->joint[CHASSIS_FRONT].updated || leg->joint[CHASSIS_REAR].updated;
    }
}

void ChassisFeedbackUpdate(ChassisInstance *chassis)
{
    if (!chassis)
        return;
    const float timeout_s = (float)chassis->config.feedback_timeout_ms * 0.001f;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        ChassisLeg *leg = &chassis->legs[side];
        if (leg->front_motor)
        {
            UpdateJoint(leg->front_motor, &chassis->config.joint[2u * side + CHASSIS_FRONT], &leg->joint[CHASSIS_FRONT],
                        timeout_s);
            UpdateJoint(leg->rear_motor, &chassis->config.joint[2u * side + CHASSIS_REAR], &leg->joint[CHASSIS_REAR],
                        timeout_s);
            UpdateGeometry(leg, &chassis->config.geometry);
        }
        if (leg->wheel_motor)
            UpdateWheel(leg->wheel_motor, &chassis->config.wheel[side], &leg->wheel_feedback, timeout_s);
    }
}

static float Clip(float value, float limit, bool *clipped)
{
    *clipped |= fabsf(value) > limit;
    return fmaxf(-limit, fminf(limit, value));
}

void ChassisTask(ChassisInstance *chassis)
{
    if (!chassis)
        return;

    const ChassisRequest request = chassis->request;
    const Chassis_Init_Config_s *config = &chassis->config;
    float torque[CHASSIS_JOINT_COUNT] = {0};
    float torque_preclip[CHASSIS_JOINT_COUNT] = {0};
    int16_t current_counts[CHASSIS_LEG_COUNT] = {0};
    float current_preclip[CHASSIS_LEG_COUNT] = {0};
    bool clipped = false;
    bool fault = (request.joint_enable_mask & ~config->joint_mask) ||
                 (request.wheel_enable_mask & ~config->wheel_mask) ||
                 (request.joint_direct_torque_mask & ~request.joint_enable_mask) ||
                 (request.wheel_current_mode_mask & ~request.wheel_enable_mask);

    for (unsigned side = 0; side < CHASSIS_LEG_COUNT && !fault; ++side)
    {
        const ChassisLeg *leg = &chassis->legs[side];
        for (unsigned joint = 0; joint < 2 && !fault; ++joint)
        {
            const unsigned i = 2u * side + joint;
            if (!(request.joint_enable_mask & (1u << i)))
                continue;
            const ChassisJointConfig *entry = &config->joint[i];
            const ChassisJointFeedback *feedback = &leg->joint[joint];
            const bool direct = request.joint_direct_torque_mask & (1u << i);
            const float reference = direct ? request.joint_torque_ref_nm[i] : request.q_ref_rad[i];
            const float base = direct ? reference
                                      : entry->kp_nm_per_rad * (reference - feedback->q_rad) -
                                            entry->kd_nm_per_rad_s * feedback->qd_rad_s;
            const float output = base + request.joint_torque_offset_nm[i];
            fault = !feedback->valid || !isfinite(output);
            if (!fault)
            {
                torque_preclip[i] = output;
                const float motor_limit = DM_T_MAX_J8009P * entry->motor_per_joint;
                torque[i] = Clip(output, fminf(entry->max_torque_nm, motor_limit), &clipped);
            }
        }
        if (!fault && (request.wheel_enable_mask & (1u << side)))
        {
            const ChassisWheelConfig *entry = &config->wheel[side];
            const bool direct = request.wheel_current_mode_mask & (1u << side);
            const float output = direct ? request.wheel_current_ref_a[side]
                                        : entry->kp_a_per_rad_s *
                                              (request.wheel_speed_ref_rad_s[side] - leg->wheel_feedback.speed_rad_s);
            fault = !leg->wheel_feedback.valid || !isfinite(output);
            if (!fault)
            {
                current_preclip[side] = output;
                const float limited = Clip(output, fminf(entry->max_current_a, kC620MaxCurrentA), &clipped);
                current_counts[side] = (int16_t)(limited * kC620CurrentCountsPerAmp);
            }
        }
    }

    chassis->fault = fault;
    chassis->clipped = clipped && !fault;
    chassis->applied_request = fault ? (ChassisRequest){0} : request;
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
            const bool enable = chassis->applied_request.joint_enable_mask & (1u << i);
            const ChassisJointConfig *entry = &config->joint[i];
            const float applied = enable ? torque[i] : 0.0f;
            leg->joint[joint].command_torque_preclip_nm = torque_preclip[i];
            leg->joint[joint].command_torque_nm = applied;
            DMMotorSetRef(motors[joint], applied * entry->direction / entry->motor_per_joint);
            if (enable)
                DMMotorEnable(motors[joint]);
            else
                DMMotorStop(motors[joint]);
        }
        if (leg->wheel_motor)
        {
            const ChassisWheelConfig *entry = &config->wheel[side];
            const bool enable = chassis->applied_request.wheel_enable_mask & (1u << side);
            const int16_t applied = enable ? current_counts[side] : 0;
            leg->wheel_feedback.command_current_preclip_a = current_preclip[side];
            leg->wheel_feedback.command_current_a = (float)applied / kC620CurrentCountsPerAmp;
            DJIMotorSetRef(leg->wheel_motor, (float)applied * entry->direction);
            if (enable)
                DJIMotorEnable(leg->wheel_motor);
            else
                DJIMotorStop(leg->wheel_motor);
        }
    }
    taskEXIT_CRITICAL();
}
