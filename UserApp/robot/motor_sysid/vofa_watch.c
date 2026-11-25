#include "vofa_watch.h"

#include "robot_config.h"
#include "vofa.h"

enum
{
    kChannelCount = 25,
    kFormatVersion = 3,
    kFourHips = 0,
    kSingleLeg = 1,
    kWheels = 2,
    kConfiguration = 3,
    kTimingPage = 7,
    kNoTimeOffset = -16777216
};

_Static_assert(sizeof(float) == 4 && kChannelCount <= VOFA_MAX_CHANNELS, "JustFloat layout mismatch");
_Static_assert((kChannelCount + 1u) * 4u * 10u * SYSID_CONTROL_HZ <= SYSID_VOFA_BAUD && SYSID_VOFA_BAUD <= 1000000u,
               "SysID exceeds the 8N1 serial link budget");
_Static_assert(SYSID_STATE_DRY_RUNNING < 16 && SYSID_WHEEL_COAST < 16 && SYSID_HIP_BOTH_ANTI_PHASE < 4 &&
                   SYSID_PHASE_MOTION_DETECTED < 8 && SYSID_FAULT_CHASSIS < 8,
               "SysID telemetry metadata fields overflow");

static uint16_t frame_seq;
static unsigned configuration_page;

bool SysIdVofaInit(void)
{
    UART_HandleTypeDef *uart = SYSID_VOFA_PORT;
    uart->Init.BaudRate = SYSID_VOFA_BAUD;
    if (HAL_UART_Init(uart) != HAL_OK || uart->hdmatx == NULL || uart->hdmarx == NULL ||
        uart->Init.WordLength != UART_WORDLENGTH_8B || uart->Init.StopBits != UART_STOPBITS_1 ||
        uart->Init.Parity != UART_PARITY_NONE)
        return false;
    VOFAInit(uart);
    return true;
}

static float TimeOffset(uint64_t event_us, uint64_t tick_us, bool valid)
{
    const uint64_t distance = event_us >= tick_us ? event_us - tick_us : tick_us - event_us;
    if (!valid || distance >= (uint64_t)(-kNoTimeOffset))
        return (float)kNoTimeOffset;
    return event_us >= tick_us ? (float)distance : -(float)distance;
}

static void WriteConfiguration(float *out, const RobotInstance *robot)
{
    const unsigned page = configuration_page;
    configuration_page = (page + 1u) % (kTimingPage + 1u);
    out[0] = page;
    const ChassisInstance *chassis = robot->chassis;
    if (page == 0u)
    {
        out[1] = (uint16_t)(robot->workpoint_id >> 16);
        out[2] = (uint16_t)robot->workpoint_id;
        out[3] = robot->direction;
        out[4] = robot->selected_parameter;
        out[5] = robot->real_hardware;
        out[6] = robot->config_valid;
        out[7] = chassis ? chassis->config.joint_mask : 0u;
        out[8] = chassis ? chassis->config.wheel_mask : 0u;
    }
    else if (page <= CHASSIS_JOINT_COUNT)
    {
        const unsigned i = page - 1u;
        out[1] = robot->center_rad[i];
        if (chassis && (chassis->config.joint_mask & (1u << i)))
        {
            const ChassisJointConfig *config = &chassis->config.joint[i];
            out[2] = 1;
            out[3] = config->kp_nm_per_rad;
            out[4] = config->kd_nm_per_rad_s;
            out[5] = config->max_torque_nm;
            out[6] = config->zero_rad;
            out[7] = config->direction;
            out[8] = config->motor_per_joint;
        }
    }
    else if (page < kTimingPage)
    {
        const unsigned side = page - 1u - CHASSIS_JOINT_COUNT;
        if (chassis && (chassis->config.wheel_mask & (1u << side)))
        {
            const ChassisWheelConfig *config = &chassis->config.wheel[side];
            out[1] = 1;
            out[2] = config->kp_a_per_rad_s;
            out[3] = config->max_current_a;
            out[4] = config->direction;
            out[5] = config->motor_per_wheel;
        }
    }
    else
    {
        const SysIdAmplitude *amplitude = robot->real_hardware ? &kRealAmplitude : &kPreviewAmplitude;
        out[1] = kTrialTiming.sweep_start_hz;
        out[2] = kTrialTiming.sweep_end_hz;
        out[3] = kTrialTiming.sweep_duration_s;
        out[4] = kTrialTiming.sweep_ramp_s;
        out[5] = kTrialTiming.step_baseline_s;
        out[6] = kTrialTiming.step_hold_s;
        out[7] = kTrialTiming.step_recovery_s;
        out[8] = kTrialTiming.steady_s;
        out[9] = kTrialTiming.coast_s;
        out[10] = kTrialTiming.hold_average_s;
        out[11] = kTrialTiming.motion_confirmation_s;
        out[12] = kTrialTiming.triangle_cycles;
        out[13] = amplitude->hip_ramp_max_nm;
        out[14] = amplitude->hip_motion_rad_s;
        out[15] = amplitude->wheel_ramp_max_a;
        out[16] = amplitude->wheel_motion_rad_s;
        out[17] = amplitude->triangle_rad;
    }
}

static void WriteSingleLeg(float *out, const RobotInstance *robot, unsigned side)
{
    const ChassisLeg *leg = robot->chassis ? &robot->chassis->legs[side] : NULL;
    for (unsigned j = 0; j < 2; ++j, out += 9)
    {
        const unsigned i = 2u * side + j;
        const ChassisJointFeedback fb = leg ? leg->joint[j] : (ChassisJointFeedback){0};
        out[0] = robot->request.q_ref_rad[i];
        out[1] = fb.q_rad;
        out[2] = fb.qd_rad_s;
        out[3] = fb.command_torque_preclip_nm;
        out[4] = fb.command_torque_nm;
        out[5] = fb.tau_nm;
        out[6] = fb.torque_raw;
        out[7] = fb.temperature_c;
        out[8] = TimeOffset(fb.feedback_time_us, robot->now_us, fb.seen);
    }
}

static void WriteWheel(float *out, const RobotInstance *robot, unsigned side)
{
    const ChassisLeg *leg = robot->chassis ? &robot->chassis->legs[side] : NULL;
    const ChassisWheelFeedback fb = leg ? leg->wheel_feedback : (ChassisWheelFeedback){0};
    out[0] = robot->request.wheel_speed_ref_rad_s[side];
    out[1] = robot->request.wheel_current_ref_a[side];
    out[2] = fb.speed_rad_s;
    out[3] = fb.command_current_preclip_a;
    out[4] = fb.command_current_a;
    out[5] = fb.current_raw;
    out[6] = fb.temperature_c;
    out[7] = TimeOffset(fb.feedback_time_us, robot->now_us, fb.seen);
    out[8] = (robot->request.wheel_current_mode_mask & (1u << side)) != 0;
}

void SysIdVofaSend(const RobotInstance *robot)
{
    const uint16_t seq = frame_seq++;
    if (SYSID_VOFA_PORT->gState != HAL_UART_STATE_READY)
        return;

    float channel[kChannelCount] = {0};
    const ChassisInstance *chassis = robot->chassis;
    const unsigned hip_mask = chassis ? chassis->applied_request.joint_enable_mask : 0u;
    const unsigned wheel_mask = chassis ? chassis->applied_request.wheel_enable_mask : 0u;
    const unsigned applied = (hip_mask & 0xfu) | ((wheel_mask & 0x3u) << 4);
    const unsigned configured = chassis ? chassis->config.joint_mask : kSysIdChassisConfig.joint_mask;
    const unsigned side = configured == 0xcu ? CHASSIS_RIGHT : CHASSIS_LEFT;
    unsigned layout = SYSID_IS_HIP(robot->trial) ? (configured == 0xfu ? kFourHips : kSingleLeg) : kWheels;
    if (applied == 0u && robot->state == SYSID_STATE_READY)
        layout = kConfiguration;

    unsigned updated = 0;
    unsigned valid = 0;
    if (chassis)
    {
        for (unsigned s = 0; s < CHASSIS_LEG_COUNT; ++s)
        {
            const ChassisLeg *leg = &chassis->legs[s];
            for (unsigned j = 0; j < 2; ++j)
            {
                const unsigned i = 2u * s + j;
                const ChassisJointFeedback *fb = &leg->joint[j];
                updated |= (unsigned)(fb->valid && fb->updated) << i;
                valid |= (unsigned)fb->valid << i;
                if (layout == kFourHips)
                {
                    float *hip = &channel[6u + 4u * i];
                    hip[0] = fb->q_rad;
                    hip[1] = fb->qd_rad_s;
                    hip[2] = fb->command_torque_nm;
                    hip[3] = fb->tau_nm;
                    if (i == seq % CHASSIS_JOINT_COUNT)
                        channel[24] = fb->temperature_c;
                }
            }
            const ChassisWheelFeedback *fb = &leg->wheel_feedback;
            updated |= (unsigned)(fb->valid && fb->updated) << (4u + s);
            valid |= (unsigned)fb->valid << (4u + s);
        }
    }

    if (layout == kConfiguration)
        WriteConfiguration(&channel[6], robot);
    else if (layout == kFourHips)
    {
        channel[22] = robot->request.q_ref_rad[0];
        channel[23] = robot->request.q_ref_rad[1];
    }
    else
    {
        channel[6] = TimeOffset(chassis ? chassis->command_time_us : 0u, robot->now_us, chassis != NULL);
        if (layout == kSingleLeg)
            WriteSingleLeg(&channel[7], robot, side);
        else
            for (unsigned s = 0; s < CHASSIS_LEG_COUNT; ++s)
                WriteWheel(&channel[7u + 9u * s], robot, s);
    }

    // Split the wrapping microsecond counter into two words that float32 can represent exactly.
    const uint32_t now = (uint32_t)robot->now_us;
    channel[0] = seq;
    channel[1] = (uint16_t)(now >> 16);
    channel[2] = (uint16_t)now;
    channel[3] = (float)((unsigned)robot->state | ((unsigned)robot->trial << 4) | ((unsigned)robot->hip_pattern << 8) |
                         ((unsigned)robot->phase << 10) | ((unsigned)(robot->direction < 0) << 13) |
                         ((unsigned)side << 14) | (layout << 15) | (kFormatVersion << 17));
    channel[4] = robot->mode;
    channel[5] = (float)(updated | ((unsigned)robot->remote_online << 6) |
                         ((unsigned)(robot->real_hardware && applied != 0u) << 7) |
                         ((unsigned)(chassis && chassis->clipped) << 8) | ((unsigned)robot->fault << 9) |
                         (valid << 12) | (applied << 18));

    VOFAJustFloatSend(channel, kChannelCount);
}
