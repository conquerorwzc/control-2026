#include "vofa_watch.h"

#include "usart.h"
#include "vofa.h"

enum
{
    kBaudRate = 921600u,
    kChannelCount = 25,
    kFormatVersion = 3,
    kFourHips = 0,
    kSingleLeg = 1,
    kWheels = 2,
    kNoTimeOffset = -16777216
};

_Static_assert(sizeof(float) == 4 && kChannelCount <= VOFA_MAX_CHANNELS, "JustFloat layout mismatch");
_Static_assert((kChannelCount + 1u) * 4u * 10u * SYSID_CONTROL_HZ <= kBaudRate && kBaudRate <= 1000000u,
               "SysID exceeds the 8N1 serial link budget");
_Static_assert(SYSID_STATE_DRY_RUNNING < 16 && SYSID_WHEEL_COAST < 16 && SYSID_HIP_BOTH_ANTI_PHASE < 4 &&
                   SYSID_PHASE_POSITIONING < 8,
               "SysID telemetry metadata fields overflow");

static bool vofa_ready;

bool VofaInit(void)
{
    huart7.Init.BaudRate = kBaudRate;
    if (HAL_UART_Init(&huart7) != HAL_OK)
        return false;
    VOFAInit(&huart7);
    vofa_ready = true;
    return true;
}

static float TimeOffset(uint64_t event_us, uint64_t tick_us, bool valid)
{
    const uint64_t distance = event_us >= tick_us ? event_us - tick_us : tick_us - event_us;
    if (!valid || distance >= (uint64_t)(-kNoTimeOffset))
        return (float)kNoTimeOffset;
    return event_us >= tick_us ? (float)distance : -(float)distance;
}

void VofaTick(const ChassisRequest *command, uint64_t now_us, bool online, unsigned configured_joints, SysIdTrial trial,
              SysIdHipPattern hip_pattern, int direction)
{
    static uint16_t frame_seq;
    // DMA 共用发送缓冲，忙时只推进序号，避免覆盖正在发送的帧。
    const uint16_t seq = frame_seq++;
    if (!vofa_ready || huart7.gState != HAL_UART_STATE_READY)
        return;

    const RobotInstance *robot = RobotGet();
    float channel[kChannelCount] = {0};
    const ChassisInstance *chassis = robot->chassis;
    const unsigned applied =
        chassis && chassis->enabled ? chassis->config->joint_mask | (chassis->config->wheel_mask << 4) : 0u;
    const unsigned side = configured_joints == 0xcu ? CHASSIS_RIGHT : CHASSIS_LEFT;
    const unsigned layout = SYSID_IS_HIP(trial) ? (configured_joints == 0xfu ? kFourHips : kSingleLeg) : kWheels;

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

    if (layout == kFourHips)
    {
        channel[22] = command->hip_position_rad[0];
        channel[23] = command->hip_position_rad[1];
    }
    else
    {
        channel[6] = TimeOffset(chassis ? chassis->command_time_us : 0u, now_us, chassis != NULL);
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
        {
            float *sample = &channel[7u + 9u * i];
            if (layout == kSingleLeg)
            {
                const unsigned joint = 2u * side + i;
                const ChassisJointFeedback fb = chassis ? chassis->legs[side].joint[i] : (ChassisJointFeedback){0};
                sample[0] = command->hip_position_rad[joint];
                sample[1] = fb.q_rad;
                sample[2] = fb.qd_rad_s;
                sample[3] = fb.command_torque_preclip_nm;
                sample[4] = fb.command_torque_nm;
                sample[5] = fb.tau_nm;
                sample[6] = fb.torque_raw;
                sample[7] = fb.temperature_c;
                sample[8] = TimeOffset(fb.feedback_time_us, now_us, fb.seen);
            }
            else
            {
                const ChassisWheelFeedback fb = chassis ? chassis->legs[i].wheel_feedback : (ChassisWheelFeedback){0};
                sample[0] = command->wheel_current_mode ? 0.0f : command->wheel_reference[i];
                sample[1] = command->wheel_current_mode ? command->wheel_reference[i] : 0.0f;
                sample[2] = fb.speed_rad_s;
                sample[3] = fb.command_current_preclip_a;
                sample[4] = fb.command_current_a;
                sample[5] = fb.current_raw;
                sample[6] = fb.temperature_c;
                sample[7] = TimeOffset(fb.feedback_time_us, now_us, fb.seen);
                sample[8] = command->wheel_current_mode;
            }
        }
    }

    // Split the wrapping microsecond counter into two words that float32 can represent exactly.
    const uint32_t now = (uint32_t)now_us;
    channel[0] = seq;
    channel[1] = (uint16_t)(now >> 16);
    channel[2] = (uint16_t)now;
    channel[3] = (float)((unsigned)robot->state | ((unsigned)trial << 4) | ((unsigned)hip_pattern << 8) |
                         ((unsigned)robot->phase << 10) | ((unsigned)(direction < 0) << 13) | ((unsigned)side << 14) |
                         (layout << 15) | (kFormatVersion << 17));
    channel[4] = robot->mode;
    channel[5] = (float)(updated | ((unsigned)online << 6) | ((unsigned)(applied != 0u) << 7) |
                         ((unsigned)(chassis && chassis->clipped) << 8) |
                         ((unsigned)(robot->state == SYSID_STATE_FAULT) << 9) | (valid << 12) | (applied << 18));

    VOFAJustFloatSend(channel, kChannelCount);
}
