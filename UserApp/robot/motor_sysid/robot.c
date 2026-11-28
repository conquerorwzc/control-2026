#include "robot.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "task.h"
#include "user_lib.h"
#include "vofa_watch.h"

static RobotInstance robot = {.hip_pattern = SYSID_HIP_PATTERN};
static LegLengthState length_state[CHASSIS_LEG_COUNT];
static float hold_sum_nm[CHASSIS_JOINT_COUNT];
static unsigned hold_samples;
static unsigned moving_mask;
static uint64_t stable_since_us;
static uint64_t moving_since_us;
static uint64_t start_us;
static float sweep_half_delta_change;
static float sweep_hold_bias_nm[2];
static uint64_t sweep_bias_time_us;
static const unsigned kAllJoints = (1u << CHASSIS_JOINT_COUNT) - 1u;
_Static_assert(SYSID_TEST_SIDES >= 1u && SYSID_TEST_SIDES <= 3u, "Select left, right or both legs");
_Static_assert(SYSID_TRIAL_DIRECTION == 1 || SYSID_TRIAL_DIRECTION == -1, "Select a trial direction");
_Static_assert(SYSID_EXPERIMENT <= SYSID_HIP_STATIC && SYSID_HIP_PATTERN <= SYSID_HIP_BOTH_ANTI_PHASE,
               "Select a valid trial and hip pattern");

static bool WheelsConfigured(void)
{
    const Chassis_Init_Config_s *c = &chassis_init_config;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        if (!(c->wheel_mask & (1u << side)))
            continue;
        if (fabsf(c->wheel[side].direction) != 1.0f || !(c->wheel[side].max_current_a > 0.0f) ||
            !(c->wheel[side].max_speed_rad_s > 0.0f) ||
            (SYSID_EXPERIMENT != SYSID_WHEEL_CURRENT_RAMP && !(c->wheel[side].kp_a_per_rad_s > 0.0f)))
            return false;
    }
    return true;
}

static SysIdMode SelectMode(uint8_t right)
{
    switch (SYSID_EXPERIMENT)
    {
    case SYSID_HIP_STATIC:
        if (switch_is_up(right))
            return SYSID_MODE_STATIC_HOLD;
        return switch_is_mid(right) ? SYSID_MODE_STATIC_GRID : SYSID_MODE_NONE;
    case SYSID_HIP:
        if (switch_is_up(right))
            return SYSID_MODE_HIP_IN_PHASE;
        if (switch_is_mid(right) && SYSID_JOINT_MASK == kAllJoints)
            return SYSID_MODE_HIP_ANTI_PHASE;
        if (switch_is_down(right))
            return SYSID_MODE_LEG_LENGTH;
        break;
    case SYSID_HIP_STEP:
        return switch_is_mid(right) ? SYSID_MODE_HIP_STEP : SYSID_MODE_NONE;
    case SYSID_HIP_RAMP:
        return switch_is_mid(right) ? SYSID_MODE_HIP_RAMP : SYSID_MODE_NONE;
    case SYSID_HIP_TRIANGLE:
        if (switch_is_up(right))
            return SYSID_MODE_HIP_TRIANGLE_02;
        if (switch_is_mid(right))
            return SYSID_MODE_HIP_TRIANGLE_05;
        if (switch_is_down(right))
            return SYSID_MODE_HIP_TRIANGLE_10;
        break;
    case SYSID_WHEEL_STEP:
    case SYSID_WHEEL_STEADY:
    case SYSID_WHEEL_COAST:
        if (switch_is_up(right))
            return SYSID_MODE_WHEEL_STEP_10;
        if (switch_is_mid(right))
            return SYSID_MODE_WHEEL_STEP_30;
        if (switch_is_down(right))
            return SYSID_MODE_WHEEL_STEP_60;
        break;
    case SYSID_WHEEL_SWEEP:
        return switch_is_mid(right) ? SYSID_MODE_WHEEL_SWEEP : SYSID_MODE_NONE;
    case SYSID_WHEEL_CURRENT_RAMP:
        return switch_is_mid(right) ? SYSID_MODE_WHEEL_CURRENT_RAMP : SYSID_MODE_NONE;
    }
    return SYSID_MODE_NONE;
}

static float TrialParameter(SysIdMode mode)
{
    switch (mode)
    {
    case SYSID_MODE_STATIC_HOLD:
    case SYSID_MODE_STATIC_GRID:
        return 1.0f;
    case SYSID_MODE_HIP_IN_PHASE:
    case SYSID_MODE_HIP_ANTI_PHASE:
        return HIP_SWEEP_AMPLITUDE_RAD;
    case SYSID_MODE_HIP_STEP:
        return HIP_STEP_RAD;
    case SYSID_MODE_HIP_RAMP:
        return HIP_RAMP_NM_S;
    case SYSID_MODE_HIP_TRIANGLE_02:
    case SYSID_MODE_HIP_TRIANGLE_05:
    case SYSID_MODE_HIP_TRIANGLE_10:
        return kTriangleSpeedRadS[mode - SYSID_MODE_HIP_TRIANGLE_02];
    case SYSID_MODE_WHEEL_STEP_10:
    case SYSID_MODE_WHEEL_STEP_30:
    case SYSID_MODE_WHEEL_STEP_60:
        return kWheelStepRadS[mode - SYSID_MODE_WHEEL_STEP_10];
    case SYSID_MODE_WHEEL_SWEEP:
        return WHEEL_SWEEP_RAD_S;
    case SYSID_MODE_WHEEL_CURRENT_RAMP:
        return WHEEL_RAMP_A_S;
    case SYSID_MODE_NONE:
    case SYSID_MODE_LEG_LENGTH:
        return 0.0f;
    }
    return 0.0f;
}

static int JointPhase(unsigned joint)
{
    static const int signs[][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
    const int axis = signs[robot.hip_pattern][joint % 2u];
    return joint >= 2u && robot.mode == SYSID_MODE_HIP_ANTI_PHASE ? -axis : axis;
}

// 台架只在启动时检查目标轨迹的相对角行程；整腿转动不改变这个角差。
static bool SweepFitsTravel(unsigned side, float amplitude)
{
    const unsigned front = 2u * side;
    const ChassisLeg *leg = &robot.chassis->legs[side];
    const float excursion = amplitude * (JointPhase(front) - JointPhase(front + 1u));
    const float lower = length_pid_config.minimum_m + HIP_SWEEP_STOP_MARGIN_M;
    const float upper = length_pid_config.maximum_m - HIP_SWEEP_STOP_MARGIN_M;
    const float zero_speed[2] = {0};
    if (!leg->geometry_valid || !(leg->length_m > lower && leg->length_m < upper))
        return false;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        const float q[2] = {leg->hip_delta_rad + sign * excursion, 0.0f};
        LegLengthState endpoint;
        if (!LegLengthGeometry(&length_pid_config, q, zero_speed, &endpoint) ||
            !(endpoint.length_m > lower && endpoint.length_m < upper))
            return false;
    }
    return true;
}

static bool TrialCanStart(float parameter)
{
    if (!(parameter > 0.0f) || (SYSID_EXPERIMENT == SYSID_HIP_TRIANGLE && !(HIP_TRIANGLE_RAD > 0.0f)))
        return false;

    if (SYSID_EXPERIMENT == SYSID_HIP_RAMP &&
        (HIP_RAMP_MAX_NM <= 0.0f || (SYSID_REAL_HARDWARE && HIP_MOTION_RAD_S <= 0.0f)))
        return false;
    if (SYSID_EXPERIMENT == SYSID_WHEEL_CURRENT_RAMP &&
        (WHEEL_RAMP_MAX_A <= 0.0f || (SYSID_REAL_HARDWARE && WHEEL_MOTION_RAD_S <= 0.0f)))
        return false;

    if (!SYSID_IS_HIP(SYSID_EXPERIMENT))
        return true;

    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(SYSID_JOINT_MASK & (1u << i)))
            continue;
        if (!SYSID_REAL_HARDWARE)
        {
            robot.hip_workpoint_rad[i] = chassis_init_config.hip[i].center_rad;
            continue;
        }
        if (!robot.chassis)
            return false;
        const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
        if (!fb->valid || !isfinite(fb->q_rad))
            return false;
        robot.hip_workpoint_rad[i] = fb->q_rad;
    }
    if (SYSID_REAL_HARDWARE && SYSID_EXPERIMENT == SYSID_HIP && SYSID_JOINT_MASK != kAllJoints)
    {
        const unsigned side = SYSID_TEST_SIDES == 1u ? CHASSIS_LEFT : CHASSIS_RIGHT;
        const ChassisLeg *leg = &robot.chassis->legs[side];
        const float length = HIP_SWEEP_WORK_LENGTH_M;
        const float a = length_pid_config.first_m, b = length_pid_config.second_m;
        const float target_delta = -2.0f * acosf((length*length + a*a - b*b) / (2.0f * length * a));
        if (!leg->geometry_valid || !isfinite(target_delta) || !isfinite(leg->hip_delta_rad) ||
            leg->hip_delta_rad >= 0.0f || leg->hip_delta_rad <= -PI ||
            !(leg->length_m >= length_pid_config.minimum_m - HIP_SWEEP_STOP_MARGIN_M &&
              leg->length_m <= length_pid_config.maximum_m + HIP_SWEEP_STOP_MARGIN_M))
            return false;
        sweep_half_delta_change = 0.5f * (target_delta - leg->hip_delta_rad);
        sweep_hold_bias_nm[0] = sweep_hold_bias_nm[1] = 0.0f;
        return true;
    }
    if (SYSID_REAL_HARDWARE)
        for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
        {
            const float span = SYSID_EXPERIMENT == SYSID_HIP_TRIANGLE ? HIP_TRIANGLE_RAD :
                               SYSID_EXPERIMENT == SYSID_HIP_STATIC || SYSID_EXPERIMENT == SYSID_HIP_RAMP ?
                                   0.0f : parameter;
            if ((SYSID_TEST_SIDES & (1u << side)) && !SweepFitsTravel(side, span))
                return false;
        }
    return true;
}

static void BuildHipRequest(ChassisRequest *request, float position)
{
    request->enabled = SYSID_REAL_HARDWARE;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        if (SYSID_JOINT_MASK & (1u << i))
            request->hip_position_rad[i] = robot.hip_workpoint_rad[i] + JointPhase(i) * position;
}

static void PositionSingleLeg(ChassisRequest *request, uint64_t now_us, float amplitude)
{
    const unsigned side = SYSID_TEST_SIDES == 1u ? CHASSIS_LEFT : CHASSIS_RIGHT;
    const unsigned first = 2u * side;
    const ChassisLeg *leg = &robot.chassis->legs[side];
    const uint64_t elapsed = now_us - start_us;
    if (!leg->geometry_valid || !leg->joint[0].valid || !leg->joint[1].valid ||
        elapsed >= HIP_SWEEP_POSITION_TIMEOUT_US)
    {
        robot.state = SYSID_STATE_FAULT;
        return;
    }
    float u = fminf((float)elapsed * 1e-6f / HIP_SWEEP_POSITIONING_S, 1.0f);
    const bool at_end = u >= 1.0f;
    u = u * u * (3.0f - 2.0f * u);
    const float dt = fminf((float)(now_us - sweep_bias_time_us) * 1e-6f, 2.0f / SYSID_CONTROL_HZ);
    sweep_bias_time_us = now_us;
    bool settled = at_end && fabsf(leg->length_m - HIP_SWEEP_WORK_LENGTH_M) <= 0.007f &&
                   !robot.chassis->clipped;
    for (unsigned j = 0; j < 2u; ++j)
    {
        const float change = j == CHASSIS_FRONT ? sweep_half_delta_change : -sweep_half_delta_change;
        const float ref = robot.hip_workpoint_rad[first + j] + u * change;
        const ChassisJointFeedback *fb = &leg->joint[j];
        const float error = remainderf(ref - fb->q_rad, 2.0f * PI);
        const float limit = chassis_init_config.hip[first + j].max_torque_nm;
        sweep_hold_bias_nm[j] = float_constrain(sweep_hold_bias_nm[j] + HIP_SWEEP_BIAS_GAIN * dt * error,
                                                -limit, limit);
        request->hip_position_rad[first + j] = ref;
        request->hip_torque_nm[first + j] = sweep_hold_bias_nm[j];
        settled = settled && fabsf(fb->qd_rad_s) < HIP_MOTION_RAD_S;
    }
    request->enabled = true;
    if (!settled)
        stable_since_us = 0;
    else if (!stable_since_us)
        stable_since_us = now_us;
    else if (now_us - stable_since_us >= 250000u && SweepFitsTravel(side, amplitude * HIP_SWEEP_HIGH_GAIN))
    {
        robot.hip_workpoint_rad[first] = request->hip_position_rad[first];
        robot.hip_workpoint_rad[first + 1u] = request->hip_position_rad[first + 1u];
        robot.phase = SYSID_PHASE_BASELINE;
        start_us = now_us;
    }
}

static void BuildWheelRequest(ChassisRequest *request, float reference, bool current_mode)
{
    request->enabled = SYSID_REAL_HARDWARE;
    request->wheel_current_mode = current_mode;
    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
        if (SYSID_WHEEL_MASK & (1u << side))
            request->wheel_reference[side] = reference;
}

static void RunLegLength(ChassisRequest *request, float dial)
{
    robot.length_target_m = LegLengthTarget(dial, kRemoteDialFullScale, &length_pid_config);
    if (!SYSID_REAL_HARDWARE)
        return;

    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        const unsigned first = 2u * side;
        if (!(SYSID_JOINT_MASK & (3u << first)))
            continue;

        const ChassisLeg *leg = &robot.chassis->legs[side];
        LegLengthState *state = &length_state[side];
        // 腿长和长度雅可比只依赖角差，使用底盘对齐过的装配分支。
        const float q[2] = {leg->hip_delta_rad, 0.0f};
        const float qd[2] = {leg->joint[CHASSIS_FRONT].qd_rad_s, leg->joint[CHASSIS_REAR].qd_rad_s};
        if (!leg->geometry_valid || !LegLengthGeometry(&length_pid_config, q, qd, state) ||
            !LegLengthStep(&length_pid_config, robot.length_target_m, state))
        {
            robot.state = SYSID_STATE_FAULT;
            return;
        }
        request->hip_torque_nm[first] = state->torque_nm[CHASSIS_FRONT];
        request->hip_torque_nm[first + 1u] = state->torque_nm[CHASSIS_REAR];
    }
    request->hip_torque_mask = SYSID_JOINT_MASK;
    request->enabled = true;
}

static void RecordHoldingTorque(uint64_t now_us)
{
    if (!SYSID_REAL_HARDWARE)
        return;
    const unsigned mask = SYSID_JOINT_MASK;
    bool stable = robot.chassis->enabled && !robot.chassis->clipped;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(mask & (1u << i)))
            continue;
        const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
        if (!fb->valid || fabsf(fb->qd_rad_s) >= HIP_MOTION_RAD_S)
            stable = false;
    }
    if (!stable)
    {
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
            hold_sum_nm[i] = 0.0f;
        hold_samples = 0;
        stable_since_us = 0;
        return;
    }
    if (!stable_since_us)
        stable_since_us = now_us;
    ++hold_samples;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        if (mask & (1u << i))
            hold_sum_nm[i] += robot.chassis->legs[i / 2u].joint[i % 2u].command_torque_nm;
}

static bool MotionDetected(bool hip, uint64_t now_us)
{
    if (!SYSID_REAL_HARDWARE)
        return false;
    unsigned observed = 0;
    if (hip)
    {
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        {
            if (!(SYSID_JOINT_MASK & (1u << i)) || !JointPhase(i))
                continue;
            const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
            if (fb->qd_rad_s * JointPhase(i) * SYSID_TRIAL_DIRECTION >= HIP_MOTION_RAD_S)
                observed |= 1u << i;
        }
    }
    else
    {
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
            if ((SYSID_WHEEL_MASK & (1u << i)) &&
                robot.chassis->legs[i].wheel_feedback.speed_rad_s * SYSID_TRIAL_DIRECTION >= WHEEL_MOTION_RAD_S)
                observed |= 1u << i;
    }
    if (!observed || !(moving_mask & observed))
    {
        moving_mask = observed;
        moving_since_us = now_us;
        return false;
    }
    moving_mask &= observed;
    return (float)(now_us - moving_since_us) * 1.0e-6f >= MOTION_CONFIRMATION_S;
}

static void RunRamp(ChassisRequest *request, float parameter, float elapsed_s, uint64_t now_us)
{
    const bool hip = SYSID_EXPERIMENT == SYSID_HIP_RAMP;
    const float ramp_s = elapsed_s - STEP_BASELINE_S;
    if (ramp_s < 0.0f)
    {
        robot.phase = SYSID_PHASE_BASELINE;
        if (hip)
        {
            BuildHipRequest(request, 0.0f);
            // 用启动结束前的稳定时间窗，配平力矩来自限幅后的有效保持指令。
            if (-ramp_s <= 2.0f * HIP_HOLD_AVERAGE_S)
                RecordHoldingTorque(now_us);
        }
        else
            BuildWheelRequest(request, 0.0f, true);
        return;
    }

    if (robot.phase == SYSID_PHASE_BASELINE && hip && SYSID_REAL_HARDWARE)
    {
        RecordHoldingTorque(now_us);
        if (!hold_samples || (float)(now_us - stable_since_us) * 1.0e-6f < HIP_HOLD_AVERAGE_S)
        {
            robot.state = SYSID_STATE_FAULT;
            return;
        }
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
            hold_sum_nm[i] /= hold_samples;
    }
    robot.phase = SYSID_PHASE_EXCITE;
    if (MotionDetected(hip, now_us))
    {
        robot.phase = SYSID_PHASE_MOTION_DETECTED;
        robot.state = SYSID_STATE_FINISHED;
        return;
    }

    const float magnitude = parameter * ramp_s;
    if (magnitude > (hip ? HIP_RAMP_MAX_NM : WHEEL_RAMP_MAX_A))
    {
        robot.state = SYSID_STATE_FINISHED;
        return;
    }
    if (!hip)
    {
        BuildWheelRequest(request, SYSID_TRIAL_DIRECTION * magnitude, true);
        return;
    }
    BuildHipRequest(request, 0.0f);
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(SYSID_JOINT_MASK & (1u << i)) || !JointPhase(i))
            continue;
        request->hip_torque_mask |= 1u << i;
        request->hip_torque_nm[i] = hold_sum_nm[i] + JointPhase(i) * SYSID_TRIAL_DIRECTION * magnitude;
    }
}

// Each of 100 poses has two 2 s holds, approached from opposite relative-angle directions.
static void GridPose(unsigned side, unsigned visit, bool approach, float target[2])
{
    const unsigned row = visit / 20u;
    unsigned column = (visit / 2u) % 10u;
    if (row % 2u)
        column = 9u - column;
    const float length = HIP_GRID_SHORT_M + (HIP_GRID_LONG_M - HIP_GRID_SHORT_M) * (float)row / 9.0f;
    const float first = chassis_init_config.geometry.first_m, second = chassis_init_config.geometry.second_m;
    const float delta = -2.0f * acosf((length * length + first * first - second * second) / (2.0f * length * first));
    const float *center = &robot.hip_workpoint_rad[2u * side];
    const float branch = chassis_init_config.hip[2u * side].center_rad - chassis_init_config.hip[2u * side+1u].center_rad;
    const float start_delta = branch + remainderf(center[0] - center[1] - branch, 2.0f * PI);
    const float swing = HIP_GRID_SWING_RAD * (2.0f * (float)column / 9.0f - 1.0f);
    const float preload = approach ? (visit % 2u ? -0.02f : 0.02f) : 0.0f;
    target[0] = center[0] + swing + 0.5f * (delta - start_delta) + preload;
    target[1] = center[1] + swing - 0.5f * (delta - start_delta) - preload;
}

static void RunStatic(ChassisRequest *request, float elapsed_s)
{
    if (robot.mode == SYSID_MODE_STATIC_HOLD)
    {
        robot.phase = SYSID_PHASE_BASELINE;
        BuildHipRequest(request, 0.0f);
        return;
    }
    const unsigned visit = (unsigned)(elapsed_s / 5.0f);
    if (visit >= 200u)
    {
        robot.state = SYSID_STATE_FINISHED;
        return;
    }
    const float within = elapsed_s - 5.0f * (float)visit;
    robot.phase = within < 3.0f ? SYSID_PHASE_EXCITE : SYSID_PHASE_BASELINE;
    request->enabled = SYSID_REAL_HARDWARE;
    for (unsigned s = 0; s < CHASSIS_LEG_COUNT; ++s)
    {
        if (!(SYSID_TEST_SIDES & (1u << s)))
            continue;
        float from[2], to[2];
        if (within >= 2.0f)
            GridPose(s, visit, true, from);
        else if (visit)
            GridPose(s, visit - 1u, false, from);
        else
        {
            from[0] = robot.hip_workpoint_rad[2u*s];
            from[1] = robot.hip_workpoint_rad[2u*s+1u];
        }
        GridPose(s, visit, within < 2.0f, to);
        float u = within < 2.0f ? within * 0.5f : fminf(within - 2.0f, 1.0f);
        u = u*u*(3.0f - 2.0f*u);
        for (unsigned j = 0; j < 2u; ++j)
            request->hip_position_rad[2u*s+j] = from[j] + u*(to[j] - from[j]);
    }
}

static void RunTrajectory(ChassisRequest *request, float parameter, uint64_t now_us)
{
    const float elapsed_s = (float)(now_us - start_us) * 1.0e-6f;
    switch (SYSID_EXPERIMENT)
    {
    case SYSID_HIP_STATIC:
        RunStatic(request, elapsed_s);
        return;
    case SYSID_HIP_RAMP:
    case SYSID_WHEEL_CURRENT_RAMP:
        RunRamp(request, parameter, elapsed_s, now_us);
        return;
    case SYSID_HIP:
    case SYSID_WHEEL_SWEEP: {
        float sweep_s = elapsed_s;
        if (SYSID_EXPERIMENT == SYSID_HIP)
        {
            if (SYSID_REAL_HARDWARE && SYSID_JOINT_MASK != kAllJoints)
            {
                if (robot.phase == SYSID_PHASE_POSITIONING)
                {
                    PositionSingleLeg(request, now_us, parameter);
                    return;
                }
                const unsigned first = SYSID_TEST_SIDES == 1u ? CHASSIS_LEFT_FRONT : CHASSIS_RIGHT_FRONT;
                request->hip_torque_nm[first] = sweep_hold_bias_nm[0];
                request->hip_torque_nm[first + 1u] = sweep_hold_bias_nm[1];
            }
            sweep_s -= HIP_SWEEP_HOLD_S;
            if (sweep_s >= SWEEP_DURATION_S + HIP_SWEEP_HOLD_S)
                break;
            if (sweep_s < 0.0f || sweep_s >= SWEEP_DURATION_S)
            {
                robot.phase = sweep_s < 0.0f ? SYSID_PHASE_BASELINE : SYSID_PHASE_RECOVERY;
                BuildHipRequest(request, 0.0f);
                return;
            }
        }
        else if (sweep_s >= SWEEP_DURATION_S)
            break;
        robot.phase = SYSID_PHASE_EXCITE;
        const float f_rate_hz_s = (SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_DURATION_S;
        const float phase_rad = 2.0f * PI * sweep_s * (SWEEP_START_HZ + 0.5f * f_rate_hz_s * sweep_s);
        const float edge_s = fminf(sweep_s, SWEEP_DURATION_S - sweep_s);
        float amplitude = parameter;
        if (SYSID_EXPERIMENT == SYSID_HIP && SYSID_JOINT_MASK != kAllJoints)
        {
            const float hz = SWEEP_START_HZ + f_rate_hz_s * sweep_s;
            const float u = float_constrain((hz - HIP_SWEEP_BOOST_START_HZ) /
                                            (SWEEP_END_HZ - HIP_SWEEP_BOOST_START_HZ), 0.0f, 1.0f);
            amplitude *= 1.0f + (HIP_SWEEP_HIGH_GAIN - 1.0f) * u * u * (3.0f - 2.0f * u);
        }
        const float reference =
            SYSID_TRIAL_DIRECTION * amplitude * float_constrain(edge_s / SWEEP_EDGE_S, 0.0f, 1.0f) * sinf(phase_rad);
        if (SYSID_EXPERIMENT == SYSID_HIP)
            BuildHipRequest(request, reference);
        else
            BuildWheelRequest(request, reference, false);
        return;
    }
    case SYSID_HIP_TRIANGLE: {
        const float peak = HIP_TRIANGLE_RAD;
        const float cycle_rad = 4.0f * peak;
        const float distance_rad = parameter * (elapsed_s - STEP_BASELINE_S);
        if (distance_rad >= TRIANGLE_CYCLES * cycle_rad)
            break;
        robot.phase = distance_rad < 0.0f ? SYSID_PHASE_BASELINE : SYSID_PHASE_EXCITE;
        const float within_cycle = fmodf(fmaxf(0.0f, distance_rad) + peak, cycle_rad);
        const float offset = within_cycle <= 2.0f * peak ? within_cycle - peak : 3.0f * peak - within_cycle;
        BuildHipRequest(request, SYSID_TRIAL_DIRECTION * offset);
        return;
    }
    case SYSID_HIP_STEP:
    case SYSID_WHEEL_STEP:
    case SYSID_WHEEL_STEADY:
    case SYSID_WHEEL_COAST: {
        const float hold_s =
            SYSID_EXPERIMENT == SYSID_HIP_STEP || SYSID_EXPERIMENT == SYSID_WHEEL_STEP ? STEP_HOLD_S : WHEEL_STEADY_S;
        const bool coast = SYSID_EXPERIMENT == SYSID_WHEEL_COAST;
        const float hold_end_s = STEP_BASELINE_S + hold_s;
        if (elapsed_s >= hold_end_s + (coast ? WHEEL_COAST_S : STEP_RECOVERY_S))
            break;
        const bool moving = elapsed_s >= STEP_BASELINE_S && elapsed_s < hold_end_s;
        robot.phase = moving                        ? SYSID_PHASE_EXCITE
                      : elapsed_s < STEP_BASELINE_S ? SYSID_PHASE_BASELINE
                                                    : SYSID_PHASE_RECOVERY;
        const float reference = moving ? SYSID_TRIAL_DIRECTION * parameter : 0.0f;
        if (SYSID_EXPERIMENT == SYSID_HIP_STEP)
            BuildHipRequest(request, reference);
        else
            BuildWheelRequest(request, reference, coast && robot.phase == SYSID_PHASE_RECOVERY);
        return;
    }
    }
    robot.state = SYSID_STATE_FINISHED;
}

static void RobotCMDTask(ChassisRequest *request, uint8_t left, SysIdMode selected, float dial, uint64_t now_us)
{
    const bool single_leg_sweep = SYSID_EXPERIMENT == SYSID_HIP && SYSID_JOINT_MASK != kAllJoints;
    if (single_leg_sweep)
    {
        // 离开双上就停止并准备下一次。上电或重连时已在双上，须先退出一次。
        if (!switch_is_up(left) || selected != SYSID_MODE_HIP_IN_PHASE)
        {
            robot.state = SYSID_STATE_READY;
            robot.mode = SYSID_MODE_NONE;
            robot.phase = SYSID_PHASE_NONE;
            return;
        }
        if (robot.state == SYSID_STATE_READY)
            robot.mode = selected;
    }
    else if (switch_is_down(left))
    {
        robot.state = SYSID_STATE_ESTOP;
        robot.mode = SYSID_MODE_NONE;
        robot.phase = SYSID_PHASE_NONE;
        return;
    }

    else if (switch_is_mid(left) && (robot.state == SYSID_STATE_ESTOP || robot.state == SYSID_STATE_READY))
    {
        robot.state = SYSID_STATE_READY;
        robot.mode = selected;
        return;
    }

    if (robot.state != SYSID_STATE_READY && robot.state != SYSID_STATE_RUNNING &&
        robot.state != SYSID_STATE_DRY_RUNNING)
        return;

    if (!switch_is_up(left) || selected == SYSID_MODE_NONE || selected != robot.mode)
    {
        robot.state = SYSID_STATE_NEED_ESTOP;
        robot.phase = SYSID_PHASE_NONE;
        return;
    }

    const bool length_mode = SYSID_EXPERIMENT == SYSID_HIP && selected == SYSID_MODE_LEG_LENGTH;
    const float parameter = TrialParameter(selected);
    if (robot.state == SYSID_STATE_READY)
    {
        robot.hip_pattern = SYSID_HIP_PATTERN;
        if (SYSID_EXPERIMENT == SYSID_HIP && !single_leg_sweep && !length_mode)
        {
            if (dial < -0.5f * kRemoteDialFullScale)
                robot.hip_pattern = SYSID_HIP_BOTH_IN_PHASE;
            else if (dial > 0.5f * kRemoteDialFullScale)
                robot.hip_pattern = SYSID_HIP_BOTH_ANTI_PHASE;
        }
        if (!length_mode && !TrialCanStart(parameter))
        {
            robot.state = SYSID_STATE_FAULT;
            return;
        }
        start_us = now_us;
        sweep_bias_time_us = now_us;
        robot.phase = single_leg_sweep && SYSID_REAL_HARDWARE ? SYSID_PHASE_POSITIONING : SYSID_PHASE_NONE;
        hold_samples = moving_mask = 0u;
        stable_since_us = moving_since_us = 0u;
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
            hold_sum_nm[i] = 0.0f;
        if (length_mode && SYSID_REAL_HARDWARE)
            for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
                PIDClear(&length_state[side].pid);
        robot.state = SYSID_REAL_HARDWARE ? SYSID_STATE_RUNNING : SYSID_STATE_DRY_RUNNING;
    }
    if (length_mode)
        RunLegLength(request, dial);
    else
        RunTrajectory(request, parameter, now_us);
}

void RobotInit(void)
{
    robot.rc_data = RemoteControlInit(SYSID_RC_PORT);
    if (!VofaInit())
        LOGERROR("[motor_sysid] VOFA init failed");
    if (!SYSID_REAL_HARDWARE)
        return;

    if (!WheelsConfigured())
    {
        LOGERROR("[motor_sysid] Check wheel configuration");
        return;
    }

    robot.chassis = ChassisInit(&chassis_init_config);
    if (!robot.chassis)
    {
        LOGERROR("[motor_sysid] Chassis init failed: check CAN filter slots");
        return;
    }
    if (SYSID_IS_HIP(SYSID_EXPERIMENT))
    {
        const ChassisGeometryConfig *geometry = &chassis_init_config.geometry;
        length_pid_config.first_m = geometry->first_m;
        length_pid_config.second_m = geometry->second_m;
        length_pid_config.virtual_second_m = geometry->virtual_second_m;
        length_pid_config.epsilon = geometry->singular_epsilon_m;
        for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
            PIDInit(&length_state[side].pid, &length_pid_config.pid);
    }
}

RobotInstance *RobotGet(void)
{
    return &robot;
}

void RobotTask(void)
{
    // 扫频和 VOFA 在队内 RobotTask 上按 2 tick 采样，错过的采样不补发。
    _Static_assert(configTICK_RATE_HZ % SYSID_CONTROL_HZ == 0, "SysID period must align with RTOS ticks");
    static TickType_t last_tick;
    const TickType_t period = configTICK_RATE_HZ / SYSID_CONTROL_HZ;
    const TickType_t tick = xTaskGetTickCount();
    const TickType_t elapsed = tick - last_tick;
    if (elapsed < period)
        return;
    last_tick = tick - (elapsed % period);
    const uint64_t now_us = DWT_GetTimeline_us();
    ChassisRequest command = {0};
    if (robot.chassis)
        ChassisFeedbackUpdate(robot.chassis);

    taskENTER_CRITICAL();
    const bool online = RemoteControlIsOnline();
    const uint8_t left = online ? robot.rc_data[TEMP].rc.switch_left : RC_SW_OFF;
    const uint8_t right = online ? robot.rc_data[TEMP].rc.switch_right : RC_SW_OFF;
    const float dial = online ? (float)robot.rc_data[TEMP].rc.dial : 0.0f;
    taskEXIT_CRITICAL();

    if (SYSID_REAL_HARDWARE && !robot.chassis)
        robot.state = SYSID_STATE_FAULT;
    else if (!online || switch_is_off(left) || switch_is_off(right))
    {
        robot.state = SYSID_STATE_NEED_ESTOP;
        robot.phase = SYSID_PHASE_NONE;
    }
    else
        RobotCMDTask(&command, left, SelectMode(right), dial, now_us);

    if (robot.chassis)
    {
        ChassisTask(robot.chassis, &command);
        if (command.enabled && !robot.chassis->enabled)
            robot.state = SYSID_STATE_FAULT;
    }
    VofaTick(&command, now_us, online, SYSID_JOINT_MASK, SYSID_EXPERIMENT, robot.hip_pattern, SYSID_TRIAL_DIRECTION);
}
