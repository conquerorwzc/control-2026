#include "robot.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "task.h"
#include "user_lib.h"
#include "vofa_watch.h"

static RobotInstance robot;
static LegLengthState length_state[CHASSIS_LEG_COUNT];
static float hold_sum_nm[CHASSIS_JOINT_COUNT];
static unsigned hold_samples;
static unsigned moving_mask;
static uint64_t stable_since_us;
static uint64_t moving_since_us;
static uint64_t start_us;
static const unsigned kAllJoints = (1u << CHASSIS_JOINT_COUNT) - 1u;
_Static_assert(SYSID_TEST_SIDES >= 1u && SYSID_TEST_SIDES <= 3u, "Select left, right or both legs");
_Static_assert(SYSID_TRIAL_DIRECTION == 1 || SYSID_TRIAL_DIRECTION == -1, "Select a trial direction");
_Static_assert(SYSID_EXPERIMENT <= SYSID_WHEEL_COAST && SYSID_HIP_PATTERN <= SYSID_HIP_BOTH_ANTI_PHASE,
               "Select a valid trial and hip pattern");

static bool HardwareConfigured(void)
{
    const Chassis_Init_Config_s *c = &chassis_init_config;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(c->joint_mask & (1u << i)))
            continue;
        // 缺少标定的轴不能注册。DMMotorInit 会发送硬件使能。
        if (!isfinite(c->hip[i].zero_rad) || fabsf(c->hip[i].direction) != 1.0f || !isfinite(c->hip[i].center_rad) ||
            !(c->hip[i].max_torque_nm > 0.0f))
            return false;
    }
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
    const int axis = signs[SYSID_HIP_PATTERN][joint % 2u];
    return joint >= 2u && robot.mode == SYSID_MODE_HIP_ANTI_PHASE ? -axis : axis;
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

    if (!robot.chassis || !SYSID_IS_HIP(SYSID_EXPERIMENT))
        return true;
    if (!(kHipStartToleranceRad > 0.0f))
        return false;

    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(SYSID_JOINT_MASK & (1u << i)))
            continue;
        const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
        const float error = remainderf(fb->q_rad - chassis_init_config.hip[i].center_rad, 2.0f * PI);
        if (!fb->valid || !(fabsf(error) <= kHipStartToleranceRad))
            return false;
    }
    return true;
}

static void BuildHipRequest(ChassisRequest *request, float position)
{
    request->enabled = SYSID_REAL_HARDWARE;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        if (SYSID_JOINT_MASK & (1u << i))
        {
            const float center = chassis_init_config.hip[i].center_rad;
            // 空跑可预览还没有标定中心的轴。
            request->hip_position_rad[i] =
                (!SYSID_REAL_HARDWARE && !isfinite(center) ? 0.0f : center) + JointPhase(i) * position;
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
        const float error = remainderf(fb->q_rad - chassis_init_config.hip[i].center_rad, 2.0f * PI);
        if (!(fabsf(error) <= kHipStartToleranceRad) || fabsf(fb->qd_rad_s) >= HIP_MOTION_RAD_S)
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

static void RunTrajectory(ChassisRequest *request, float parameter, uint64_t now_us)
{
    const float elapsed_s = (float)(now_us - start_us) * 1.0e-6f;
    switch (SYSID_EXPERIMENT)
    {
    case SYSID_HIP_RAMP:
    case SYSID_WHEEL_CURRENT_RAMP:
        RunRamp(request, parameter, elapsed_s, now_us);
        return;
    case SYSID_HIP:
    case SYSID_WHEEL_SWEEP: {
        if (elapsed_s >= SWEEP_DURATION_S)
            break;
        robot.phase = SYSID_PHASE_EXCITE;
        const float f_rate_hz_s = (SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_DURATION_S;
        const float phase_rad = 2.0f * PI * elapsed_s * (SWEEP_START_HZ + 0.5f * f_rate_hz_s * elapsed_s);
        const float edge_s = fminf(elapsed_s, SWEEP_DURATION_S - elapsed_s);
        const float reference =
            SYSID_TRIAL_DIRECTION * parameter * float_constrain(edge_s / SWEEP_EDGE_S, 0.0f, 1.0f) * sinf(phase_rad);
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
    if (switch_is_down(left))
    {
        robot.state = SYSID_STATE_ESTOP;
        robot.mode = SYSID_MODE_NONE;
        robot.phase = SYSID_PHASE_NONE;
        return;
    }

    if (switch_is_mid(left) && (robot.state == SYSID_STATE_ESTOP || robot.state == SYSID_STATE_READY))
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
        if (!length_mode && !TrialCanStart(parameter))
        {
            robot.state = SYSID_STATE_FAULT;
            return;
        }
        start_us = now_us;
        robot.phase = SYSID_PHASE_NONE;
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
    {
        LOGERROR("[motor_sysid] VOFA init failed");
        return;
    }
    if (SYSID_REAL_HARDWARE)
    {
        if (HardwareConfigured())
            robot.chassis = ChassisInit(&chassis_init_config);
        if (!robot.chassis)
            LOGERROR("[motor_sysid] Check motor configuration and CAN filter slots");
        if (SYSID_EXPERIMENT == SYSID_HIP && robot.chassis)
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
    VofaTick(&command, now_us, online, SYSID_JOINT_MASK, SYSID_EXPERIMENT, SYSID_HIP_PATTERN, SYSID_TRIAL_DIRECTION);
}
