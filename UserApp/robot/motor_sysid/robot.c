#include "robot.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "robot_config.h"
#include "task.h"
#include "vofa_watch.h"

static RobotInstance robot;
static const SysIdAmplitude *amplitude;
static float hold_sum_nm[CHASSIS_JOINT_COUNT];
static float hold_torque_nm[CHASSIS_JOINT_COUNT];
static unsigned hold_samples;
static unsigned moving_mask;
static uint64_t stable_since_us;
static uint64_t moving_since_us;
static const float kTwoPi = 6.28318530718f;
static const unsigned kAllJoints = (1u << CHASSIS_JOINT_COUNT) - 1u;
static const unsigned kAllWheels = (1u << CHASSIS_LEG_COUNT) - 1u;

static bool Positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static float WheelStepSpeed(SysIdMode mode)
{
    switch (mode)
    {
    case SYSID_MODE_WHEEL_STEP_10:
        return 10.0f;
    case SYSID_MODE_WHEEL_STEP_30:
        return 30.0f;
    case SYSID_MODE_WHEEL_STEP_60:
        return 60.0f;
    default:
        return 0.0f;
    }
}

static int HipAxisPhase(unsigned axis)
{
    switch (robot.hip_pattern)
    {
    case SYSID_HIP_FRONT:
        return axis == CHASSIS_FRONT;
    case SYSID_HIP_REAR:
        return axis == CHASSIS_REAR;
    case SYSID_HIP_BOTH_IN_PHASE:
        return 1;
    case SYSID_HIP_BOTH_ANTI_PHASE:
        return axis == CHASSIS_FRONT ? 1 : -1;
    }
    return 0;
}

static unsigned JointMask(void)
{
    return kSysIdChassisConfig.joint_mask;
}

static unsigned WheelMask(void)
{
    return kSysIdChassisConfig.wheel_mask;
}

static bool ValidTrialConfig(void)
{
    if ((unsigned)robot.trial > SYSID_WHEEL_COAST || (robot.direction != 1 && robot.direction != -1) ||
        (SYSID_IS_HIP(robot.trial) && (unsigned)robot.hip_pattern > SYSID_HIP_BOTH_ANTI_PHASE))
        return false;

    switch (robot.trial)
    {
    case SYSID_HIP:
        if (!Positive(amplitude->hip_sweep_rad))
            return false;
        break;
    case SYSID_HIP_STEP:
        if (!Positive(amplitude->hip_step_rad))
            return false;
        break;
    case SYSID_HIP_TRIANGLE:
        if (!Positive(amplitude->triangle_rad) || !kTrialTiming.triangle_cycles)
            return false;
        break;
    case SYSID_HIP_RAMP:
        if (!Positive(amplitude->hip_ramp_nm_s) || !Positive(amplitude->hip_ramp_max_nm) ||
            (robot.real_hardware && !Positive(amplitude->hip_motion_rad_s)))
            return false;
        break;
    case SYSID_WHEEL_SWEEP:
        if (!Positive(amplitude->wheel_sweep_rad_s))
            return false;
        break;
    case SYSID_WHEEL_CURRENT_RAMP:
        if (!Positive(amplitude->wheel_ramp_a_s) || !Positive(amplitude->wheel_ramp_max_a) ||
            (robot.real_hardware && !Positive(amplitude->wheel_motion_rad_s)))
            return false;
        break;
    default:
        break;
    }

    if (robot.trial == SYSID_HIP || robot.trial == SYSID_WHEEL_SWEEP)
    {
        if (!Positive(kTrialTiming.sweep_start_hz) || !Positive(kTrialTiming.sweep_ramp_s) ||
            !(kTrialTiming.sweep_end_hz > kTrialTiming.sweep_start_hz &&
              kTrialTiming.sweep_end_hz < SYSID_CONTROL_HZ * 0.5f &&
              kTrialTiming.sweep_duration_s > 2.0f * kTrialTiming.sweep_ramp_s))
            return false;
    }
    if (robot.trial != SYSID_HIP && robot.trial != SYSID_WHEEL_SWEEP && !Positive(kTrialTiming.step_baseline_s))
        return false;
    if ((robot.trial == SYSID_HIP_RAMP || robot.trial == SYSID_WHEEL_CURRENT_RAMP) &&
        !Positive(kTrialTiming.motion_confirmation_s))
        return false;
    if (robot.trial == SYSID_HIP_RAMP &&
        (!Positive(kTrialTiming.hold_average_s) || kTrialTiming.hold_average_s >= kTrialTiming.step_baseline_s))
        return false;
    if ((robot.trial == SYSID_HIP_STEP || robot.trial == SYSID_WHEEL_STEP) &&
        (!Positive(kTrialTiming.step_hold_s) || !Positive(kTrialTiming.step_recovery_s)))
        return false;
    if (robot.trial == SYSID_WHEEL_STEADY &&
        (!Positive(kTrialTiming.steady_s) || !Positive(kTrialTiming.step_recovery_s)))
        return false;
    if (robot.trial == SYSID_WHEEL_COAST && (!Positive(kTrialTiming.steady_s) || !Positive(kTrialTiming.coast_s)))
        return false;
    if (!robot.real_hardware)
        return true;
    if (kMaxControlGapUs <= 1000000u / SYSID_CONTROL_HZ || !SYSID_TEST_SIDES || (SYSID_TEST_SIDES & ~kAllWheels))
        return false;
    if (SYSID_IS_HIP(robot.trial))
    {
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        {
            if (!(JointMask() & (1u << i)))
                continue;
            const SysIdHipLimits *limits = &kHipLimits[i];
            if (!Positive(kSysIdChassisConfig.joint[i].kp_nm_per_rad) || !isfinite(limits->min_rad) ||
                !isfinite(limits->max_rad) || !Positive(limits->start_error_rad) ||
                !Positive(limits->start_speed_rad_s) || !Positive(limits->max_speed_rad_s) ||
                limits->start_speed_rad_s > limits->max_speed_rad_s || !Positive(limits->max_feedback_torque_nm) ||
                !Positive(limits->max_temperature_c) || !Positive(limits->start_command_torque_nm) ||
                !(robot.center_rad[i] - limits->start_error_rad >= limits->min_rad &&
                  robot.center_rad[i] + limits->start_error_rad <= limits->max_rad))
                return false;
        }
    }
    else
    {
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
        {
            if (!(WheelMask() & (1u << i)))
                continue;
            const SysIdWheelLimits *limits = &kWheelLimits[i];
            if (!Positive(limits->max_speed_rad_s) || !Positive(limits->max_feedback_current_raw) ||
                !Positive(limits->max_temperature_c) || !Positive(limits->start_speed_rad_s) ||
                limits->start_speed_rad_s > limits->max_speed_rad_s ||
                (robot.trial != SYSID_WHEEL_CURRENT_RAMP && !Positive(kSysIdChassisConfig.wheel[i].kp_a_per_rad_s)))
                return false;
        }
    }
    return true;
}

static SysIdMode SelectMode(uint8_t right)
{
    switch (robot.trial)
    {
    case SYSID_HIP:
        if (right == RC_SW_UP)
            return SYSID_MODE_HIP_IN_PHASE;
        if (right == RC_SW_MID && JointMask() == kAllJoints)
            return SYSID_MODE_HIP_ANTI_PHASE;
        break;
    case SYSID_HIP_STEP:
        if (right == RC_SW_MID)
            return SYSID_MODE_HIP_STEP;
        break;
    case SYSID_HIP_RAMP:
        if (right == RC_SW_MID)
            return SYSID_MODE_HIP_RAMP;
        break;
    case SYSID_HIP_TRIANGLE:
        if (right == RC_SW_UP)
            return SYSID_MODE_HIP_TRIANGLE_02;
        if (right == RC_SW_MID)
            return SYSID_MODE_HIP_TRIANGLE_05;
        if (right == RC_SW_DOWN)
            return SYSID_MODE_HIP_TRIANGLE_10;
        break;
    case SYSID_WHEEL_STEP:
    case SYSID_WHEEL_STEADY:
    case SYSID_WHEEL_COAST:
        if (right == RC_SW_UP)
            return SYSID_MODE_WHEEL_STEP_10;
        if (right == RC_SW_MID)
            return SYSID_MODE_WHEEL_STEP_30;
        if (right == RC_SW_DOWN)
            return SYSID_MODE_WHEEL_STEP_60;
        break;
    case SYSID_WHEEL_SWEEP:
        if (right == RC_SW_MID)
            return SYSID_MODE_WHEEL_SWEEP;
        break;
    case SYSID_WHEEL_CURRENT_RAMP:
        if (right == RC_SW_MID)
            return SYSID_MODE_WHEEL_CURRENT_RAMP;
        break;
    }
    return SYSID_MODE_NONE;
}

static float TrialParameter(void)
{
    switch (robot.mode)
    {
    case SYSID_MODE_HIP_IN_PHASE:
    case SYSID_MODE_HIP_ANTI_PHASE:
        return amplitude->hip_sweep_rad;
    case SYSID_MODE_HIP_STEP:
        return amplitude->hip_step_rad;
    case SYSID_MODE_HIP_RAMP:
        return amplitude->hip_ramp_nm_s;
    case SYSID_MODE_HIP_TRIANGLE_02:
        return 0.2f;
    case SYSID_MODE_HIP_TRIANGLE_05:
        return 0.5f;
    case SYSID_MODE_HIP_TRIANGLE_10:
        return 1.0f;
    case SYSID_MODE_WHEEL_SWEEP:
        return amplitude->wheel_sweep_rad_s;
    case SYSID_MODE_WHEEL_CURRENT_RAMP:
        return amplitude->wheel_ramp_a_s;
    default:
        return WheelStepSpeed(robot.mode);
    }
}

static int JointPhase(unsigned i)
{
    const int between_legs = i >= 2u && robot.mode == SYSID_MODE_HIP_ANTI_PHASE ? -1 : 1;
    return between_legs * HipAxisPhase(i % 2u);
}

static bool ValidSelectedRun(void)
{
    if (!robot.real_hardware)
        return true;
    if (SYSID_IS_HIP(robot.trial))
    {
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        {
            if (!(JointMask() & (1u << i)) || !JointPhase(i))
                continue;
            if (robot.trial == SYSID_HIP_RAMP)
            {
                if (amplitude->hip_ramp_max_nm > kSysIdChassisConfig.joint[i].max_torque_nm)
                    return false;
                continue;
            }
            const float center = robot.center_rad[i];
            const float signed_step = JointPhase(i) * robot.direction * amplitude->hip_step_rad;
            const float travel = robot.trial == SYSID_HIP            ? amplitude->hip_sweep_rad
                                 : robot.trial == SYSID_HIP_TRIANGLE ? amplitude->triangle_rad
                                                                     : 0.0f;
            if (center - travel < kHipLimits[i].min_rad || center + travel > kHipLimits[i].max_rad ||
                (robot.trial == SYSID_HIP_STEP &&
                 (center + signed_step < kHipLimits[i].min_rad || center + signed_step > kHipLimits[i].max_rad)))
                return false;
        }
    }
    else
    {
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
        {
            if (!(WheelMask() & (1u << i)))
                continue;
            if (robot.trial == SYSID_WHEEL_CURRENT_RAMP
                    ? amplitude->wheel_ramp_max_a > kSysIdChassisConfig.wheel[i].max_current_a
                    : robot.selected_parameter > kWheelLimits[i].max_speed_rad_s)
                return false;
        }
    }
    return true;
}

static bool TrialFeedbackValid(bool starting)
{
    if (!robot.real_hardware)
        return true;

    for (unsigned side = 0; side < CHASSIS_LEG_COUNT; ++side)
    {
        const ChassisLeg *leg = &robot.chassis->legs[side];
        if (SYSID_IS_HIP(robot.trial))
        {
            for (unsigned joint = CHASSIS_FRONT; joint <= CHASSIS_REAR; ++joint)
            {
                const unsigned i = 2u * side + joint;
                if (!(kSysIdChassisConfig.joint_mask & (1u << i)))
                    continue;
                const ChassisJointFeedback *fb = &leg->joint[joint];
                const SysIdHipLimits *limits = &kHipLimits[i];
                if (!fb->valid || fb->q_rad < limits->min_rad || fb->q_rad > limits->max_rad ||
                    fabsf(fb->qd_rad_s) > limits->max_speed_rad_s ||
                    fabsf(fb->tau_nm) > limits->max_feedback_torque_nm || fb->temperature_c > limits->max_temperature_c)
                    return false;
                if (starting)
                {
                    const ChassisJointConfig *motor = &kSysIdChassisConfig.joint[i];
                    const float error = robot.center_rad[i] - fb->q_rad;
                    const float torque = motor->kp_nm_per_rad * error - motor->kd_nm_per_rad_s * fb->qd_rad_s;
                    if (fabsf(error) > limits->start_error_rad || fabsf(fb->qd_rad_s) > limits->start_speed_rad_s ||
                        !isfinite(torque) || fabsf(torque) > limits->start_command_torque_nm)
                        return false;
                }
            }
        }
        else if (kSysIdChassisConfig.wheel_mask & (1u << side))
        {
            const ChassisWheelFeedback *fb = &leg->wheel_feedback;
            const SysIdWheelLimits *limits = &kWheelLimits[side];
            if (!fb->valid || fabsf(fb->speed_rad_s) > limits->max_speed_rad_s ||
                fabsf(fb->current_raw) > limits->max_feedback_current_raw ||
                fb->temperature_c > limits->max_temperature_c ||
                (starting && fabsf(fb->speed_rad_s) > limits->start_speed_rad_s))
                return false;
        }
    }
    return true;
}

static void SetFault(SysIdFault fault)
{
    robot.state = SYSID_STATE_FAULT;
    robot.fault = fault;
    robot.phase = SYSID_PHASE_NONE;
}

static void BuildHipRequest(float position, bool enable)
{
    const unsigned mask = JointMask();
    robot.request.joint_enable_mask = enable ? mask : 0u;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        if (mask & (1u << i))
            robot.request.q_ref_rad[i] = robot.center_rad[i] + JointPhase(i) * position;
}

static void BuildWheelRequest(float reference, bool current_mode)
{
    const unsigned mask = WheelMask();
    robot.request.wheel_enable_mask = robot.real_hardware ? mask : 0u;
    robot.request.wheel_current_mode_mask = current_mode ? mask : 0u;
    for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
    {
        if (!(mask & (1u << i)))
            continue;
        if (current_mode)
            robot.request.wheel_current_ref_a[i] = reference;
        else
            robot.request.wheel_speed_ref_rad_s[i] = reference;
    }
}

static void RecordHoldingTorque(void)
{
    if (!robot.real_hardware)
        return;
    const unsigned mask = JointMask();
    bool stable = robot.chassis->applied_request.joint_enable_mask == mask && !robot.chassis->clipped;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(mask & (1u << i)))
            continue;
        const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
        if (fabsf(fb->q_rad - robot.center_rad[i]) > kHipLimits[i].start_error_rad ||
            fabsf(fb->qd_rad_s) >= amplitude->hip_motion_rad_s)
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
        stable_since_us = robot.now_us;
    ++hold_samples;
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        if (mask & (1u << i))
            hold_sum_nm[i] += robot.chassis->legs[i / 2u].joint[i % 2u].command_torque_nm;
}

static bool MotionDetected(bool hip)
{
    if (!robot.real_hardware)
        return false;
    unsigned observed = 0;
    if (hip)
    {
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        {
            if (!(JointMask() & (1u << i)) || !JointPhase(i))
                continue;
            const ChassisJointFeedback *fb = &robot.chassis->legs[i / 2u].joint[i % 2u];
            if (fb->qd_rad_s * JointPhase(i) * robot.direction >= amplitude->hip_motion_rad_s)
                observed |= 1u << i;
        }
    }
    else
    {
        for (unsigned i = 0; i < CHASSIS_LEG_COUNT; ++i)
            if ((WheelMask() & (1u << i)) &&
                robot.chassis->legs[i].wheel_feedback.speed_rad_s * robot.direction >= amplitude->wheel_motion_rad_s)
                observed |= 1u << i;
    }
    if (!observed || !(moving_mask & observed))
    {
        moving_mask = observed;
        moving_since_us = robot.now_us;
        return false;
    }
    moving_mask &= observed;
    return (float)(robot.now_us - moving_since_us) * 1.0e-6f >= kTrialTiming.motion_confirmation_s;
}

static void RunRamp(float elapsed_s, bool hip)
{
    const float ramp_s = elapsed_s - kTrialTiming.step_baseline_s;
    if (ramp_s < 0.0f)
    {
        robot.phase = SYSID_PHASE_BASELINE;
        if (hip)
        {
            BuildHipRequest(0.0f, robot.real_hardware);
            // 用启动结束前的稳定时间窗，配平力矩来自限幅后的有效保持指令。
            if (-ramp_s <= 2.0f * kTrialTiming.hold_average_s)
                RecordHoldingTorque();
        }
        else
            BuildWheelRequest(0.0f, true);
        return;
    }

    if (robot.phase == SYSID_PHASE_BASELINE && hip && robot.real_hardware)
    {
        RecordHoldingTorque();
        if (!hold_samples || (float)(robot.now_us - stable_since_us) * 1.0e-6f < kTrialTiming.hold_average_s)
        {
            SetFault(SYSID_FAULT_START);
            return;
        }
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
            hold_torque_nm[i] = hold_sum_nm[i] / hold_samples;
    }
    robot.phase = SYSID_PHASE_EXCITE;
    if (MotionDetected(hip))
    {
        robot.phase = SYSID_PHASE_MOTION_DETECTED;
        robot.state = SYSID_STATE_FINISHED;
        return;
    }

    const float magnitude = robot.selected_parameter * ramp_s;
    if (magnitude > (hip ? amplitude->hip_ramp_max_nm : amplitude->wheel_ramp_max_a))
    {
        robot.state = SYSID_STATE_FINISHED;
        return;
    }
    if (!hip)
    {
        BuildWheelRequest(robot.direction * magnitude, true);
        return;
    }
    BuildHipRequest(0.0f, robot.real_hardware);
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
    {
        if (!(JointMask() & (1u << i)) || !JointPhase(i))
            continue;
        const float effort = hold_torque_nm[i] + JointPhase(i) * robot.direction * magnitude;
        if (robot.real_hardware && fabsf(effort) > kSysIdChassisConfig.joint[i].max_torque_nm)
        {
            robot.request = (ChassisRequest){0};
            robot.state = SYSID_STATE_FINISHED;
            return;
        }
        robot.request.joint_direct_torque_mask |= 1u << i;
        robot.request.joint_torque_ref_nm[i] = hold_torque_nm[i];
        robot.request.joint_torque_offset_nm[i] = JointPhase(i) * robot.direction * magnitude;
    }
}

static void RunTrajectory(float elapsed_s)
{
    switch (robot.trial)
    {
    case SYSID_HIP_RAMP:
    case SYSID_WHEEL_CURRENT_RAMP:
        RunRamp(elapsed_s, robot.trial == SYSID_HIP_RAMP);
        return;
    case SYSID_HIP:
    case SYSID_WHEEL_SWEEP: {
        if (elapsed_s >= kTrialTiming.sweep_duration_s)
            break;
        robot.phase = SYSID_PHASE_EXCITE;
        const float f_rate_hz_s =
            (kTrialTiming.sweep_end_hz - kTrialTiming.sweep_start_hz) / kTrialTiming.sweep_duration_s;
        const float phase_rad = kTwoPi * elapsed_s * (kTrialTiming.sweep_start_hz + 0.5f * f_rate_hz_s * elapsed_s);
        const float edge_s = fminf(elapsed_s, kTrialTiming.sweep_duration_s - elapsed_s);
        const float reference = robot.direction * robot.selected_parameter *
                                fminf(1.0f, edge_s / kTrialTiming.sweep_ramp_s) * sinf(phase_rad);
        if (robot.trial == SYSID_HIP)
            BuildHipRequest(reference, robot.real_hardware);
        else
            BuildWheelRequest(reference, false);
        return;
    }
    case SYSID_HIP_TRIANGLE: {
        const float peak = amplitude->triangle_rad;
        const float cycle_rad = 4.0f * peak;
        const float distance_rad = robot.selected_parameter * (elapsed_s - kTrialTiming.step_baseline_s);
        if (distance_rad >= kTrialTiming.triangle_cycles * cycle_rad)
            break;
        robot.phase = distance_rad < 0.0f ? SYSID_PHASE_BASELINE : SYSID_PHASE_EXCITE;
        const float within_cycle = fmodf(fmaxf(0.0f, distance_rad) + peak, cycle_rad);
        const float offset = within_cycle <= 2.0f * peak ? within_cycle - peak : 3.0f * peak - within_cycle;
        BuildHipRequest(robot.direction * offset, robot.real_hardware);
        return;
    }
    case SYSID_HIP_STEP:
    case SYSID_WHEEL_STEP:
    case SYSID_WHEEL_STEADY:
    case SYSID_WHEEL_COAST: {
        const float hold_s = robot.trial == SYSID_HIP_STEP || robot.trial == SYSID_WHEEL_STEP ? kTrialTiming.step_hold_s
                                                                                              : kTrialTiming.steady_s;
        const bool coast = robot.trial == SYSID_WHEEL_COAST;
        const float hold_end_s = kTrialTiming.step_baseline_s + hold_s;
        if (elapsed_s >= hold_end_s + (coast ? kTrialTiming.coast_s : kTrialTiming.step_recovery_s))
            break;
        const bool moving = elapsed_s >= kTrialTiming.step_baseline_s && elapsed_s < hold_end_s;
        robot.phase = moving                                     ? SYSID_PHASE_EXCITE
                      : elapsed_s < kTrialTiming.step_baseline_s ? SYSID_PHASE_BASELINE
                                                                 : SYSID_PHASE_RECOVERY;
        const float reference = moving ? robot.direction * robot.selected_parameter : 0.0f;
        if (robot.trial == SYSID_HIP_STEP)
            BuildHipRequest(reference, robot.real_hardware);
        else
            BuildWheelRequest(reference, coast && robot.phase == SYSID_PHASE_RECOVERY);
        return;
    }
    }
    robot.state = SYSID_STATE_FINISHED;
}

static void RunTrial(uint8_t left, SysIdMode selected)
{
    if (left == RC_SW_DOWN)
    {
        robot.state = SYSID_STATE_ESTOP;
        robot.mode = SYSID_MODE_NONE;
        robot.fault = SYSID_FAULT_NONE;
        robot.phase = SYSID_PHASE_NONE;
        robot.selected_parameter = 0.0f;
        return;
    }

    if (left == RC_SW_MID && (robot.state == SYSID_STATE_ESTOP || robot.state == SYSID_STATE_READY))
    {
        robot.state = SYSID_STATE_READY;
        robot.mode = selected;
        robot.selected_parameter = TrialParameter();
        return;
    }

    if (robot.state == SYSID_STATE_READY && left == RC_SW_UP && selected != SYSID_MODE_NONE && selected == robot.mode)
    {
        if (!ValidSelectedRun() || !TrialFeedbackValid(true))
            SetFault(SYSID_FAULT_START);
        else
        {
            robot.state = SYSID_STATE_STARTING;
            if (SYSID_IS_HIP(robot.trial))
                BuildHipRequest(0.0f, false);
        }
        return;
    }

    if (robot.state != SYSID_STATE_STARTING && robot.state != SYSID_STATE_RUNNING &&
        robot.state != SYSID_STATE_DRY_RUNNING)
    {
        if (robot.state == SYSID_STATE_ESTOP || robot.state == SYSID_STATE_READY)
            robot.state = SYSID_STATE_NEED_ESTOP;
        return;
    }
    if (left != RC_SW_UP || selected != robot.mode)
    {
        robot.state = SYSID_STATE_NEED_ESTOP;
        return;
    }
    if (robot.state == SYSID_STATE_STARTING)
    {
        if (!ValidSelectedRun() || !TrialFeedbackValid(true))
        {
            SetFault(SYSID_FAULT_START);
            return;
        }
        robot.start_us = robot.now_us;
        robot.phase = SYSID_PHASE_NONE;
        hold_samples = moving_mask = 0;
        stable_since_us = moving_since_us = 0;
        for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        {
            hold_sum_nm[i] = 0.0f;
            hold_torque_nm[i] = 0.0f;
        }
        robot.state = robot.real_hardware ? SYSID_STATE_RUNNING : SYSID_STATE_DRY_RUNNING;
    }

    RunTrajectory((float)(robot.now_us - robot.start_us) * 1.0e-6f);
}

void RobotInit(void)
{
    robot = (RobotInstance){.trial = SYSID_EXPERIMENT,
                            .hip_pattern = SYSID_HIP_PATTERN,
                            .workpoint_id = kWorkpointId,
                            .direction = SYSID_TRIAL_DIRECTION,
                            .real_hardware = SYSID_REAL_HARDWARE != 0};
    for (unsigned i = 0; i < CHASSIS_JOINT_COUNT; ++i)
        robot.center_rad[i] = robot.real_hardware && (JointMask() & (1u << i)) ? kCenterRad[i] : 0.0f;
    amplitude = robot.real_hardware ? &kRealAmplitude : &kPreviewAmplitude;
    const bool telemetry_ready = SysIdVofaInit();
    robot.remote = RemoteControlInit(SYSID_RC_PORT);
    robot.config_valid = telemetry_ready && robot.remote && ValidTrialConfig();
    if (robot.config_valid && robot.real_hardware)
    {
        robot.chassis = ChassisInit(&kSysIdChassisConfig);
        robot.config_valid = robot.chassis != NULL;
    }
    if (!robot.config_valid)
        SetFault(SYSID_FAULT_CONFIG);
}

RobotInstance *RobotGetInstance(void)
{
    return &robot;
}

void RobotTask(void)
{
    const uint64_t previous_us = robot.now_us;
    robot.now_us = DWT_GetTimeline_us();
    robot.request = (ChassisRequest){0};
    if (robot.chassis)
        ChassisFeedbackUpdate(robot.chassis);

    uint8_t left = RC_SW_OFF;
    uint8_t right = RC_SW_OFF;
    taskENTER_CRITICAL();
    robot.remote_online = robot.remote && RemoteControlIsOnline();
    if (robot.remote)
    {
        left = robot.remote[TEMP].rc.switch_left;
        right = robot.remote[TEMP].rc.switch_right;
    }
    taskEXIT_CRITICAL();

    if (!robot.config_valid)
        SetFault(SYSID_FAULT_CONFIG);
    else if (!robot.remote_online || left == RC_SW_OFF || right == RC_SW_OFF)
        SetFault(SYSID_FAULT_REMOTE);
    else if (previous_us &&
             (robot.now_us <= previous_us || (robot.real_hardware && robot.now_us - previous_us > kMaxControlGapUs)))
        SetFault(SYSID_FAULT_TIMING);
    else if (!TrialFeedbackValid(false))
        SetFault(SYSID_FAULT_FEEDBACK);
    else
        RunTrial(left, SelectMode(right));

    if (robot.chassis)
    {
        robot.chassis->request = robot.request;
        ChassisTask(robot.chassis);
        if (robot.chassis->fault)
            SetFault(SYSID_FAULT_CHASSIS);
    }
    SysIdVofaSend(&robot);
}
