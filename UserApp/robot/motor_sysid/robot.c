#include "robot.h"
#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "remote_control.h"
#include "robot_config.h"
#include "task.h"
#include "vofa.h"

_Static_assert(WL_CONTROL_HZ *SYSID_RUNNER_PERIOD_US == 1000000u, "Runner/task period mismatch");
_Static_assert(sizeof(float) == 4 && SYSID_VOFA_CHANNELS == 24 && SYSID_VOFA_CHANNELS <= VOFA_MAX_CHANNELS,
               "JustFloat frame layout mismatch");
// 8N1: (24 floats + tail) * 4 bytes * 10 bits * 500 Hz = 500000 bit/s.
_Static_assert((SYSID_VOFA_CHANNELS + 1u) * 4u * 10u * WL_CONTROL_HZ <= SYSID_VOFA_BAUD && SYSID_VOFA_BAUD <= 1000000u,
               "SysID exceeds the serial link budget");

static RC_ctrl_t *remote;
static SysIdRunnerInstance runner;
static bool ready;
static uint16_t frame_seq;
static uint64_t last_daemon_us;

static SysIdSwitch DecodeSwitch(uint8_t position)
{
    switch (position)
    {
    case RC_SW_DOWN:
        return SYSID_SWITCH_DOWN;
    case RC_SW_MID:
        return SYSID_SWITCH_MIDDLE;
    case RC_SW_UP:
        return SYSID_SWITCH_UP;
    default:
        return SYSID_SWITCH_INVALID;
    }
}

// 本目标拥有自己的实例。仅复用队内 DMMotorTask 的发送入口，不改 OSTask 或总线驱动。
static DMMotorInstance *hips[SYSID_HIP_COUNT];
static SysIdHipInstance hip_control;
static uint32_t last_feed[SYSID_HIP_COUNT];
static uint64_t last_fresh_us[SYSID_HIP_COUNT], previous_tick_us;
static bool observed[SYSID_HIP_COUNT], registered, registration_stop_seen;
enum
{
    SYSID_BOOT_OK,
    SYSID_CONFIG_ERROR,
    SYSID_FILTER_ERROR,
    SYSID_SENDER_ERROR,
    SYSID_LINK_LOST
};
static unsigned inhibit; // 非零为整机闭锁；重新上电前不再注册、不调用掉线重使能回调。

static bool Installed(unsigned i)
{
    return (kHipControl.measured_mask & (1u << i)) != 0;
}

static unsigned CheckHardware(void)
{
    if (!SysIdHipInit(&hip_control, &kHipControl) || kRealTrial.dry_run ||
        (kTestSide != SYSID_HIP_SIDE_LEFT && kTestSide != SYSID_HIP_SIDE_RIGHT) || kFeedbackTimeoutUs == 0 ||
        kMaxControlGapUs <= SYSID_RUNNER_PERIOD_US)
        return SYSID_CONFIG_ERROR;
    const unsigned first = kTestSide == SYSID_HIP_SIDE_LEFT ? 0u : 2u;
    for (unsigned j = 0; j < 2; ++j)
    {
        const SysIdHipJointConfig *c = &kHipControl.joint[first + j];
        if (!Installed(first + j) || (kRealDirection[j] != 1 && kRealDirection[j] != -1) ||
            kRealTrial.min_angle_rad[j] < c->q_min_rad || kRealTrial.max_angle_rad[j] > c->q_max_rad)
            return SYSID_CONFIG_ERROR;
    }
    for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if (!Installed(i))
            continue;
        const SysIdHipWiring *w = &kHipWiring[i];
        if ((w->bus != &hcan1 && w->bus != &hcan2 && w->bus != &hcan3) || (w->sign != 1 && w->sign != -1) ||
            !isfinite(w->zero_rad) || !w->tx_id || !w->rx_id || w->tx_id > 0x7ffu || w->rx_id > 0x7ffu ||
            w->tx_id == w->rx_id || w->feedback_id > 15u ||
            kHipControl.joint[i].max_abs_command_torque_nm > DM_T_MAX_J8009P)
            return SYSID_CONFIG_ERROR;
        unsigned filters = 1;
        for (unsigned j = 0; j < i; ++j)
        {
            const SysIdHipWiring *other = &kHipWiring[j];
            if (!Installed(j) || other->bus != w->bus)
                continue;
            ++filters;
            if (other->tx_id == w->tx_id || other->rx_id == w->rx_id || other->tx_id == w->rx_id ||
                other->rx_id == w->tx_id)
                return SYSID_CONFIG_ERROR;
        }
        if (filters > w->bus->Init.StdFiltersNbr)
            return SYSID_FILTER_ERROR;
    }
    return SYSID_BOOT_OK;
}

static void WriteHips(const SysIdHipOutput *out)
{
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if (!hips[i])
            continue;
        if (out && out->status == SYSID_HIP_ACTIVE &&
            (i < 2u ? kTestSide == SYSID_HIP_SIDE_LEFT : kTestSide == SYSID_HIP_SIDE_RIGHT))
        {
            DMMotorSetRef(hips[i], out->torque_nm[i]);
            DMMotorEnable(hips[i]);
        }
        else
        {
            DMMotorSetRef(hips[i], 0.0f);
            DMMotorStop(hips[i]);
        }
    }
    taskEXIT_CRITICAL();
}

static bool RegisterHips(void)
{
    // DMMotorInit 本身会发一次硬件使能。此函数只能在校验后由人工下→中触发一次。
    for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if (!Installed(i))
            continue;
        const SysIdHipWiring *w = &kHipWiring[i];
        Motor_Init_Config_s cfg = {
            .motor_type = J8009P,
            .can_init_config = {.can_handle = w->bus, .tx_id = w->tx_id, .rx_id = w->rx_id},
            .controller_setting_init_config = {
                .motor_reverse_flag = w->sign < 0 ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
                .feedback_reverse_flag = w->sign < 0 ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL}};
        hips[i] = DMMotorInit(&cfg);
        if (!hips[i] || !hips[i]->daemon || !hips[i]->motor_can_instance)
            return false;
        DMMotorSetRef(hips[i], 0.0f);
        DMMotorStop(hips[i]);
        osThreadDef(sysid_dm, DMMotorTask, osPriorityBelowNormal, 0, 128);
        if (!osThreadCreate(osThread(sysid_dm), hips[i]))
            return false;
        last_feed[i] = hips[i]->feed_cnt; // 注册前/期间积累的值不发“本拍更新”标记
    }
    return true;
}

static unsigned ReadHips(SysIdHipInput *in, uint64_t now)
{
    unsigned fresh = 0;
    for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        if (!hips[i])
            continue;
        taskENTER_CRITICAL();
        const DM_Motor_Measure_s m = hips[i]->measure;
        const uint32_t feed = hips[i]->feed_cnt;
        const unsigned dlc = hips[i]->motor_can_instance->rx_len;
        taskEXIT_CRITICAL();
        const bool valid_frame = dlc == 8 && m.state == 1 && m.id == kHipWiring[i].feedback_id;
        if (feed != last_feed[i])
        {
            last_feed[i] = feed;
            last_fresh_us[i] = now;
            observed[i] = valid_frame;
            if (valid_frame)
                fresh |= 1u << i;
        }
        in->feedback[i] =
            (SysIdHipFeedback){.valid = valid_frame && observed[i] && now - last_fresh_us[i] <= kFeedbackTimeoutUs,
                               .q_rad = m.total_angle - kHipWiring[i].zero_rad,
                               .qd_rad_s = m.velocity,
                               .tau_nm = m.torque,
                               .temperature_c = fmaxf(m.T_Mos, m.T_Rotor)};
    }
    return fresh;
}

void RobotInit(void)
{
    if (!SysIdRunnerInit(&runner, SYSID_REAL_HIPS ? &kRealTrial : &kSysIdConfig))
        inhibit = SYSID_CONFIG_ERROR;
    if (SYSID_REAL_HIPS && !inhibit)
        inhibit = CheckHardware();
    SYSID_VOFA_PORT->Init.BaudRate = SYSID_VOFA_BAUD;
    if (HAL_UART_Init(SYSID_VOFA_PORT) != HAL_OK || SYSID_VOFA_PORT->hdmatx == NULL)
        return;
    VOFAInit(SYSID_VOFA_PORT);
    remote = RemoteControlInit(SYSID_RC_PORT);
    last_daemon_us = DWT_GetTimeline_us();
    ready = remote != NULL;
}

void RobotTask(void)
{
    if (!ready)
        return;
    const uint64_t now = DWT_GetTimeline_us();
    if (!inhibit && now - last_daemon_us >= SYSID_DAEMON_TICK_US)
    {
        last_daemon_us = now;
        // 本独立目标是 DaemonTask 的唯一调用者。原 DM 离线回调会重使能并阻塞 100ms；
        // 递减到零前直接锁停，之后不再调用 daemon，避免发出该帧或在控制拍阻塞。
        for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
            if (hips[i] && hips[i]->daemon->temp_count <= 1u)
                inhibit = SYSID_LINK_LOST;
        if (!inhibit)
            DaemonTask();
    }

    SysIdRunnerInput in = {.now_us = now, .synthetic_input_valid = !SYSID_REAL_HIPS};
    taskENTER_CRITICAL();
    in.remote_online = !inhibit && RemoteControlIsOnline() != 0;
    in.left = DecodeSwitch(remote[TEMP].rc.switch_left);
    in.right = DecodeSwitch(remote[TEMP].rc.switch_right);
    taskEXIT_CRITICAL();

    if (SYSID_REAL_HIPS && !registered && !inhibit)
    {
        if (registration_stop_seen && in.remote_online && in.left == SYSID_SWITCH_MIDDLE &&
            in.right != SYSID_SWITCH_INVALID)
        {
            registered = RegisterHips(); // 阻塞式初始化期间所有命令均为零
            if (!registered)
                inhibit = SYSID_SENDER_ERROR;
            registration_stop_seen = false;
            last_daemon_us = DWT_GetTimeline_us();
            WriteHips(NULL);
            return; // 注册期间的老遥控/反馈不得进入本次试验
        }
        registration_stop_seen = in.remote_online && in.left == SYSID_SWITCH_DOWN && in.right != SYSID_SWITCH_INVALID;
    }

    SysIdHipInput hi = {.side = kTestSide};
    unsigned flags = 0;
    if (registered && !inhibit)
    {
        flags = ReadHips(&hi, now);
        in.feedback_valid = now > previous_tick_us && now - previous_tick_us <= kMaxControlGapUs &&
                            SysIdHipFeedbackValid(&kHipControl, hi.feedback);
        if (in.feedback_valid)
            flags |= 1u << 5;
    }
    previous_tick_us = now;
    const unsigned first = kTestSide == SYSID_HIP_SIDE_RIGHT ? 2u : 0u;
    for (unsigned j = 0; j < SYSID_LEG_COUNT; ++j)
    {
        in.center_rad[j] = SYSID_REAL_HIPS ? kHipControl.joint[first + j].center_rad : kPreviewCenterRad[j];
        in.direction[j] = SYSID_REAL_HIPS ? kRealDirection[j] : kPreviewDirection[j];
    }
    SysIdRunnerOutput out = SysIdRunnerStep(&runner, &in);
    if (inhibit)
        out.state = SYSID_STATE_FAULT;
    hi.state = out.state;
    for (unsigned j = 0; j < SYSID_LEG_COUNT; ++j)
        hi.q_ref_rad[j] = in.center_rad[j] + in.direction[j] * out.excitation_rad;
    SysIdHipOutput ctrl = {0};
    if (registered)
    {
        ctrl = SysIdHipStep(&hip_control, &hi);
        if (ctrl.status == SYSID_HIP_FAULT)
            out.state = SYSID_STATE_FAULT;
        WriteHips(&ctrl); // 在检查 VOFA/DMA 之前每拍执行，包括停止、故障和完成
    }
    if (ctrl.status == SYSID_HIP_ACTIVE)
        flags |= 1u << 7;
    if (ctrl.clipped)
        flags |= 1u << 4;
    if (in.remote_online)
        flags |= 1u << 6;

    const uint16_t seq = frame_seq++;
    if (SYSID_VOFA_PORT->gState != HAL_UART_STATE_READY)
        return;
    float channels[SYSID_VOFA_CHANNELS] = {(float)seq,           (float)((uint32_t)now >> 16),
                                           (float)(uint16_t)now, (float)out.state,
                                           (float)out.mode,      (float)(flags | (inhibit << 8))};
    for (unsigned i = 0; i < SYSID_HIP_COUNT; ++i)
    {
        channels[6 + 4 * i] = hi.feedback[i].q_rad;
        channels[7 + 4 * i] = hi.feedback[i].qd_rad_s;
        channels[8 + 4 * i] = ctrl.torque_nm[i];
        channels[9 + 4 * i] = hi.feedback[i].tau_nm;
    }
    channels[22] = hi.q_ref_rad[0];
    channels[23] = hi.q_ref_rad[1];
    VOFAJustFloatSend(channels, SYSID_VOFA_CHANNELS);
}
