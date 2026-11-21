#include "robot.h"
#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "daemon.h"
#include "remote_control.h"
#include "robot_config.h"
#include "task.h"
#include "vofa.h"

_Static_assert(WL_CONTROL_HZ * SYSID_RUNNER_PERIOD_US == 1000000u, "Runner/task period mismatch");
_Static_assert(sizeof(float) == 4 && SYSID_VOFA_CHANNELS == 24 && SYSID_VOFA_CHANNELS <= VOFA_MAX_CHANNELS,
               "JustFloat frame layout mismatch");
// 8N1: (24 floats + tail) * 4 bytes * 10 bits * 500 Hz = 500000 bit/s.
_Static_assert((SYSID_VOFA_CHANNELS + 1u) * 4u * 10u * WL_CONTROL_HZ <= SYSID_VOFA_BAUD &&
                   SYSID_VOFA_BAUD <= 1000000u, "SysID exceeds the serial link budget");

static RC_ctrl_t *remote;
static SysIdRunnerInstance runner;
static bool ready;
static uint16_t frame_seq;
static uint64_t last_daemon_us;

static SysIdSwitch DecodeSwitch(uint8_t position)
{
    switch (position)
    {
    case RC_SW_DOWN: return SYSID_SWITCH_DOWN;
    case RC_SW_MID: return SYSID_SWITCH_MIDDLE;
    case RC_SW_UP: return SYSID_SWITCH_UP;
    default: return SYSID_SWITCH_INVALID;
    }
}

void RobotInit(void)
{
    // 这个 target 没有已绑定的电机实例/发送任务。拒绝把预览开关直接当成出力开关。
    if (!kSysIdConfig.dry_run || !SysIdRunnerInit(&runner, &kSysIdConfig))
        return;

    // USART1 仅供本目标 VOFA 使用；不改 CubeMX 文件、不动遥控串口或时钟树。
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
    // 当前专用调度不含公共 daemon 任务；只在这个无电机目标中为队内 RC 看门狗提供节拍。
    if (now - last_daemon_us >= SYSID_DAEMON_TICK_US)
    {
        last_daemon_us = now;
        DaemonTask();
    }
    SysIdRunnerInput input = {.now_us = now, .synthetic_input_valid = true};
    taskENTER_CRITICAL();
    input.remote_online = RemoteControlIsOnline() != 0;
    input.left = DecodeSwitch(remote[TEMP].rc.switch_left);
    input.right = DecodeSwitch(remote[TEMP].rc.switch_right);
    taskEXIT_CRITICAL();
    for (unsigned i = 0; i < SYSID_LEG_COUNT; ++i)
    {
        input.center_rad[i] = kPreviewCenterRad[i];
        input.direction[i] = kPreviewDirection[i];
    }
    const SysIdRunnerOutput out = SysIdRunnerStep(&runner, &input);

    const uint16_t seq = frame_seq++; // 按控制拍计数，即使 DMA 忙也不重编号或补发旧帧。
    if (SYSID_VOFA_PORT->gState != HAL_UART_STATE_READY)
        return; // 不覆盖队内 VOFA 的静态 DMA 发送缓冲，不阻塞控制。
    float channels[SYSID_VOFA_CHANNELS] = {0};
    channels[0] = (float)seq;
    channels[1] = (float)((uint32_t)now >> 16);
    channels[2] = (float)((uint16_t)now);
    channels[3] = (float)out.state;
    channels[4] = (float)out.mode;
    // bit6=RC online；bit5=驱动就绪、bit7=允许真实执行、bit0..3=新反馈均为0。
    channels[5] = input.remote_online ? 64.0f : 0.0f;
    // 6..21 没有真实传感器/命令；保持占位零，不允许导入真机辨识。
    channels[22] = kPreviewCenterRad[0] + kPreviewDirection[0] * out.excitation_rad;
    channels[23] = kPreviewCenterRad[1] + kPreviewDirection[1] * out.excitation_rad;
    VOFAJustFloatSend(channels, SYSID_VOFA_CHANNELS);
}
