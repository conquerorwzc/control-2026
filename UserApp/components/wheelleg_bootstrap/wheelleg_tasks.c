#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "gen_config.h"
#include "message_center.h"
#include "robot.h"
#include "task.h"

typedef struct
{
    uint32_t control_steps;
    uint32_t policy_slots;
    uint32_t overruns;
    uint32_t last_period_us;
    uint32_t max_execution_us;
} WheelLegTimingStatus;

// 调试器可只读观察；发布的副本供后续遥测使用，当前没有电机实例。
WheelLegTimingStatus wheelleg_timing_status;
static Publisher_t *timing_pub;

_Static_assert(configTICK_RATE_HZ % WL_CONTROL_HZ == 0, "RTOS tick must divide the control period");
_Static_assert(WL_CONTROL_HZ / WL_POLICY_HZ == WL_POLICY_DECIMATION, "Invalid policy decimation");

static void WheelLegTask(void const *argument)
{
    (void)argument;
    RobotInit();
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = configTICK_RATE_HZ / WL_CONTROL_HZ;
    const uint32_t period_us = 1000000u / WL_CONTROL_HZ;
    uint64_t previous_start = DWT_GetTimeline_us();
    for (;;)
    {
        const uint64_t start = DWT_GetTimeline_us();
        wheelleg_timing_status.last_period_us = (uint32_t)(start - previous_start);
        previous_start = start;
        RobotTask();
        if (wheelleg_timing_status.control_steps % WL_POLICY_DECIMATION == 0u)
        {
            // 这是 100 Hz 调度槽，尚未集成网络推理。
            wheelleg_timing_status.policy_slots++;
            PubPushMessage(timing_pub, &wheelleg_timing_status);
        }
        wheelleg_timing_status.control_steps++;
        const uint32_t elapsed_us = (uint32_t)(DWT_GetTimeline_us() - start);
        if (elapsed_us > wheelleg_timing_status.max_execution_us)
            wheelleg_timing_status.max_execution_us = elapsed_us;
        if (elapsed_us >= period_us || xTaskGetTickCount() - last_wake >= period_ticks)
        {
            wheelleg_timing_status.overruns++;
            // 超期后重新对齐；不连续补跑多个控制周期。
            last_wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

void OSTaskInit(void)
{
    // 使用已配置的实际时钟，避免 BSPInit 中 STM32H7 宏/固定频率不匹配。
    DWT_Init(SystemCoreClock / 1000000u);
    BSPLogInit();
    timing_pub = PubRegister("wheelleg_timing", sizeof(WheelLegTimingStatus));
    osThreadDef(wheelleg, WheelLegTask, osPriorityNormal, 0, 512);
    const osThreadId handle = osThreadCreate(osThread(wheelleg), NULL);
    configASSERT(handle != NULL);
}
