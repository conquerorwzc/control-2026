#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "robot.h"
#include "task.h"

uint32_t leg_length_overruns;
_Static_assert(configTICK_RATE_HZ % LEG_LENGTH_CONTROL_HZ == 0, "Control period must be RTOS tick aligned");

static void LengthTask(void const *argument)
{
    (void)argument;
    RobotInit();
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = configTICK_RATE_HZ / LEG_LENGTH_CONTROL_HZ;
    for (;;)
    {
        RobotTask();
        if (xTaskGetTickCount() - wake >= period)
        {
            ++leg_length_overruns;
            wake = xTaskGetTickCount();
        }
        wake += period;
        const int32_t remaining = (int32_t)(wake - xTaskGetTickCount());
        if (remaining > 0)
            vTaskDelay((TickType_t)remaining);
    }
}

static void LengthDaemonTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        DaemonTask();
        osDelay(10);
    }
}

void OSTaskInit(void)
{
    DWT_Init(SystemCoreClock / 1000000u);
    BSPLogInit();
    osThreadDef(length, LengthTask, osPriorityNormal, 0, 1024);
    configASSERT(osThreadCreate(osThread(length), NULL) != NULL);
    osThreadDef(lengthdaemon, LengthDaemonTask, osPriorityBelowNormal, 0, 256);
    configASSERT(osThreadCreate(osThread(lengthdaemon), NULL) != NULL);
}
