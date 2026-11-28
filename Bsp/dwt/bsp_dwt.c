/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

#include "bsp_dwt.h"
#include "cmsis_os.h"

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_us;
static uint32_t last_cycles, cycle_wraps;

// Sample at least once per counter period. Tasks and ISRs share one coherent snapshot.
static uint64_t DWT_ReadCycles(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t current = DWT->CYCCNT;
    if (current < last_cycles)
        ++cycle_wraps;
    last_cycles = current;
    const uint32_t wraps = cycle_wraps;
    __set_PRIMASK(primask);
    return ((uint64_t)wraps << 32) | current;
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    last_cycles = cycle_wraps = 0;
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000u;
    CPU_FREQ_Hz_us = CPU_Freq_mHz;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __set_PRIMASK(primask);
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    const uint32_t current = (uint32_t)DWT_ReadCycles();
    const uint32_t delta = current - *cnt_last;
    *cnt_last = current;
    return (float)delta / CPU_FREQ_Hz;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    const uint32_t current = (uint32_t)DWT_ReadCycles();
    const uint32_t delta = current - *cnt_last;
    *cnt_last = current;
    return (double)delta / CPU_FREQ_Hz;
}

void DWT_SysTimeUpdate(void)
{
    (void)DWT_ReadCycles();
}

float DWT_GetTimeline_s(void)
{
    return (float)DWT_GetTimeline_us() * 1.0e-6f;
}

float DWT_GetTimeline_ms(void)
{
    return (float)DWT_GetTimeline_us() * 0.001f;
}

uint64_t DWT_GetTimeline_us(void)
{
    return DWT_ReadCycles() / CPU_FREQ_Hz_us;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
