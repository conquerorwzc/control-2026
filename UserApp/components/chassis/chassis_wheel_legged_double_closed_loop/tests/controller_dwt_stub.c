/**
 ******************************************************************************
 * @file    controller_dwt_stub.c
 * @brief   主机端 controller PID 固定周期 DWT 测试桩
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 为主机端 controller.c 单测提供固定 1 ms 的 PID 时间间隔。
 *
 * @param count controller PID 保存的 DWT 计数器位置。
 * @return 固定控制周期，单位 s。
 */
float DWT_GetDeltaT(uint32_t *count)
{
    if (count != 0)
    {
        (*count)++;
    }
    return 0.001f;
}
