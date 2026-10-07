/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* 串口错误统计: 前 USART_ERR_VERBOSE_CNT 次完整打印, 之后按 1Hz 汇总
 * (噪声错误可能几十条/秒, 逐条打印会把有效日志全部淹掉) */
#define USART_ERR_VERBOSE_CNT 3
typedef struct {
    uint32_t total;       // 累计错误次数
    uint32_t total_last;  // 上次汇总时的累计值(算速率)
    uint32_t pe, fe, ne, ore;  // 累计分类次数
    float log_time;       // 上次汇总时间
} UsartErrStat_s;
static UsartErrStat_s usart_err_stat[DEVICE_USART_CNT];

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *        ORE(溢出)多是 CPU/DMA 来不及处理(中断占用久、日志过多);
 *        FE/NE/PE(帧错/噪声/校验错)才是线上干扰(走线、共地、EMI), 两者排查方向完全不同
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            uint32_t err = huart->ErrorCode;
            UsartErrStat_s *stat = &usart_err_stat[i];

            /* 清错误标志兜底: HAL 中断里一般已清 ORE, 但残留标志会让本回调被反复触发 */
            __HAL_UART_CLEAR_PEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            huart->ErrorCode = HAL_UART_ERROR_NONE;

            /* 重新挂接收 */
            USARTServiceInit(usart_instance[i]);
            /* 重挂结果检测: 若 HAL 状态没回到 BUSY_RX, 说明这次重挂没生效(该串口会一直收不到数据) */
            if (huart->RxState != HAL_UART_STATE_BUSY_RX)
            {
                LOGERROR("[bsp_usart] RX re-arm FAILED, RxState 0x%X, idx [%d]", (unsigned)huart->RxState, i);
            }

            /* ---- 错误统计: 前几次完整打印, 之后 1Hz 汇总, 避免刷屏淹没有效日志 ---- */
            stat->total++;
            if (err & HAL_UART_ERROR_PE) stat->pe++;
            if (err & HAL_UART_ERROR_FE) stat->fe++;
            if (err & HAL_UART_ERROR_NE) stat->ne++;
            if (err & HAL_UART_ERROR_ORE) stat->ore++;

            if (stat->total <= USART_ERR_VERBOSE_CNT)
            {
                LOGWARNING("[bsp_usart] USART error callback triggered, idx [%d], ErrorCode 0x%X (PE %d, FE %d, NE %d, ORE %d), total %d",
                           i, (unsigned)err,
                           (err & HAL_UART_ERROR_PE) ? 1 : 0, (err & HAL_UART_ERROR_FE) ? 1 : 0,
                           (err & HAL_UART_ERROR_NE) ? 1 : 0, (err & HAL_UART_ERROR_ORE) ? 1 : 0,
                           (int)stat->total);
            }
            else
            {
                float now = DWT_GetTimeline_ms();
                if (now - stat->log_time >= 1000.0f)
                {
                    stat->log_time = now;
                    LOGWARNING("[bsp_usart] USART error %d/s, total %d (PE %d, FE %d, NE %d, ORE %d), idx [%d]",
                               (int)(stat->total - stat->total_last), (int)stat->total, (int)stat->pe, (int)stat->fe,
                               (int)stat->ne, (int)stat->ore, i);
                    stat->total_last = stat->total;
                }
            }
            return;
        }
    }
}