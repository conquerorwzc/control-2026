#include "selfcontrol.h"
#include "bsp_log.h"
#include "crc_func.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include <stdbool.h>

#define SELF_CONTROL_FRAME_SIZE 39u // 接收缓冲区大小
#define ROBOT_INTERACTIVE_DATA_CMD_ID 0x0302

// 下位机协议参数（与下位机 protocol_packed 固定 data_length=30 一致）
#define SC_SOF_BYTE 0xA5u
#define SC_HEADER_LEN 5u // SOF(1)+len(2)+seq(1)+crc8(1)
#define SC_CMD_LEN 2u
#define SC_CRC16_LEN 2u
#define SC_MAX_DATA_LEN 30u

// 流式缓存：用来拼帧、处理粘包/拆包
#define SC_CACHE_SIZE 256u
static uint8_t sc_cache[SC_CACHE_SIZE];
static uint16_t sc_cache_len = 0;

// ===== 统计：用来验证每秒解析到多少帧 =====
volatile uint32_t sc_stat_frames_ok = 0;   // CRC16通过且吐出完整帧的次数
volatile uint32_t sc_stat_crc8_fail = 0;   // 头CRC8失败次数
volatile uint32_t sc_stat_crc16_fail = 0;  // 帧CRC16失败次数
volatile uint32_t sc_stat_resync_drop = 0; // 为重新同步丢弃的字节数

// 通过 DMA 计数器推断本次实际接收字节数（不依赖 bsp 传 Size）
static uint16_t SelfControl_GuessRxSizeFromDma(const USARTInstance *inst)
{
    if (inst == NULL || inst->usart_handle == NULL || inst->usart_handle->hdmarx == NULL)
        return 0;

    uint16_t buf_size = (uint16_t)inst->recv_buff_size;
    uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(inst->usart_handle->hdmarx);
    if (remain > buf_size)
        return 0;

    return (uint16_t)(buf_size - remain);
}

// 流式喂入并解析（解析成功后调用 selfcontrol_data_solve 更新 unpacked_data）
static void SelfControl_FeedBytes(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return;

    // 1) 追加到缓存；满了就丢最老的数据，确保不断流
    if (len >= SC_CACHE_SIZE)
    {
        buf += (len - SC_CACHE_SIZE);
        len = SC_CACHE_SIZE;
        sc_cache_len = 0;
    }
    else if ((uint32_t)sc_cache_len + len > SC_CACHE_SIZE)
    {
        uint16_t drop = (uint16_t)((uint32_t)sc_cache_len + len - SC_CACHE_SIZE);
        memmove(sc_cache, sc_cache + drop, sc_cache_len - drop);
        sc_cache_len -= drop;
    }

    memcpy(sc_cache + sc_cache_len, buf, len);
    sc_cache_len += len;

    // 2) 尽可能多地吐出完整帧
    while (sc_cache_len >= SC_HEADER_LEN)
    {
        // 2.1 找帧头 0xA5
        uint16_t pos = 0;
        while (pos < sc_cache_len && sc_cache[pos] != SC_SOF_BYTE)
            pos++;

        if (pos > 0)
        {
            memmove(sc_cache, sc_cache + pos, sc_cache_len - pos);
            sc_cache_len -= pos;
            sc_stat_resync_drop += pos;
            if (sc_cache_len < SC_HEADER_LEN)
                break;
        }

        // 2.2 CRC8 校验帧头
        if (!verify_CRC8_check_sum((unsigned char *)sc_cache, SC_HEADER_LEN))
        {
            sc_stat_crc8_fail++;
            sc_stat_resync_drop++;

            // 这个 0xA5 不是真帧头，丢 1 字节继续找
            memmove(sc_cache, sc_cache + 1, sc_cache_len - 1);
            sc_cache_len -= 1;
            continue;
        }

        // 2.3 读取 data_length（小端）并计算整帧长度
        uint16_t data_len = (uint16_t)(sc_cache[1] | (sc_cache[2] << 8));
        if (data_len > SC_MAX_DATA_LEN)
        {
            memmove(sc_cache, sc_cache + 1, sc_cache_len - 1);
            sc_cache_len -= 1;
            continue;
        }

        uint16_t frame_len = (uint16_t)(SC_HEADER_LEN + SC_CMD_LEN + data_len + SC_CRC16_LEN);
        if (sc_cache_len < frame_len)
        {
            // 数据不够一帧，等下次再来
            break;
        }

        // 2.4 CRC16 校验整帧
        if (!verify_CRC16_check_sum(sc_cache, frame_len))
        {
            sc_stat_crc16_fail++;
            sc_stat_resync_drop++;

            memmove(sc_cache, sc_cache + 1, sc_cache_len - 1);
            sc_cache_len -= 1;
            continue;
        }

        // 2.5 成功得到一帧完整数据：调用你原来的解包入口
        selfcontrol_data_solve(sc_cache);
        sc_stat_frames_ok++;

        // 2.6 移除本帧，继续解析下一帧（一次回调可能吐出多帧）
        memmove(sc_cache, sc_cache + frame_len, sc_cache_len - frame_len);
        sc_cache_len -= frame_len;
    }
}

// 控制器实例
static SelfC self_control;

// 获取电机角度
float SelfControlGetMotorAngle(const SelfC *controller, uint8_t motor_index)
{
    if (controller == NULL || motor_index >= 5)
    {
        return 0.0f;
    }
    return controller->unpacked_data.motors[motor_index].angle;
}


UnpackedControllerData_t *GetSelfControlDataPtr(void)
{
    return &self_control.unpacked_data;
}

// 解析自定义控制器数据包
static bool parse_custom_controller_data(const uint8_t *packed_data, uint16_t packed_size,
                                         UnpackedControllerData_t *unpacked_data)
{
    if (packed_data == NULL || unpacked_data == NULL)
        return false;
    if (packed_size < 39)
        return false;
    if (packed_data[0] != 0xA5)
        return false;

    uint16_t cmd_id = ((uint16_t)packed_data[6] << 8) | packed_data[5];
    if (cmd_id != 0x0302)
        return false;

    const uint8_t *data_ptr = &packed_data[7]; // 指向 Data 段起始位置

    if (data_ptr[0] != 0x20)
        return false;

    // 解析电机数据 (4个电机)
    for (int i = 0; i < 5; i++)
    {
        unpacked_data->motors[i].id = data_ptr[1 + i * 5];
        int16_t angle_raw = ((int16_t)data_ptr[3 + i * 5] << 8) | data_ptr[2 + i * 5];
        unpacked_data->motors[i].angle = (float)angle_raw / 100.0f;
        unpacked_data->motors[i].is_online = data_ptr[5 + i * 5];
        // 扭矩状态字段已移除，保留为预留字节
    }
    return true;
}

// 数据解析函数(保持原有可用逻辑)
void selfcontrol_data_solve(uint8_t *frame)
{
    if (frame == NULL)
        return;

    // SOF
    if (frame[0] != SC_SOF_BYTE)
        return;

    // CRC8(header)
    if (!verify_CRC8_check_sum((unsigned char *)frame, SC_HEADER_LEN))
        return;

    // frame_len
    uint16_t data_len = (uint16_t)(frame[1] | (frame[2] << 8));
    if (data_len > SC_MAX_DATA_LEN)
        return;
    uint16_t frame_len = (uint16_t)(SC_HEADER_LEN + SC_CMD_LEN + data_len + SC_CRC16_LEN);

    // CRC16(whole frame)
    if (!verify_CRC16_check_sum((uint8_t *)frame, frame_len))
        return;

    // cmd_id（小端，cmd_id 在 frame[5..6]）
    uint16_t cmd_id = (uint16_t)(frame[5] | (frame[6] << 8));

    switch (cmd_id)
    {
    case ROBOT_INTERACTIVE_DATA_CMD_ID: // 自定义控制器数据(0x0302)
        // 使用原有解析逻辑，适配新数据结构
        parse_custom_controller_data(frame, frame_len, &self_control.unpacked_data);
        break;
    default:
        break;
    }
}

// USART接收回调函数
static void SelfControlRxCallback()
{
    if (self_control.usart_instance != NULL)
    {
        // 1) 推断本次实际接收长度（不改 bsp）
        uint16_t rx_len = SelfControl_GuessRxSizeFromDma(self_control.usart_instance);

        // 兜底：猜不到就喂整块
        if (rx_len == 0 || rx_len > SELF_CONTROL_FRAME_SIZE)
        {
            rx_len = SELF_CONTROL_FRAME_SIZE;
        }

        // 2) 可选：复制一份原始数据到模块buffer（方便你调试观察）
        uint16_t copy_len = (rx_len > SELF_CONTROL_FRAME_SIZE) ? SELF_CONTROL_FRAME_SIZE : rx_len;
        memcpy(self_control.selfcontrol_buff, self_control.usart_instance->recv_buff, copy_len);

        // 3) 核心：流式解包，解决粘包/拆包/错位
        SelfControl_FeedBytes(self_control.usart_instance->recv_buff, rx_len);
    }
}

// 初始化函数
SelfC *SelfControlInit(UART_HandleTypeDef *usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = SelfControlRxCallback;
    conf.usart_handle = usart_handle;
    conf.recv_buff_size = SELF_CONTROL_FRAME_SIZE;
    // 直接将USART实例保存到模块结构体中
    self_control.usart_instance = USARTRegister(&conf);

    return &self_control;
}