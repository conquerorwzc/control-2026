#include "remote_control.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 遥控器数据 (使用枚举 RC_DATA_NUM 作为数组大小)
static RC_ctrl_t rc_ctrl[RC_DATA_NUM];
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 =
        (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ =
        (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 =
        (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET; // 左侧拨轮
    RectifyRCjoystick();

    // 开关状态 (可以直接隐式转换为 RCSwitchState_e)
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    // ================== 1. 获取原始按键数据 ==================
    uint16_t raw_keys = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));

    // 提取当前是否有 Ctrl 或 Shift 被按下 (使用枚举索引)
    uint8_t is_ctrl = (raw_keys & (1 << KEY_CTRL)) ? 1 : 0;
    uint8_t is_shift = (raw_keys & (1 << KEY_SHIFT)) ? 1 : 0;

    // 先将当前所有按键状态清零，防止数据残留
    memset(rc_ctrl[TEMP].key, 0, sizeof(rc_ctrl[TEMP].key));

    // ================== 2. 互斥分配按键状态 ==================
    // 根据修饰键的状态，将 raw_keys 放入且仅放入一个状态数组中
    if (is_ctrl && is_shift)
    {
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].keys = raw_keys;
    }
    else if (is_ctrl)
    {
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys = raw_keys;
    }
    else if (is_shift)
    {
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys = raw_keys;
    }
    else
    {
        rc_ctrl[TEMP].key[KEY_PRESS_NORMAL].keys = raw_keys;
    }

    // ================== 3. 边沿检测 (按键计数器) ==================
    for (uint8_t state = 0; state < KEY_PRESS_STATE_NUM; state++)
    {
        uint16_t state_key_now = rc_ctrl[TEMP].key[state].keys;
        uint16_t state_key_last = rc_ctrl[LAST].key[state].keys;

        // 遍历所有按键位
        for (uint16_t i = 0, j = 0x1; i < KEY_NUM_TOTAL; j <<= 1, i++)
        {
            // 跳过修饰键本身，防止产生无意义的 trigger
            if (i == KEY_CTRL || i == KEY_SHIFT)
                continue;

            // 发生上升沿
            if ((state_key_now & j) && !(state_key_last & j))
            {
                rc_ctrl[TEMP].key_count[state][i]++;
            }
        }
    }

    // ================== 4. 鼠标按键计数 ==================
    uint8_t mouse_left_now = rc_ctrl[TEMP].mouse.press_l,
        mouse_left_last = rc_ctrl[LAST].mouse.press_l,
        mouse_right_now = rc_ctrl[TEMP].mouse.press_r,
        mouse_right_last = rc_ctrl[LAST].mouse.press_r;

    // 鼠标左键按下计数（检测上升沿），索引 0
    if (mouse_left_now && !mouse_left_last)
        rc_ctrl[TEMP].mouse_count[0]++;
    // 鼠标右键按下计数（检测上升沿），索引 1
    if (mouse_right_now && !mouse_right_last)
        rc_ctrl[TEMP].mouse_count[1]++;

    // 保存上一次的数据,用于按键持续按下和切换的判断
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线
        .callback = RCLostCallback,
        .owner_id = NULL,
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}