#include "new_RC_VT13.c"
#include <stdlib.h>
#include <string.h>
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"

static const uint16_t vt13_crc16_tab[256] = { /* ... 保持原来的 CRC 表不变 ... */ };

static uint16_t vt13_crc16_calc(const uint8_t *p_msg, uint16_t len, uint16_t crc)
{
    if (p_msg == NULL) return 0xFFFFu;
    while (len--)
    {
        crc = (uint16_t)(crc >> 8) ^ vt13_crc16_tab[((uint16_t)(crc) ^ (uint16_t)(*p_msg++)) & 0x00FFu];
    }
    return crc;
}

static uint8_t vt13_verify_crc16(const uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2u) return 0u;
    uint16_t expected = vt13_crc16_calc(p_msg, (uint16_t)(len - 2u), 0xFFFFu);
    return ((expected & 0xFFu) == p_msg[len - 2u] &&
            ((expected >> 8u) & 0xFFu) == p_msg[len - 1u]) ? 1u : 0u;
}

static VT13_RC_t        vt13_rc;
static USARTInstance   *vt13_usart;
static DaemonInstance  *vt13_daemon;
static uint8_t          vt13_init_flag = 0;

/* ------------------- 内部处理逻辑 ------------------- */

static void VT13UpdateKeyboardMouseStatus(void)
{
    // 1. 获取键盘位域对应的 uint16 原始数据
    uint16_t raw_keys = *(uint16_t*)&vt13_rc.mouse_key[TEMP].keyboard;

    uint8_t is_ctrl  = (raw_keys & VT13_KEY_CTRL)  ? 1 : 0;
    uint8_t is_shift = (raw_keys & VT13_KEY_SHIFT) ? 1 : 0;

    // 2. 备份旧状态并清空当前分流状态
    memcpy(vt13_rc.last_key, vt13_rc.key, sizeof(vt13_rc.key));
    memset(vt13_rc.key, 0, sizeof(vt13_rc.key));

    // 3. 按照 Ctrl/Shift 修饰键进行分流 (逻辑对齐 remote_control.c)
    if (is_ctrl && is_shift)
        vt13_rc.key[KEY_PRESS_WITH_CTRL_SHIFT].keys = raw_keys;
    else if (is_ctrl)
        vt13_rc.key[KEY_PRESS_WITH_CTRL].keys = raw_keys;
    else if (is_shift)
        vt13_rc.key[KEY_PRESS_WITH_SHIFT].keys = raw_keys;
    else
        vt13_rc.key[KEY_PRESS_NORMAL].keys = raw_keys;

    // 4. 键盘边沿检测
    for (uint8_t state = 0; state < KEY_PRESS_STATE_NUM; state++)
    {
        uint16_t now  = vt13_rc.key[state].keys;
        uint16_t last = vt13_rc.last_key[state].keys;
        for (uint16_t i = 0, j = 0x1; i < KEY_NUM_TOTAL; j <<= 1, i++)
        {
            if (i == KEY_CTRL || i == KEY_SHIFT) continue;
            if ((now & j) && !(last & j)) vt13_rc.key_count[state][i]++;
        }
    }

    // 5. 鼠标计数
    if (vt13_rc.mouse_key[TEMP].mouse.press_l && !vt13_rc.mouse_key[LAST].mouse.press_l)
        vt13_rc.mouse_count[0]++;
    if (vt13_rc.mouse_key[TEMP].mouse.press_r && !vt13_rc.mouse_key[LAST].mouse.press_r)
        vt13_rc.mouse_count[1]++;
    if (vt13_rc.mouse_key[TEMP].mouse.press_m && !vt13_rc.mouse_key[LAST].mouse.press_m)
        vt13_rc.mouse_count[2]++;
}

static void VT13UpdateButtonStatus(void)
{
    /* 暂停键 / 功能键 / 扳机 / 鼠标各键的 Toggle 逻辑 */
    #define UPDATE_TOGGLE(btn, flag, count, last) \
    if (btn && !last) { flag = !flag; count++; }  \
    last = btn;

    UPDATE_TOGGLE(vt13_rc.rc.pause,   vt13_rc.button_status.pause_flag,   vt13_rc.button_status.pause_count,   vt13_rc.button_status.pause_last)
    UPDATE_TOGGLE(vt13_rc.rc.fn_1,    vt13_rc.button_status.fn_1_flag,    vt13_rc.button_status.fn_1_count,    vt13_rc.button_status.fn_1_last)
    UPDATE_TOGGLE(vt13_rc.rc.fn_2,    vt13_rc.button_status.fn_2_flag,    vt13_rc.button_status.fn_2_count,    vt13_rc.button_status.fn_2_last)
    UPDATE_TOGGLE(vt13_rc.rc.trigger, vt13_rc.button_status.trigger_flag, vt13_rc.button_status.trigger_count, vt13_rc.button_status.trigger_last)
    
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_l, vt13_rc.button_status.mouse_l_flag, vt13_rc.button_status.mouse_l_count, vt13_rc.button_status.mouse_l_last)
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_r, vt13_rc.button_status.mouse_r_flag, vt13_rc.button_status.mouse_r_count, vt13_rc.button_status.mouse_r_last)
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_m, vt13_rc.button_status.mouse_m_flag, vt13_rc.button_status.mouse_m_count, vt13_rc.button_status.mouse_m_last)
}

static uint8_t VT13Decode(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < VT13_FRAME_SIZE) return 0u;
    if (buf[0] != VT13_SOF_1 || buf[1] != VT13_SOF_2) return 0u;
    if (!vt13_verify_crc16(buf, VT13_FRAME_SIZE)) return 0u;

    uint64_t raw64 = 0;
    memcpy(&raw64, buf + 2, sizeof(uint64_t));

    /* 摇杆/拨轮解析 */
    vt13_rc.rc.rocker_r_ = VT13_CH_TO_SIGNED((raw64 >>  0) & 0x7FFu);
    vt13_rc.rc.rocker_r1 = VT13_CH_TO_SIGNED((raw64 >> 11) & 0x7FFu);
    vt13_rc.rc.rocker_l1 = VT13_CH_TO_SIGNED((raw64 >> 22) & 0x7FFu);
    vt13_rc.rc.rocker_l_ = VT13_CH_TO_SIGNED((raw64 >> 33) & 0x7FFu);
    vt13_rc.rc.mode_switch = (uint8_t)((raw64 >> 44) & 0x3u);
    vt13_rc.rc.pause       = (uint8_t)((raw64 >> 46) & 0x1u);
    vt13_rc.rc.fn_1        = (uint8_t)((raw64 >> 47) & 0x1u);
    vt13_rc.rc.fn_2        = (uint8_t)((raw64 >> 48) & 0x1u);
    vt13_rc.rc.dial        = VT13_CH_TO_SIGNED((raw64 >> 49) & 0x7FFu);
    vt13_rc.rc.trigger     = (uint8_t)((raw64 >> 60) & 0x1u);

    /* 键鼠解析 - 修复原有的内存越界 Bug */
    memcpy(&vt13_rc.mouse_key[LAST], &vt13_rc.mouse_key[TEMP], sizeof(VT13_MouseKey_t));
    memcpy(&vt13_rc.mouse_key[TEMP], &buf[10], sizeof(VT13_MouseKey_t));
    
    VT13UpdateButtonStatus();        // 更新硬件 Toggle 状态
    VT13UpdateKeyboardMouseStatus(); // 更新逻辑键盘计数

    return 1u;
}

static void VT13RxCallback(void)
{
    if (VT13Decode(vt13_usart->recv_buff, VT13_FRAME_SIZE)) {
        DaemonReload(vt13_daemon);
    }
}

static void VT13LostCallback(void *id)
{
    memset(&vt13_rc, 0, sizeof(vt13_rc));
    USARTServiceInit(vt13_usart);
    LOGWARNING("[VT13] remote control lost");
}

VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart)
{
    USART_Init_Config_s usart_conf = {
        .usart_handle    = huart,
        .recv_buff_size  = VT13_FRAME_SIZE,
        .module_callback = VT13RxCallback,
    };
    vt13_usart = USARTRegister(&usart_conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 70, 
        .callback     = VT13LostCallback,
    };
    vt13_daemon = DaemonRegister(&daemon_conf);

    memset(&vt13_rc, 0, sizeof(vt13_rc));
    vt13_init_flag = 1;
    return &vt13_rc;
}

uint8_t VT13RemoteIsOnline(void)
{
    return (vt13_init_flag && vt13_daemon != NULL) ? DaemonIsOnline(vt13_daemon) : 0u;
}