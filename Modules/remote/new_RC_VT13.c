#include "new_RC_VT13.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"



/* ================================================================
 *  CRC-16/CCITT-FALSE
 *  多项式 P(x) = x^16 + x^12 + x^5 + 1  (0x1021)
 *  初值 0xFFFF，输入/输出不反转，无最终 XOR
 * ================================================================ */
static const uint16_t vt13_crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

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