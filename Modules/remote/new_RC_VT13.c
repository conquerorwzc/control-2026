//
// Created by zhouz on 2026/2/28.
//

#include <stdlib.h>
#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "new_RC_VT13.h"

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

/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值
 * @param p_msg  数据指针
 * @param len    数据字节数（不含 CRC 本身）
 * @param crc    初始值（首次调用传入 0xFFFF）
 * @return       计算得到的 CRC16
 */
static uint16_t vt13_crc16_calc(const uint8_t *p_msg, uint16_t len, uint16_t crc)
{
    if (p_msg == NULL) return 0xFFFFu;
    while (len--)
    {
        crc = (uint16_t)(crc >> 8) ^ vt13_crc16_tab[((uint16_t)(crc) ^ (uint16_t)(*p_msg++)) & 0x00FFu];
    }
    return crc;
}

/**
 * @brief 校验一帧数据的 CRC16
 * @param p_msg  完整帧数据（含末尾 2 字节 CRC）
 * @param len    帧总字节数（含 CRC）
 * @return       1=校验通过，0=失败
 */
static uint8_t vt13_verify_crc16(const uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2u) return 0u;
    uint16_t expected = vt13_crc16_calc(p_msg, (uint16_t)(len - 2u), 0xFFFFu);
    return ((expected & 0xFFu) == p_msg[len - 2u] &&
            ((expected >> 8u) & 0xFFu) == p_msg[len - 1u]) ? 1u : 0u;
}

/* ================================================================
 *  模块私有变量
 * ================================================================ */
static VT13_RC_t        vt13_rc[2];        // [0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static USARTInstance   *vt13_usart;        // 串口实例
static DaemonInstance  *vt13_daemon;       // 守护进程实例
static uint8_t          vt13_init_flag = 0;// 初始化标志

/* ================================================================
 *  内部函数
 * ================================================================ */

/**
 * @brief 遥控器离线回调：清空数据并尝试重新启动串口接收
 */
static void VT13LostCallback(void *id)
{
    (void)id;
    memset(vt13_rc, 0, sizeof(vt13_rc));
    USARTServiceInit(vt13_usart);
    LOGWARNING("[VT13] remote control lost");
}

/**
 * @brief 更新按钮状态：处理toggle标志和按下计数
 */
static void VT13UpdateButtonStatus(void)
{
    /* 暂停键处理 */
    if (vt13_rc[TEMP].rc.pause && !vt13_rc[TEMP].button_status.pause_last) {
        vt13_rc[TEMP].button_status.pause_flag = !vt13_rc[TEMP].button_status.pause_flag;
        vt13_rc[TEMP].button_status.pause_count++;
    }
    vt13_rc[TEMP].button_status.pause_last = vt13_rc[TEMP].rc.pause;

    /* 功能键1处理 */
    if (vt13_rc[TEMP].rc.fn_1 && !vt13_rc[TEMP].button_status.fn_1_last) {
        vt13_rc[TEMP].button_status.fn_1_flag = !vt13_rc[TEMP].button_status.fn_1_flag;
        vt13_rc[TEMP].button_status.fn_1_count++;
    }
    vt13_rc[TEMP].button_status.fn_1_last = vt13_rc[TEMP].rc.fn_1;

    /* 功能键2处理 */
    if (vt13_rc[TEMP].rc.fn_2 && !vt13_rc[TEMP].button_status.fn_2_last) {
        vt13_rc[TEMP].button_status.fn_2_flag = !vt13_rc[TEMP].button_status.fn_2_flag;
        vt13_rc[TEMP].button_status.fn_2_count++;
    }
    vt13_rc[TEMP].button_status.fn_2_last = vt13_rc[TEMP].rc.fn_2;

    /* 扳机键处理 */
    if (vt13_rc[TEMP].rc.trigger && !vt13_rc[TEMP].button_status.trigger_last) {
        vt13_rc[TEMP].button_status.trigger_flag = !vt13_rc[TEMP].button_status.trigger_flag;
        vt13_rc[TEMP].button_status.trigger_count++;
    }
    vt13_rc[TEMP].button_status.trigger_last = vt13_rc[TEMP].rc.trigger;

    /* 鼠标左键处理 */
    if (vt13_rc[TEMP].mouse_key.mouse.press_l && !vt13_rc[TEMP].button_status.mouse_l_last) {
        vt13_rc[TEMP].button_status.mouse_l_flag = !vt13_rc[TEMP].button_status.mouse_l_flag;
        vt13_rc[TEMP].button_status.mouse_l_count++;
    }
    vt13_rc[TEMP].button_status.mouse_l_last = vt13_rc[TEMP].mouse_key.mouse.press_l;

    /* 鼠标右键处理 */
    if (vt13_rc[TEMP].mouse_key.mouse.press_r && !vt13_rc[TEMP].button_status.mouse_r_last) {
        vt13_rc[TEMP].button_status.mouse_r_flag = !vt13_rc[TEMP].button_status.mouse_r_flag;
        vt13_rc[TEMP].button_status.mouse_r_count++;
    }
    vt13_rc[TEMP].button_status.mouse_r_last = vt13_rc[TEMP].mouse_key.mouse.press_r;

    /* 鼠标中键处理 */
    if (vt13_rc[TEMP].mouse_key.mouse.press_m && !vt13_rc[TEMP].button_status.mouse_m_last) {
        vt13_rc[TEMP].button_status.mouse_m_flag = !vt13_rc[TEMP].button_status.mouse_m_flag;
        vt13_rc[TEMP].button_status.mouse_m_count++;
    }
    vt13_rc[TEMP].button_status.mouse_m_last = vt13_rc[TEMP].mouse_key.mouse.press_m;
}

/**
 * @brief 解析一帧 VT13 原始数据，写入双缓冲的 TEMP 槽位
 *
 * 帧格式（21 字节）：
 *   [0..1]   帧头 SOF（0xA9 0x53）
 *   [2..9]   摇杆 / 功能键 / 拨轮 / 扳机（8 字节位域，小端 uint64_t）
 *   [10..18] 鼠标 + 键盘（9 字节，直接 memcpy 到 VT13_MouseKey_t）
 *   [19..20] CRC-16/CCITT-FALSE
 *
 * 双缓冲机制（TEMP / LAST）：
 *   - TEMP 在帧间持久保留，decode 仅覆盖 rc / mouse_key 等协议字段，
 *     而 key_count / button_status 不会被清零，实现跨帧累加计数
 *   - 每帧末尾 memcpy(TEMP → LAST)，使 LAST 始终保存上一帧的完整快照，
 *     供下一帧 key_count 的边沿检测使用
 *   - 遥控器离线时 LostCallback 会 memset 整个双缓冲，重置所有状态
 *
 * 按键计次（与旧 SBUS 遥控器一致的上升沿检测）：
 *   - 对比 TEMP（当前帧）与 LAST（上一帧）的键盘位域
 *   - 仅在 bit 从 0→1 时递增对应 key_count，避免持续按下时重复计数
 *   - 区分三种模式：普通按键 / ctrl 组合 / shift 组合
 *
 * @param buf  接收缓冲区指针，长度须 >= VT13_FRAME_SIZE
 * @param len  数据长度
 * @return     1=解析成功(有效帧), 0=失败（帧头错误 / CRC 校验失败 / 参数无效）
 */
static uint8_t VT13Decode(const uint8_t *buf, uint16_t len)
{
    /* ---- 参数校验 ---- */
    if (buf == NULL || len < VT13_FRAME_SIZE) return 0u;

    /* ---- 1. 帧头校验：必须为 0xA9 0x53 ---- */
    if (buf[0] != VT13_SOF_1 || buf[1] != VT13_SOF_2){
        LOGWARNING("[VT13] bad SOF: 0x%02X 0x%02X", buf[0], buf[1]);
        return 0u;
    }

    /* ---- 2. CRC-16/CCITT-FALSE 校验 ----
     * 对前 19 字节（SOF + 有效载荷）计算 CRC，与帧尾 2 字节比对
     * 校验失败说明数据在传输中损坏，丢弃整帧 */
    if (!vt13_verify_crc16(buf, VT13_FRAME_SIZE)){
        LOGWARNING("[VT13] CRC error");
        return 0u;
    }

    /* ---- 3. 解析有效载荷段一：摇杆 / 功能键 / 拨轮 / 扳机 ----
     * buf[2..9] 共 8 字节，以小端序加载为 uint64_t，统一移位提取
     * 各字段位偏移及宽度参见头文件中帧布局注释 */
    uint64_t raw64 = 0;
    memcpy(&raw64, buf + 2, sizeof(uint64_t));

    /* 摇杆/拨轮：提取 11bit 原始值，减去中值 1024 转换为有符号偏移 [-660, +660] */
    vt13_rc[TEMP].rc.rocker_r_ = VT13_CH_TO_SIGNED((raw64 >>  0) & 0x7FFu); // ch_0 右摇杆水平
    vt13_rc[TEMP].rc.rocker_r1 = VT13_CH_TO_SIGNED((raw64 >> 11) & 0x7FFu); // ch_1 右摇杆垂直
    vt13_rc[TEMP].rc.rocker_l1 = VT13_CH_TO_SIGNED((raw64 >> 22) & 0x7FFu); // ch_2 左摇杆垂直
    vt13_rc[TEMP].rc.rocker_l_ = VT13_CH_TO_SIGNED((raw64 >> 33) & 0x7FFu); // ch_3 左摇杆水平

    /* 功能键 / 拨杆：直接取 1~2 bit，无需偏移校正 */
    vt13_rc[TEMP].rc.mode_switch = (uint8_t)((raw64 >> 44) & 0x3u);  // s1 左拨杆，2bit
    vt13_rc[TEMP].rc.pause       = (uint8_t)((raw64 >> 46) & 0x1u);  // 暂停键
    vt13_rc[TEMP].rc.fn_1        = (uint8_t)((raw64 >> 47) & 0x1u);  // 功能键 1
    vt13_rc[TEMP].rc.fn_2        = (uint8_t)((raw64 >> 48) & 0x1u);  // 功能键 2
    vt13_rc[TEMP].rc.dial        = VT13_CH_TO_SIGNED((raw64 >> 49) & 0x7FFu); // 拨轮
    vt13_rc[TEMP].rc.trigger     = (uint8_t)((raw64 >> 60) & 0x1u);  // 扳机

    /* ---- 4. 解析有效载荷段二：鼠标 + 键盘 ----
     * buf[10..18] 共 9 字节，与 VT13_MouseKey_t 内存布局一致，整块拷贝 */
    memcpy(&vt13_rc[TEMP].mouse_key, &buf[10], sizeof(VT13_MouseKey_t));

    /* ---- 5. 键盘位域 → Key_t 格式转换 ----
     * mouse_key.keyboard 是 packed 位域结构体，Key_t 是独立联合体
     * 两者均为 16bit，通过 uint16_t 指针直接复制 */
    *(uint16_t *)&vt13_rc[TEMP].key[KEY_PRESS] = *(uint16_t *)&vt13_rc[TEMP].mouse_key.keyboard;

    /* ---- 6. ctrl / shift 组合键传播 ----
     * 若 ctrl 按下：当前键盘状态完整复制到 KEY_PRESS_WITH_CTRL 槽位
     * 若 ctrl 松开：清零 KEY_PRESS_WITH_CTRL 槽位（所有组合键视为释放）
     * shift 同理。
     * 这样上层只需检查 key_count[KEY_PRESS_WITH_CTRL][Key_X] 即可知道
     * Ctrl+X 是否被按下，无需每次手动判断 ctrl 状态 */
    if (vt13_rc[TEMP].key[KEY_PRESS].ctrl)
        vt13_rc[TEMP].key[KEY_PRESS_WITH_CTRL] = vt13_rc[TEMP].key[KEY_PRESS];
    else
        memset(&vt13_rc[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (vt13_rc[TEMP].key[KEY_PRESS].shift)
        vt13_rc[TEMP].key[KEY_PRESS_WITH_SHIFT] = vt13_rc[TEMP].key[KEY_PRESS];
    else
        memset(&vt13_rc[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    /* ---- 7. 键盘按键上升沿检测 & 计次 ----
     * 遍历 16 个键位（i=位索引, j=位掩码），对比 TEMP（当前帧）与 LAST（上一帧）：
     *
     *   - 普通按键：当前按下 && 上次未按 && 当前 ctrl/shift 均未按下 → key_count[KEY_PRESS][i]++
     *     条件中排除 ctrl/shift 是为了避免：按住 Ctrl 时 'C' 键既触发普通计数又触发组合计数
     *   - ctrl 组合：组合键当前按下 && 上次未按 → key_count[KEY_PRESS_WITH_CTRL][i]++
     *     仅当 ctrl 本身也按下时 key_with_ctrl 才非零（由步骤 6 保证）
     *   - shift 组合：同上
     *
     * 跳过 i=4(shift) 和 i=5(ctrl)：这两个修饰键自身不计次，只用于组合判断 */
    uint16_t key_now   = vt13_rc[TEMP].key[KEY_PRESS].keys;
    uint16_t key_last  = vt13_rc[LAST].key[KEY_PRESS].keys;
    uint16_t key_with_ctrl       = vt13_rc[TEMP].key[KEY_PRESS_WITH_CTRL].keys;
    uint16_t key_with_shift      = vt13_rc[TEMP].key[KEY_PRESS_WITH_SHIFT].keys;
    uint16_t key_last_with_ctrl  = vt13_rc[LAST].key[KEY_PRESS_WITH_CTRL].keys;
    uint16_t key_last_with_shift = vt13_rc[LAST].key[KEY_PRESS_WITH_SHIFT].keys;

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++) {
        if (i == 4 || i == 5) // 跳过 ctrl(4) 和 shift(5) 修饰键本身
            continue;
        // 普通按键上升沿：当前按下 && 上次未按 && ctrl/shift 均未激活
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            vt13_rc[TEMP].key_count[KEY_PRESS][i]++;
        // ctrl 组合键上升沿：组合状态当前有效 && 上次无效
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            vt13_rc[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // shift 组合键上升沿：组合状态当前有效 && 上次无效
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            vt13_rc[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    /* ---- 8. 更新物理按钮状态（toggle 标志 & 独立计数器）---- */
    VT13UpdateButtonStatus();

    /* ---- 9. 保存当前帧快照到 LAST ----
     * TEMP 完整拷贝至 LAST，作为下一帧边沿检测的参考基准
     * 注意：key_count / button_status 也随之拷贝，保证下一帧 LAST 中的
     * 计数值和 _last 字段与本帧一致 */
    memcpy(&vt13_rc[LAST], &vt13_rc[TEMP], sizeof(VT13_RC_t));
    return 1u;
}

/**
 * @brief 串口接收完成回调：解析成功后喂狗
 */
static void VT13RxCallback(void)
{
  if (VT13Decode(vt13_usart->recv_buff, VT13_FRAME_SIZE)) {
    DaemonReload(vt13_daemon);
  }
}

/**
 * @brief 初始化 VT13 遥控器，注册串口实例和守护进程
 * @param huart  遥控器所使用的 UART 外设句柄指针（如 &huart6）
 * @return       指向内部 VT13_RC_t 数据的指针
 */
VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart)
{
    /* 注册串口实例 */
    USART_Init_Config_s usart_conf = {
        .usart_handle    = huart,
        .recv_buff_size  = VT13_FRAME_SIZE,
        .module_callback = VT13RxCallback,
    };
    vt13_usart = USARTRegister(&usart_conf);

    /* 注册守护进程：遥控器约 14ms 发一帧，100ms 未收到视为离线 */
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 70,           // 30 * 10ms = 300ms 超时
        .init_count   = 0,
        .callback     = VT13LostCallback,
        .owner_id     = NULL,
    };
    vt13_daemon = DaemonRegister(&daemon_conf);

    /* 初始化遥控器数据结构（静态变量已零初始化，此处显式清零以确保） */
    memset(vt13_rc, 0, sizeof(vt13_rc));

    vt13_init_flag = 1;
    return vt13_rc;
}

uint8_t VT13RemoteIsOnline(void)
{
    if (vt13_init_flag && vt13_daemon != NULL) {
        return DaemonIsOnline(vt13_daemon);
    }
    return 0u;
}
