#include "new_RC_VT13.h" // 确保引用的是头文件
#include <stdlib.h>
#include <string.h>
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"

/* ================================================================
 * CRC-16/CCITT-FALSE
 * 多项式 P(x) = x^16 + x^12 + x^5 + 1  (0x1021)
 *  初值 0xFFFF，输入/输出不反转，无最终 XOR
 * ================================================================ */
static const uint16_t vt13_crc16_tab[256] = {
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
 * @brief 内部 CRC 计算函数
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
 * @brief 校验帧尾的两个 CRC 字节
 */
static uint8_t vt13_verify_crc16(const uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2u) return 0u;
    uint16_t expected = vt13_crc16_calc(p_msg, (uint16_t)(len - 2u), 0xFFFFu);
    return ((expected & 0xFFu) == p_msg[len - 2u] &&
            ((expected >> 8u) & 0xFFu) == p_msg[len - 1u]) ? 1u : 0u;
}

/* ------------------- 模块静态全局变量 ------------------- */
static VT13_RC_t        vt13_rc;            // 全局遥控器数据实例
static USARTInstance   *vt13_usart;         // 绑定的串口实例
static DaemonInstance  *vt13_daemon;        // 掉线监控守护进程
static uint8_t          vt13_init_flag = 0; // 初始化完成标志

/* ================================================================
 * 内部核心处理逻辑
 * ================================================================ */

/**
 * @brief 将原始键盘数据按修饰键(Ctrl/Shift)拆分，并执行边沿检测（核心精髓所在）
 * @note  这是保证新老控制逻辑(如Shift+R/Ctrl+C)能够无缝衔接的关键函数
 */
static void VT13UpdateKeyboardMouseStatus(void)
{
    // 1. 获取 16位 纯净按键数据：利用指针强转，将位域结构体转换成 uint16_t 方便进行位运算
    // 注意：此强转依赖于 STM32 的小端序架构（Little-Endian）
    uint16_t raw_keys = *(uint16_t*)&vt13_rc.mouse_key[TEMP].keyboard;

    // 提取当前修饰键的状态
    uint8_t is_ctrl  = (raw_keys & VT13_KEY_CTRL)  ? 1 : 0;
    uint8_t is_shift = (raw_keys & VT13_KEY_SHIFT) ? 1 : 0;

    // 2. 拷贝保存上一次解析好的键盘状态矩阵，并清空当前的矩阵以备分配
    memcpy(vt13_rc.last_key, &vt13_rc.key, sizeof(vt13_rc.key));
    memset(&vt13_rc.key, 0, sizeof(vt13_rc.key));

    // 3. 修饰键分流：NORMAL 始终写入原始按键（保证底盘 WASD 等基础功能永远有数据），
    //    同时按修饰键时额外写入对应通道（供 Shift+R、Ctrl+Shift+V 等组合键使用）
    // 3. 严格互斥的修饰键分流：保证按了修饰键时，NORMAL 通道绝对干净！
    if (is_ctrl && is_shift)
    {
        vt13_rc.key.arr[KEY_PRESS_WITH_CTRL_SHIFT].keys = raw_keys;
    }
    else if (is_ctrl)
    {
        vt13_rc.key.arr[KEY_PRESS_WITH_CTRL].keys = raw_keys;
    }
    else if (is_shift)
    {
        vt13_rc.key.arr[KEY_PRESS_WITH_SHIFT].keys = raw_keys;
    }
    else
    {
        // 🌟 只有在没有任何修饰键按下时，NORMAL 才有资格获取按键状态！
        vt13_rc.key.arr[KEY_PRESS_NORMAL].keys = raw_keys;
    }
    for (uint8_t state = 0; state < KEY_PRESS_STATE_NUM; state++)
    {
        uint16_t now  = vt13_rc.key.arr[state].keys;
        uint16_t last = vt13_rc.last_key[state].keys;

        // 遍历这 16 个按键位
        for (uint16_t i = 0, j = 0x1; i < KEY_NUM_TOTAL; j <<= 1, i++)
        {
            // 修饰键本身不需要触发动作，跳过
            if (i == KEY_CTRL || i == KEY_SHIFT) continue;

            // 如果某位从 0 变成了 1 (即按下瞬间)
            if ((now & j) && !(last & j)) {
                vt13_rc.key_count.arr[state][i]++; // 对应状态通道的按键计数器 +1
            }
        }
    }

    // 5. 鼠标左、右、中键的独立边沿检测
    if (vt13_rc.mouse_key[TEMP].mouse.press_l && !vt13_rc.mouse_key[LAST].mouse.press_l)
        vt13_rc.mouse_count[0]++;
    if (vt13_rc.mouse_key[TEMP].mouse.press_r && !vt13_rc.mouse_key[LAST].mouse.press_r)
        vt13_rc.mouse_count[1]++;
    if (vt13_rc.mouse_key[TEMP].mouse.press_m && !vt13_rc.mouse_key[LAST].mouse.press_m)
        vt13_rc.mouse_count[2]++;
}

/**
 * @brief 更新 VT13 独占硬件开关/按键的 Toggle (切换) 状态
 */
static void VT13UpdateButtonStatus(void)
{
    // 宏函数：当按键按下且上一次未按下时(上升沿)，翻转 flag 并累加 count
    #define UPDATE_TOGGLE(btn, flag, count, last) \
    if (btn && !last) { flag = !flag; count++; }  \
    last = btn;

    UPDATE_TOGGLE(vt13_rc.rc.pause,   vt13_rc.button_status.pause_flag,   vt13_rc.button_status.pause_count,   vt13_rc.button_status.pause_last)
    UPDATE_TOGGLE(vt13_rc.rc.fn_1,    vt13_rc.button_status.fn_1_flag,    vt13_rc.button_status.fn_1_count,    vt13_rc.button_status.fn_1_last)
    UPDATE_TOGGLE(vt13_rc.rc.fn_2,    vt13_rc.button_status.fn_2_flag,    vt13_rc.button_status.fn_2_count,    vt13_rc.button_status.fn_2_last)
    UPDATE_TOGGLE(vt13_rc.rc.trigger, vt13_rc.button_status.trigger_flag, vt13_rc.button_status.trigger_count, vt13_rc.button_status.trigger_last)

    // 鼠标硬件层按键 Toggle 更新
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_l, vt13_rc.button_status.mouse_l_flag, vt13_rc.button_status.mouse_l_count, vt13_rc.button_status.mouse_l_last)
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_r, vt13_rc.button_status.mouse_r_flag, vt13_rc.button_status.mouse_r_count, vt13_rc.button_status.mouse_r_last)
    UPDATE_TOGGLE(vt13_rc.mouse_key[TEMP].mouse.press_m, vt13_rc.button_status.mouse_m_flag, vt13_rc.button_status.mouse_m_count, vt13_rc.button_status.mouse_m_last)
}

/**
 * @brief 核心解包函数，处理来自串口的原始裸流数据
 * @return 成功解析返回 1，否则返回 0
 */
static uint8_t VT13Decode(const uint8_t *buf, uint16_t len)
{
    // 1. 基础异常防护与校验
    if (buf == NULL || len < VT13_FRAME_SIZE) return 0u;
    if (buf[0] != VT13_SOF_1 || buf[1] != VT13_SOF_2) return 0u; // 帧头匹配
    if (!vt13_verify_crc16(buf, VT13_FRAME_SIZE)) return 0u;     // CRC 完整性验证

    // 2. 优雅的位提取操作
    // 利用 uint64 承载连续的 8 个字节 (Byte2~Byte9), 然后统一移位切片，避免了恶心的跨字节拼接
    uint64_t raw64 = 0;
    memcpy(&raw64, buf + 2, sizeof(uint64_t));

    /* -------- 11位通道映射解析 -------- */
    vt13_rc.rc.rocker_r_ = VT13_CH_TO_SIGNED((raw64 >>  0) & 0x7FFu); // 截取[0:10]位
    vt13_rc.rc.rocker_r1 = VT13_CH_TO_SIGNED((raw64 >> 11) & 0x7FFu); // 截取[11:21]位
    vt13_rc.rc.rocker_l1 = VT13_CH_TO_SIGNED((raw64 >> 22) & 0x7FFu); // 截取[22:32]位
    vt13_rc.rc.rocker_l_ = VT13_CH_TO_SIGNED((raw64 >> 33) & 0x7FFu); // 截取[33:43]位

    /* -------- 拨杆与按键解析 -------- */
    vt13_rc.rc.mode_switch = (uint8_t)((raw64 >> 44) & 0x3u);  // [44:45] (2位)
    vt13_rc.rc.pause       = (uint8_t)((raw64 >> 46) & 0x1u);  // [46]
    vt13_rc.rc.fn_1        = (uint8_t)((raw64 >> 47) & 0x1u);  // [47]
    vt13_rc.rc.fn_2        = (uint8_t)((raw64 >> 48) & 0x1u);  // [48]
    vt13_rc.rc.dial        = VT13_CH_TO_SIGNED((raw64 >> 49) & 0x7FFu); // [49:59] (11位)
    vt13_rc.rc.trigger     = (uint8_t)((raw64 >> 60) & 0x1u);  // [60]

    // 3. 键鼠内存刷新
    // 必须先把旧的 TEMP 数据拷贝到 LAST 缓冲，然后再从串口流中读取新数据
    memcpy(&vt13_rc.mouse_key[LAST], &vt13_rc.mouse_key[TEMP], sizeof(VT13_MouseKey_t));
    memcpy(&vt13_rc.mouse_key[TEMP], &buf[10], sizeof(VT13_MouseKey_t)); // 键鼠数据起始偏移量为 10

    // 4. 更新后续应用层需要的业务逻辑状态
    VT13UpdateButtonStatus();        // 更新硬件按键的计次与 flag
    VT13UpdateKeyboardMouseStatus(); // 处理核心业务层面的按键逻辑

    return 1u;
}

/**
 * @brief USART 接收完成回调 (注册到 bsp_usart)
 */
static void VT13RxCallback(void)
{
    // 如果一帧解包成功，就去喂一次守护进程的狗，防止触发离线判定
    if (VT13Decode(vt13_usart->recv_buff, VT13_FRAME_SIZE)) {
        DaemonReload(vt13_daemon);
    }
}

/**
 * @brief 遥控器掉线回调 (由 Daemon 触发)
 */
static void VT13LostCallback(void *id)
{
    // 掉线后安全策略：清空所有缓存数据（防止车疯跑），并尝试重启串口 DMA 接收
    memset(&vt13_rc, 0, sizeof(vt13_rc));
    USARTServiceInit(vt13_usart);
    LOGWARNING("[VT13] remote control lost");
}

/* ================================================================
 * 模块外部接口
 * ================================================================ */

/**
 * @brief 初始化 VT13 遥控器模块
 * @param huart 对应的硬件串口句柄
 * @return 遥控器实例指针
 */
VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart)
{
    // 1. 注册串口 DMA 接收回调服务
    USART_Init_Config_s usart_conf = {
        .usart_handle    = huart,
        .recv_buff_size  = VT13_FRAME_SIZE,
        .module_callback = VT13RxCallback,
    };
    vt13_usart = USARTRegister(&usart_conf);

    // 2. 注册 Daemon 离线监控（假设后台任务周期是1ms，70代表 70ms 没收到包就算掉线）
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 70,
        .callback     = VT13LostCallback,
        .owner_id     = NULL,
    };
    vt13_daemon = DaemonRegister(&daemon_conf);

    // 3. 数据与状态位清空
    memset(&vt13_rc, 0, sizeof(vt13_rc));
    vt13_init_flag = 1;

    return &vt13_rc;
}

/**
 * @brief 以共享串口方式初始化VT13遥控器(用于两个模块共用同一串口的场景)
 *        不注册新串口,而是作为 secondary callback 挂载到已有实例
 */
VT13_RC_t *VT13RemoteInitShared(USARTInstance *shared_instance)
{
    if (shared_instance == NULL)
        while (1); // 传入空指针,死循环报错

    // 1. 挂载 secondary callback,共享 recv_buff
    vt13_usart = USARTAddSecondaryCallback(shared_instance->usart_handle, VT13RxCallback);

    // 2. 注册 Daemon 离线监控
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 70,
        .callback     = VT13LostCallback,
        .owner_id     = NULL,
    };
    vt13_daemon = DaemonRegister(&daemon_conf);

    // 3. 数据与状态位清空
    memset(&vt13_rc, 0, sizeof(vt13_rc));
    vt13_init_flag = 1;

    return &vt13_rc;
}

/**
 * @brief 查询遥控器是否在线
 */
uint8_t VT13RemoteIsOnline(void)
{
    return (vt13_init_flag && vt13_daemon != NULL) ? DaemonIsOnline(vt13_daemon) : 0u;
}