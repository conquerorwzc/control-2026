/**
 * @file new_RC_VT13.h
 * @brief VT13 遥控器接收与解析驱动头文件
 * @note 兼容老版大疆遥控器的键鼠状态机的逻辑 API，实现底层协议无缝切换
 */

#ifndef CONTROL_2026_NEW_REMOTE_CONTROL_VT13_H
#define CONTROL_2026_NEW_REMOTE_CONTROL_VT13_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "remote_control.h" // 必须包含老版头文件以使用其枚举(KEY_PRESS_STATE_NUM等)和结构体定义

/* ------------------- 帧格式常量 ------------------- */
#define VT13_SOF_1       0xA9u      // 帧头1
#define VT13_SOF_2       0x53u      // 帧头2
#define VT13_FRAME_SIZE  21u        // 数据帧总长度：2(帧头) + 8(摇杆拨杆) + 9(键鼠) + 2(CRC16)

/* ------------------- 摇杆/拨轮量程 ------------------- */
#define VT13_CH_MID      1024u      // 摇杆中值
#define VT13_CH_MIN      364u       // 摇杆最小值
#define VT13_CH_MAX      1684u      // 摇杆最大值
// 宏：将 0~2047 的无符号摇杆原始值，转换为以 0 为中心的有符号值 (如 -660 ~ +660)
#define VT13_CH_TO_SIGNED(raw)  ((int16_t)(raw) - (int16_t)VT13_CH_MID)

/* ------------------- 键位转换宏 (对齐老版) ------------------- */
// 对应键盘位域中各个按键的 bit 偏移，用于边沿检测时的掩码计算
#define VT13_KEY_W      (1u << 0)
#define VT13_KEY_S      (1u << 1)
#define VT13_KEY_A      (1u << 2)
#define VT13_KEY_D      (1u << 3)
#define VT13_KEY_SHIFT  (1u << 4)
#define VT13_KEY_CTRL   (1u << 5)
#define VT13_KEY_Q      (1u << 6)
#define VT13_KEY_E      (1u << 7)
#define VT13_KEY_R      (1u << 8)
#define VT13_KEY_F      (1u << 9)
#define VT13_KEY_G      (1u << 10)
#define VT13_KEY_Z      (1u << 11)
#define VT13_KEY_X      (1u << 12)
#define VT13_KEY_C      (1u << 13)
#define VT13_KEY_V      (1u << 14)
#define VT13_KEY_B      (1u << 15)

/* ------------------- 原始键鼠数据结构 ------------------- */
#pragma pack(push, 1) // 强制 1 字节对齐，防止结构体填充导致 memcpy 错位
typedef struct
{
    // 鼠标数据 (共 7 bytes)
    struct
    {
        int16_t  x;           ///< 鼠标 X 轴移动速度 (2 bytes)
        int16_t  y;           ///< 鼠标 Y 轴移动速度 (2 bytes)
        int16_t  z;           ///< 鼠标滚轮移动速度 (2 bytes)
        uint8_t press_l : 2;  ///< 鼠标左键，1=按下
        uint8_t press_r : 2;  ///< 鼠标右键，1=按下
        uint8_t press_m : 2;  ///< 鼠标中键，1=按下
        uint8_t _pad    : 2;  ///< 补齐凑满 1 byte
    } mouse;

    // 键盘数据 (共 2 bytes，16个按键每个占 1 bit)
    struct
    {
        uint16_t w     : 1;
        uint16_t s     : 1;
        uint16_t a     : 1;
        uint16_t d     : 1;
        uint16_t shift : 1;
        uint16_t ctrl  : 1;
        uint16_t q     : 1;
        uint16_t e     : 1;
        uint16_t r     : 1;
        uint16_t f     : 1;
        uint16_t g     : 1;
        uint16_t z     : 1;
        uint16_t x     : 1;
        uint16_t c     : 1;
        uint16_t v     : 1;
        uint16_t b     : 1;
    } keyboard;
} VT13_MouseKey_t;
#pragma pack(pop)

/* ------------------- 遥控器应用层总控结构体 ------------------- */
typedef struct
{
    // 1. 摇杆、拨杆与按键 (解包后的有符号直观数据)
    struct
    {
        /* -------- 摇杆 / 拨轮 -------- */
        int16_t rocker_l_;      ///< 左摇杆水平
        int16_t rocker_l1;      ///< 左摇杆竖直
        int16_t rocker_r_;      ///< 右摇杆水平
        int16_t rocker_r1;      ///< 右摇杆竖直
        int16_t dial;           ///< 左侧拨轮

        /* -------- 拨杆 / 功能键 / 扳机 -------- */
        uint8_t  mode_switch;   ///< 左上角模式拨杆 (1=上, 3=中, 2=下)
        uint8_t  pause;         ///< 暂停键
        uint8_t  fn_1;          ///< 自定义功能键 1
        uint8_t  fn_2;          ///< 自定义功能键 2
        uint8_t  trigger;       ///< 扳机键
    } rc;

    // 2. 鼠标+键盘底层缓冲数据 (用于保存当前帧[0]与上一帧[1])
    VT13_MouseKey_t mouse_key[2]; ///< [0]=TEMP, [1]=LAST

    // 3. 键盘状态分流与统计 (向下兼容核心)
    Key_t   key[KEY_PRESS_STATE_NUM];             ///< 当前按键状态 (分摊到 Normal, Ctrl, Shift, Ctrl+Shift 四个数组中)
    Key_t   last_key[KEY_PRESS_STATE_NUM];        ///< 上一次按键状态 (仅用于内部边沿检测对比)
    uint8_t key_count[KEY_PRESS_STATE_NUM][KEY_NUM_TOTAL]; ///< 按键上升沿触发次数累加器

    // 4. 鼠标按键计数
    uint8_t mouse_count[3];                       ///< [0]:左键 [1]:右键 [2]:中键 的点击次数累加

    // 5. VT13 硬件按钮状态跟踪 (处理短按 Toggle 和 长按)
    struct {
        uint8_t  pause_flag;        
        uint8_t  fn_1_flag;         
        uint8_t  fn_2_flag;         
        uint8_t  trigger_flag;      
        uint8_t  mouse_l_flag;      
        uint8_t  mouse_r_flag;      
        uint8_t  mouse_m_flag;      
        
        uint32_t pause_count;       
        uint32_t fn_1_count;        
        uint32_t fn_2_count;        
        uint32_t trigger_count;     
        uint32_t mouse_l_count;     
        uint32_t mouse_r_count;     
        uint32_t mouse_m_count;     
        
        uint8_t  pause_last;        
        uint8_t  fn_1_last;         
        uint8_t  fn_2_last;         
        uint8_t  trigger_last;      
        uint8_t  mouse_l_last;      
        uint8_t  mouse_r_last;      
        uint8_t  mouse_m_last;      
    } button_status;
} VT13_RC_t;

/* ------------------- 外部接口申明 ------------------- */
VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart);
uint8_t VT13RemoteIsOnline(void);

#endif // CONTROL_2026_NEW_REMOTE_CONTROL_VT13_H