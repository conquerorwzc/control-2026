#ifndef CONTROL_2026_NEW_REMOTE_CONTROL_VT13_H
#define CONTROL_2026_NEW_REMOTE_CONTROL_VT13_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "remote_control.h" // 必须包含老版头文件以使用其枚举和结构体定义

/* ------------------- 帧格式常量 ------------------- */
#define VT13_SOF_1       0xA9u
#define VT13_SOF_2       0x53u
#define VT13_FRAME_SIZE  21u

/* ------------------- 摇杆/拨轮量程 ------------------- */
#define VT13_CH_MID      1024u
#define VT13_CH_MIN      364u
#define VT13_CH_MAX      1684u
#define VT13_CH_TO_SIGNED(raw)  ((int16_t)(raw) - (int16_t)VT13_CH_MID)

/* ------------------- 键位转换宏 ------------------- */
// 这里的宏定义与 remote_control.h 保持兼容，方便代码移植
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

#pragma pack(push, 1)
typedef struct
{
    struct
    {
        int16_t  x;           ///< 鼠标 X 轴移动速度
        int16_t  y;           ///< 鼠标 Y 轴移动速度
        int16_t  z;           ///< 鼠标滚轮移动速度
        uint8_t press_l : 2; ///< 鼠标左键，1=按下
        uint8_t press_r : 2; ///< 鼠标右键，1=按下
        uint8_t press_m : 2; ///< 鼠标中键，1=按下
        uint8_t _pad    : 2; ///< 对齐填充
    } mouse;

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

typedef struct
{
    struct
    {
        /* -------- 摇杆 / 拨轮 -------- */
        int16_t rocker_l_;  
        int16_t rocker_l1;  
        int16_t rocker_r_;  
        int16_t rocker_r1;  
        int16_t dial;          

        /* -------- 拨杆 / 功能键 / 扳机 -------- */
        uint8_t  mode_switch;   
        uint8_t  pause;         
        uint8_t  fn_1;          
        uint8_t  fn_2;          
        uint8_t  trigger;       
    } rc;

    /* -------- 鼠标 + 键盘原始数据（数组化以支持边沿检测）-------- */
    VT13_MouseKey_t mouse_key[2]; ///< [0]=TEMP, [1]=LAST

    /* -------- 键盘按键跟踪（与 remote_control.h 结构完全对齐）-------- */
    Key_t   key[KEY_PRESS_STATE_NUM];             ///< 当前键盘逻辑状态
    Key_t   last_key[KEY_PRESS_STATE_NUM];        ///< 用于内部检测边沿
    uint8_t key_count[KEY_PRESS_STATE_NUM][KEY_NUM_TOTAL]; ///< 键盘按键次数计数

    /* -------- 鼠标按键计数 [0]:左 [1]:右 [2]:中 -------- */
    uint8_t mouse_count[3];

    /* -------- 按钮状态跟踪（VT13 硬件按键） -------- */
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

VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart);
uint8_t VT13RemoteIsOnline(void);

#endif