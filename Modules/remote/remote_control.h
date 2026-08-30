/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

#include "main.h"
#include "usart.h"

/* ----------------------- Enum Definitions -------------------------------- */

// 1. 用于遥控器数据读取数组的索引
typedef enum {
    TEMP = 0,
    LAST = 1,
    RC_DATA_NUM // 自动等于2
} RCDataIdx_e;

// 2. 键盘互斥修饰状态枚举
typedef enum {
    KEY_PRESS_NORMAL = 0,       // 无任何修饰键按下
    KEY_PRESS_WITH_CTRL,        // 仅按下 Ctrl
    KEY_PRESS_WITH_SHIFT,       // 仅按下 Shift
    KEY_PRESS_WITH_CTRL_SHIFT,  // 同时按下 Ctrl 和 Shift
    KEY_PRESS_STATE_NUM         // 状态总数 (自动等于4)，用于数组长度
} KeyPressState_e;

// 3. 遥控器拨杆状态枚举
typedef enum {
    RC_SW_OFF  = 0,  // 遥控器断连 (或无信号)
    RC_SW_UP   = 1,  // 开关向上
    RC_SW_DOWN = 2,  // 开关向下
    RC_SW_MID  = 3   // 开关中间
} RCSwitchState_e;

// 4. 键盘按键索引枚举 (替代原有的宏定义)
typedef enum {
    KEY_W = 0,
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_NUM_TOTAL   // 自动等于16，用于数组长度
} KeyIndex_e;

/* ----------------------- Macros ------------------------------------------ */

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

// 三个判断开关状态的宏 (配合枚举使用)
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s)  ((s) == RC_SW_MID)
#define switch_is_up(s)   ((s) == RC_SW_UP)
#define switch_is_off(s)  ((s) == RC_SW_OFF)

/* ----------------------- Data Struct ------------------------------------- */

// 位域结构体 (极大提升解析速度，这部分保持不变)
typedef union
{
    struct
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys;
} Key_t;

typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t dial;      // 侧边拨轮

        uint8_t switch_left;  // 左侧开关 (可强转为 RCSwitchState_e)
        uint8_t switch_right; // 右侧开关 (可强转为 RCSwitchState_e)
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    // 键盘状态数组，大小为 4 (对应 4 种互斥的修饰键状态)
    Key_t key[KEY_PRESS_STATE_NUM];

    // 按键触发计数器 [修饰键状态][按键索引]
    uint8_t key_count[KEY_PRESS_STATE_NUM][KEY_NUM_TOTAL];

    // 鼠标按键计数器 [0]:左键 [1]:右键
    uint8_t mouse_count[2];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

#endif