new_RC_VT13
<p align='right'>2026-02-28</p>

```
/************************* 发射机 VT13 ****************************
 *                                                                 *
 *   -----------------------------------------------------         *
 *   |  S1 拨杆                                  暂停键 |         *
 *   |  (上/中/下)                                fn_1  |         *
 *   |                                           fn_2  |         *
 *   |                                           扳机  |         *
 *   |                                                   |         *
 *   |    | ^ |                                | ^ |     |         *
 *   |    | 2 | 左摇杆                     右摇杆 | 1 |     |         *
 *   | ---     ---                          ---     ---  |         *
 *   |<     3     >                        <     0     >|         *
 *   | ---     ---                          ---     ---  |         *
 *   |    |   |                                |   |     |         *
 *   |    |   |                                |   |     |         *
 *   |                                                   |         *
 *   -----------------------------------------------------         *
 *                                                                 *
 ************************ 串口 / 帧参数 ***************************
 *                                                                 *
 *  波特率        921600                                          *
 *  帧间隔        ~14ms                                           *
 *  帧总长        21 字节                                          *
 *  帧头          0xA9 0x53                                       *
 *  CRC           末尾 2 字节，CRC-16/CCITT-FALSE                  *
 *                poly=0x1021, init=0xFFFF, 无反转/无最终XOR       *
 *                                                                 *
 ******************************************************************
 *                                                                 *
 *********************** 有效载荷布局 *******************************
 *                                                                 *
 *  buf[2..9]  — 摇杆 / 功能键 / 拨轮 / 扳机（8 字节位域）         *
 *    bits [0..10]    ch_0  : 右摇杆水平（11bit，范围 364~1684）   *
 *    bits [11..21]   ch_1  : 右摇杆垂直（11bit）                  *
 *    bits [22..32]   ch_2  : 左摇杆垂直（11bit）                  *
 *    bits [33..43]   ch_3  : 左摇杆水平（11bit）                  *
 *    bits [44..45]   s1    : 左拨杆（2bit）                       *
 *    bits [46]       pause : 暂停键（1bit，1=按下）               *
 *    bits [47]       fn_1  : 功能键 1（1bit，1=按下）             *
 *    bits [48]       fn_2  : 功能键 2（1bit，1=按下）             *
 *    bits [49..59]   dial  : 拨轮（11bit，范围 364~1684）         *
 *    bits [60]       trigger: 扳机（1bit，1=按下）                *
 *    bits [61..63]   保留（3bit，对齐至 8 字节）                  *
 *                                                                 *
 *  buf[10..18] — 鼠标 + 键盘（9 字节，可整体 memcpy）             *
 *    buf[10..11]   mouse.x        int16_t  小端，有符号           *
 *    buf[12..13]   mouse.y        int16_t  小端，有符号           *
 *    buf[14..15]   mouse.z        int16_t  小端，有符号（滚轮）    *
 *    buf[16]       mouse_buttons  uint8_t                          *
 *                  [1:0]=press_l, [3:2]=press_r, [5:4]=press_m    *
 *                  值 0=未按下, 1=按下                             *
 *    buf[17..18]   keyboard       uint16_t 小端，位域              *
 *                  每 bit 对应一个按键，定义见键盘信息              *
 *                                                                 *
 *  buf[19..20] — CRC-16/CCITT-FALSE                               *
 *                                                                 *
 ******************************************************************
 *                                                                 *
 ************************ 摇杆 / 拨轮量程 ***************************
 *                                                                 *
 *  原始 ADC     11bit，范围 [364, 1684]                           *
 *  中值         1024                                               *
 *  有符号值     减去 1024 后范围 [-660, +660]                     *
 *  转换宏       VT13_CH_TO_SIGNED(raw) = raw - 1024               *
 *                                                                 *
 ******************************************************************
 *                                                                 *
 ************************** 键盘信息 ********************************
 *                                                                 *
 *  keyboard 位域（uint16_t，小端），与 Key_t 联合体布局一致：      *
 *                                                                 *
 *    Bit 0  : W 键         Bit 8  : R 键                          *
 *    Bit 1  : S 键         Bit 9  : F 键                          *
 *    Bit 2  : A 键         Bit 10 : G 键                          *
 *    Bit 3  : D 键         Bit 11 : Z 键                          *
 *    Bit 4  : Shift 键     Bit 12 : X 键                          *
 *    Bit 5  : Ctrl 键      Bit 13 : C 键                          *
 *    Bit 6  : Q 键         Bit 14 : V 键                          *
 *    Bit 7  : E 键         Bit 15 : B 键                          *
 *                                                                 *
 ******************************************************************/
```

---

## 数据结构

### VT13_RC_t — 遥控器完整数据

```c
typedef struct {
    // 摇杆 / 拨轮 / 功能键（每帧从原始位域解析）
    struct {
        int16_t rocker_l_;   // 左摇杆水平 (ch_3), [-660, +660], 正=右
        int16_t rocker_l1;   // 左摇杆垂直 (ch_2), [-660, +660], 正=上
        int16_t rocker_r_;   // 右摇杆水平 (ch_0), [-660, +660], 正=右
        int16_t rocker_r1;   // 右摇杆垂直 (ch_1), [-660, +660], 正=上
        int16_t dial;        // 拨轮, [-660, +660]
        uint8_t mode_switch; // 左拨杆 s1 (2bit 原始值)
        uint8_t pause;       // 暂停键, 1=按下
        uint8_t fn_1;        // 功能键 1, 1=按下
        uint8_t fn_2;        // 功能键 2, 1=按下
        uint8_t trigger;     // 扳机键, 1=按下
    } rc;

    // 鼠标 + 键盘（整块 memcpy 自原始帧）
    VT13_MouseKey_t mouse_key;

    // 键盘按键跟踪（与旧遥控器一致的计次机制）
    Key_t   key[3];           // [0]=普通, [1]=ctrl组合, [2]=shift组合
    uint8_t key_count[3][16]; // 按键按下次数（上升沿计数）
    uint8_t mouse_count[3][2];// 鼠标按键按下次数

    // 物理按钮状态跟踪（toggle 标志 + 独立计数器）
    struct {
        uint8_t  pause_flag, fn_1_flag, fn_2_flag, trigger_flag;
        uint8_t  mouse_l_flag, mouse_r_flag, mouse_m_flag;
        uint32_t pause_count, fn_1_count, fn_2_count, trigger_count;
        uint32_t mouse_l_count, mouse_r_count, mouse_m_count;
        uint8_t  pause_last, fn_1_last, fn_2_last, trigger_last;
        uint8_t  mouse_l_last, mouse_r_last, mouse_m_last;
    } button_status;
} VT13_RC_t;
```

### Key_t — 键盘位域联合体

```c
typedef union {
    struct {
        uint16_t w : 1, s : 1, a : 1, d : 1;        // bit 0~3
        uint16_t shift : 1, ctrl : 1;                 // bit 4~5
        uint16_t q : 1, e : 1, r : 1, f : 1, g : 1;  // bit 6~10
        uint16_t z : 1, x : 1, c : 1, v : 1, b : 1;  // bit 11~15
    };
    uint16_t keys;  // 整型视角，用于循环位运算
} Key_t;
```

---

## 双缓冲机制（TEMP / LAST）

```
宏定义:  TEMP = 0,  LAST = 1
存储:    static VT13_RC_t vt13_rc[2];
```

每帧处理流程：

```
1. 解析原始帧 → 写入 TEMP.rc / TEMP.mouse_key（覆盖刷新）
2. 键盘位域 → key[KEY_PRESS]（覆盖刷新）
3. ctrl/shift 组合键传播 → key[KEY_PRESS_WITH_CTRL/SHIFT]（覆盖刷新）
4. 边沿检测：TEMP.key[*] vs LAST.key[*] → key_count 递增（累加）
5. 物理按钮更新：TEMP.button_status._last vs TEMP.rc.* → flag/count 更新（累加）
6. memcpy(LAST, TEMP) → LAST 成为下一帧的参考基准
```

关键不变式：
- **TEMP 跨帧持久化**：解码只覆盖 `rc` / `mouse_key` / `key[]` 等协议字段，`key_count` 和 `button_status` 不会被清零，从而实现计数器跨帧累加
- **LAST 是上一帧的完整快照**：步骤 6 保证 LAST 总是上一帧的 TEMP 副本
- **离线时全部清零**：`LostCallback` 调用 `memset(vt13_rc, 0, sizeof(vt13_rc))`，重置双缓冲内所有状态

---

## 按键计次机制

### 键盘按键（16 键 × 3 模式）

基于**上升沿检测**，避免按键持续按下时重复计数：

- **普通按键**：`key_now=1 && key_last=0 && ctrl未按 && shift未按` → `key_count[KEY_PRESS][i]++`
- **Ctrl 组合**：`ctrl按下时 key_with_ctrl=1 && key_last_with_ctrl=0` → `key_count[KEY_PRESS_WITH_CTRL][i]++`
- **Shift 组合**：同上

跳过 shift 自身（bit 4）和 ctrl 自身（bit 5）的计数——它们只作为组合键修饰符使用。

### 物理按键（7 个独立按钮）

`button_status` 中的每个按钮有两个追踪维度：

| 按钮 | toggle 标志 | 计数器 | 行为 |
|------|------------|--------|------|
| pause | `pause_flag` | `pause_count` | 每次按下翻转 flag，递增 count |
| fn_1 | `fn_1_flag` | `fn_1_count` | 同上 |
| fn_2 | `fn_2_flag` | `fn_2_count` | 同上 |
| trigger | `trigger_flag` | `trigger_count` | 同上 |
| 鼠标左 | `mouse_l_flag` | `mouse_l_count` | 同上 |
| 鼠标右 | `mouse_r_flag` | `mouse_r_count` | 同上 |
| 鼠标中 | `mouse_m_flag` | `mouse_m_count` | 同上 |

`_last` 字段保存上一帧状态，用于边缘检测；`_flag` 是 toggle 模式（0/1 翻转），适合用作开关类功能。

---

## API

### VT13RemoteInit

```c
VT13_RC_t *VT13RemoteInit(UART_HandleTypeDef *huart);
```

- 注册串口实例（波特率 921600，帧长 21 字节）
- 注册守护进程（超时 700ms，离线时触发 `VT13LostCallback`）
- 返回指向 `vt13_rc[TEMP]` 的指针，上层通过该指针读取遥控器数据

### VT13RemoteIsOnline

```c
uint8_t VT13RemoteIsOnline(void);
```

- 返回 1 = 在线，0 = 离线 / 尚未初始化

---

## 使用示例

```c
// 初始化
VT13_RC_t *rc = VT13RemoteInit(&huart6);

// 轮询读取
if (VT13RemoteIsOnline()) {
    // 摇杆
    int16_t vx = rc->rc.rocker_l_;   // 左摇杆水平
    int16_t vy = rc->rc.rocker_l1;   // 左摇杆垂直

    // 拨杆档位
    if (switch_left(rc->rc.mode_switch))     { /* 拨杆在上 */ }
    if (switch_middle(rc->rc.mode_switch))   { /* 拨杆在中 */ }
    if (switch_right(rc->rc.mode_switch))    { /* 拨杆在下 */ }

    // 键盘按键次数（普通按下）
    if (rc->key_count[KEY_PRESS][Key_W] > 0) { /* W 键被按过 */ }
    // Ctrl 组合键次数
    if (rc->key_count[KEY_PRESS_WITH_CTRL][Key_C] > 0) { /* Ctrl+C 被按过 */ }
    // 按键持续按下判断（检查当前帧是否按下）
    if (rc->key[KEY_PRESS].w) { /* W 键正在被按住 */ }

    // 物理按钮 toggle 状态
    if (rc->button_status.pause_flag) { /* 暂停键处于激活态 */ }
    if (rc->button_status.fn_1_count > 0) { /* fn_1 被按过 */ }

    // 鼠标
    int16_t mouse_x = rc->mouse_key.mouse.x;
    int16_t mouse_y = rc->mouse_key.mouse.y;
    uint8_t left_pressed = rc->mouse_key.mouse.press_l;
}
```

---

## 与旧遥控器 (remote_control / DT7) 的主要区别

| 项目 | DT7 / SBUS | VT13 |
|------|-----------|------|
| 帧长 | 18 字节 | 21 字节 |
| CRC | 无 | CRC-16/CCITT-FALSE |
| 波特率 | 100000 | 921600 |
| 帧间隔 | ~14ms | ~14ms |
| 数据解析 | SBUS 位拼接 | 64bit 移位提取 |
| 拨杆 | 左右各一个 (switch_left/right) | 仅左拨杆 (mode_switch) |
| 物理按键 | 无独立按键 | pause / fn_1 / fn_2 / trigger |
| 鼠标按键 | press_l / press_r (uint8_t) | press_l / press_r / press_m (2bit) |
| 鼠标滚轮 | 无 | mouse.z |
| 键盘计次 | key_count[3][16] | 相同机制 |
| 双缓冲 | TEMP / LAST | 相同机制 |
| 离线超时 | 100ms | 700ms |
