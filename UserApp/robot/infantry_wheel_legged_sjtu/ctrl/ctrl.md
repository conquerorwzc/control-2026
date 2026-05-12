# ctrl 模块 infantry_wheel_legged_sjtu

## 概述

`ctrl` 模块负责将遥控器（摇杆/拨杆）和键鼠输入映射为机器人底盘、云台、发射机构的控制指令。

模块包含核心函数：

- **`JoyStickCtrl()`** — 遥控器摇杆/拨杆控制
- **`MouseKeyCtrl()`** — 键盘鼠标控制
- **`CtrlSolve()`** — OCD 链路下合并遥控器与键鼠输出，并统一解算运动
- **`EmergencyHandler()`** — 急停与失能保护

---

## 机器人模式总览

| 模式     | 枚举值                 | 说明                                 |
| -------- | ---------------------- | ------------------------------------ |
| 断电     | `ROBOT_POWER_OFF`      | 所有模块失能                         |
| 底盘跟随 | `ROBOT_CHASSIS_FOLLOW` | 底盘自动朝向运动方向，云台独立       |
| 小陀螺   | `ROBOT_CHASSIS_ROTATE` | 底盘持续旋转，正弦速度调制平移       |
| 底盘自由 | `ROBOT_CHASSIS_FREE`   | 底盘对齐云台，直接前后移动/跳跃/调腿 |

---

## 一、遥控器控制 (JoyStickCtrl)

### 1.1 拨杆模式选择

#### 右拨杆 = 中

- 底盘 ON，云台 ON
- 拨轮绝对值 > 20 **或** Shift 按住 → **ROTATE（小陀螺）**；拨轮触发时旋转强度随拨轮绝对值线性增大
- 否则 → **FOLLOW（底盘跟随）**

#### 右拨杆 = 上

- 底盘 ON，云台 ON
- 拨轮绝对值 > 20 **或** Shift 按住 → **ROTATE（小陀螺）**；拨轮触发时旋转强度随拨轮绝对值线性增大
- 否则 → **FREE（自由模式）**
  - 左拨杆 = 中 → `CHASSIS_JUMP_READY`（跳跃准备）
  - 左拨杆 = 上 → `CHASSIS_JUMP_START`（起跳，力 = 55 × JUMP_FORCE）

#### 右拨杆 = 下

- 底盘失能（见急停章节）

### 1.2 发射控制

> 仅在**右拨杆 ≠ 上**时生效。右拨杆 = 上时强制 `SHOOT_OFF` / `FRICTION_OFF` / `LOAD_STOP`。

#### 左拨杆 = 中

- 摩擦轮 ON，拨弹停止（待命状态）

#### 左拨杆 = 上

- 摩擦轮 ON
- 按下持续时间 < 1 秒 → **单发** (`LOAD_1_BULLET`)
- 按下持续时间 ≥ 1 秒 → **连发** (`LOAD_BURSTFIRE`)

#### 左拨杆 = 下

- 发射失能（见急停章节）

### 1.3 摇杆映射

| 摇杆                      | 功能                             | 增益系数 |
| ------------------------- | -------------------------------- | -------- |
| 右摇杆 水平 (`rocker_r_`) | 云台 Yaw 增量 + FOLLOW yaw 前馈  | −0.00035 |
| 右摇杆 垂直 (`rocker_r1`) | 云台 Pitch 增量（仅右拨杆 ≠ 上） | −0.00006 |
| 左摇杆 水平 (`rocker_l_`) | 底盘速度 X 分量（因模式而异）    | 见各模式 |
| 左摇杆 垂直 (`rocker_l1`) | 底盘速度 Y 分量（因模式而异）    | 见各模式 |
| 拨轮 (`dial`)             | 绝对值 > 20 触发小陀螺，并缩放 `wz` 强度 | 0~800    |

### 1.4 各模式底盘解算

#### FOLLOW 模式（双板）

1. 左摇杆输入 → `(vx, vy)`，增益 **0.0045**
2. 有输入时：
   - `follow_err = atan2(vy, vx) − 90° − offset_angle`
   - 误差归一化到 ±180°
   - 倒车优化：`|err| > 90°` 时反向行驶并修正角度
   - `yaw_ff = clamp(云台 yaw 输入增量 × 10.0, ±12°)`（右摇杆/鼠标 X 预判）
   - `target_yaw = IMU_yaw + follow_err + yaw_ff`
   - 对齐衰减：`vx *= cos³(follow_err)`
3. 无输入时：
   - `target_yaw = IMU_yaw − offset_angle + yaw_ff`（静止回正 + 云台输入预判）
4. 速度限幅：±2.50 m/s
5. `theta_ff = 0`（前馈关闭）

#### ROTATE 模式（小陀螺）

1. `target_yaw` 对齐当前底盘 yaw，避免 yaw 位置环额外拉回
2. 拨轮触发时：`wz = 800 × clamp(|dial| / 660, 0, 1)`
3. 仅 Shift 触发且拨轮未偏移时：`wz = 800`
4. `vx = 0`，原地小陀螺

> **注意**：这里使用 `|dial|`，只改变小陀螺强度，不改变默认旋转方向。

#### FREE 模式

| 操作      | 摇杆        | 增益                | 说明                             |
| --------- | ----------- | ------------------- | -------------------------------- |
| 前进/后退 | 右摇杆 垂直 | 0.003               | 经 ramp_controller 平滑          |
| Roll 倾斜 | 左摇杆 水平 | 0.0004（死区 > 10） | 直接角度指令                     |
| 腿长调节  | 左摇杆 垂直 | 0.0000005           | 限幅 [LEG_MIN, LEG_MAX]          |
| 底盘朝向  | —           | —                   | 单板：右摇杆积分；双板：对齐云台 |

---

## 二、键鼠控制 (MouseKeyCtrl, OCD)

当前 `USE_OCD_CTRL` 下，`MouseKeyCtrl()` 不是完整控制闭环入口。它只读取 VT13 的 `mouse_key`，把键鼠意图叠加到共享变量 `intent_shared`，或设置 `mouse_fire` / `mouse_burst` / `mk_request_vision` / `mk_set_leg_length` 这类 flag。真正的自瞄覆盖、开火合并、腿长写入、运动解算和 `rc_data_last` 更新都在 `CtrlSolve()` 里完成。

调用顺序必须是：

1. `JoyStickCtrl()`：清空 `intent_shared`，写入遥控器意图，设置机器人模式。
2. `MouseKeyCtrl()`：把键鼠意图叠加到 `intent_shared`，设置键鼠相关 flag。
3. `CtrlSolve()`：合并两路输入，调用 `RobotMotionSolve()`。

### 2.1 帧内状态处理

每帧进入 `MouseKeyCtrl()` 时会清零：

- `mouse_fire = 0`
- `mouse_burst = 0`
- `mk_request_vision = 0`

它不会清空 `intent_shared`，因为该变量已经由本帧的 `JoyStickCtrl()` 清零并写入遥控器输入；键鼠输入在此基础上做加性叠加。`mk_set_leg_length` 也不会在这里清零，它由 `CtrlSolve()` 消费后清零。

### 2.2 鼠标操作

| 操作            | 当前行为                                      | 最终生效位置  |
| --------------- | --------------------------------------------- | ------------- |
| 鼠标 X 移动     | `gimbal_ctrl_cmd->yaw += -mouse.x * 0.002`，并写入 `gimbal_yaw_ff` | 本函数直接累加 |
| 鼠标 Y 移动     | `gimbal_ctrl_cmd->pitch += -mouse.y * 0.002`  | 本函数直接累加 |
| 左键按住        | 摩擦轮开启时置 `mouse_fire = 1`               | `CtrlSolve()` |
| 左键长按 > 0.3s | 摩擦轮开启时置 `mouse_burst = 1`              | `CtrlSolve()` |
| 右键按住        | 视觉数据非零时置 `mk_request_vision = 1`      | `CtrlSolve()` |

左键松开时会刷新 `mouse_trigger_time`，所以下一次按下从 0 开始计时。开火模式在 `CtrlSolve()` 中与遥控器扳机 OR 合并：连发优先，其次单发，否则停止拨弹。

右键自瞄不是在 `MouseKeyCtrl()` 内直接改云台模式；它只设置请求 flag。`CtrlSolve()` 看到 `mk_request_vision` 后才把云台切到 `GIMBAL_VISION`，并用视觉 yaw/pitch 覆盖前面累加的手动云台输入，同时清零 `gimbal_yaw_ff`。

### 2.3 移动键 (WASD)

WASD 直接叠加到 `intent_shared.vx/vy`。比例由当前 `robot->robot_mode` 决定：

| 当前模式                         | `wasd_scale` | 单位/含义                         |
| -------------------------------- | ------------ | --------------------------------- |
| `ROBOT_CHASSIS_FOLLOW`           | `2.0f`       | 平衡站立速度量级，约 m/s          |
| `ROBOT_CHASSIS_PROSTRATE_FOLLOW` | `660.0f`     | 卧倒轮速/摇杆原始量级             |
| `ROBOT_CHASSIS_FREE` 且站立      | `2.0f`       | 平衡站立速度量级，约 m/s          |
| `ROBOT_CHASSIS_FREE` 且卧倒      | `660.0f`     | 卧倒轮速/摇杆原始量级             |
| 其他模式                         | `0.0f`       | 不接受 WASD 平移输入              |

| 按键 | 写入动作                         |
| ---- | -------------------------------- |
| `W`  | `intent_shared.vy += wasd_scale` |
| `S`  | `intent_shared.vy -= wasd_scale` |
| `D`  | `intent_shared.vx += wasd_scale` |
| `A`  | `intent_shared.vx -= wasd_scale` |

这里不做对角线归一化。若同时按多个方向，`vx/vy` 会按分量直接相加，后续由 `RobotMotionSolve()` 按当前机器人模式解释。

### 2.4 功能键

| 按键     | 触发方式 | 当前行为                                                                 |
| -------- | -------- | ------------------------------------------------------------------------ |
| `X`      | 上升沿   | `gimbal_ctrl_cmd->yaw += 180.0f`                                         |
| `C`      | 上升沿   | 在 `BOOST` 和 `NORMAL` 间切换超级电容控制命令                            |
| `Q`      | 按住     | `intent_shared.roll_delta -= 0.4f`                                       |
| `E`      | 按住     | `intent_shared.roll_delta += 0.4f`                                       |
| `R`      | 上升沿   | `leg_step` 最多加到 2，并请求写入 `LEG_TABLE[leg_step]`                  |
| `F`      | 上升沿   | `leg_step` 最少减到 0，并请求写入 `LEG_TABLE[leg_step]`                  |
| `Fn1`    | 上升沿   | 热切换倒立摆平衡态和 `CHASSIS_PROSTRATE`，初始为 `CHASSIS_PROSTRATE`     |
| `Ctrl+G` | 上升沿   | 与 `Fn1` 相同，热切换倒立摆平衡态和 `CHASSIS_PROSTRATE`                 |
| `Ctrl+V` | 上升沿   | 在正常状态和 `CHASSIS_JUMP_READY` 之间切换                              |
| `V`      | 上升沿   | 仅在 `CHASSIS_JUMP_READY` 锁存时触发 `CHASSIS_JUMP_START` 并写入 `JUMP_FORCE` |
| `Shift`  | 按住     | 不在 `MouseKeyCtrl()` 中处理；由 `JoyStickCtrl()` / `CtrlSolve()` 参与小陀螺判定 |

三档腿长表为：

| 档位 | 腿长目标 |
| ---- | -------- |
| 0    | `0.20f`  |
| 1    | `0.285f` |
| 2    | `0.370f` |

### 2.5 倒立摆/卧倒热切换

OCD 控制层使用本地姿态锁存 `mk_stand_mode` 作为唯一有效姿态源，初始值为 0，即 `CHASSIS_PROSTRATE`。VT13 的 `fn_1_flag` 不再直接决定姿态。

`Fn1` 上升沿和 `Ctrl+G` 上升沿都会对 `mk_stand_mode` 取反：如果当前是倒立摆平衡态，则切到 `CHASSIS_PROSTRATE`；如果当前是卧倒，则切回倒立摆平衡态。因此按 `Ctrl+G` 热切换后，再按 `Fn1` 仍然可以切到另一个形态。

切换时会清除 `CHASSIS_JUMP_READY` 锁存，并按当前 `pause_flag` / `dial` / `Shift` 重新计算 `robot_mode`。`CHASSIS_RECOVERY` 和已经起跳后的 `CHASSIS_JUMP_START` 期间不响应形态切换。

### 2.6 跳跃键鼠流程

`Ctrl+V` 第一次按下后，OCD 控制层会锁存 `mk_jump_ready = 1`，每帧把底盘保持在 `CHASSIS_JUMP_READY`，并把机器人运动模式保持为 `ROBOT_CHASSIS_FOLLOW`。这会阻止 `JoyStickCtrl()` 下一帧把底盘模式覆盖回正常站立/卧倒，同时准备跳跃期间走 FOLLOW 的 yaw 解算，不再挂在 FREE 下。

在 `CHASSIS_JUMP_READY` 下再次按 `Ctrl+V`，会清除准备锁存，并立即按当前 `mk_stand_mode` / `pause_flag` / `dial` / `Shift` 恢复正常模式。

在 `CHASSIS_JUMP_READY` 下按单独的 `V`，会切到 `CHASSIS_JUMP_START`，写入 `jump_force = JUMP_FORCE`。控制层会保持起跳状态直到 chassis 的 `jump_state` 离开过 `IDLE` 后再次回到 `IDLE`；如果控制侧读不到真实跳跃状态，则 1.2 秒后兜底恢复正常模式。

### 2.7 当前没有做的事

- `MouseKeyCtrl()` 不设置 `gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON`。
- `MouseKeyCtrl()` 不调用 `RobotMotionSolve()`。
- `MouseKeyCtrl()` 不更新 `rc_data_last`。
- 当前 OCD 键鼠没有实现飞坡切换或 `Ctrl+Z` 速度档切换。

---

## 三、急停与保护 (EmergencyHandler)

### 3.1 触发条件

| 条件                          | 效果                                      |
| ----------------------------- | ----------------------------------------- |
| 左右拨杆**同时拨下**          | 全局断电 `ROBOT_POWER_OFF`，所有模块失能  |
| 遥控器**断连** (`switch_off`) | 全局断电 `ROBOT_POWER_OFF`                |
| 右拨杆**拨下**                | 底盘单独失能 `CHASSIS_POWER_OFF`          |
| 左拨杆**拨下**                | 发射单独失能 `SHOOT_OFF` / `FRICTION_OFF` |
| IMU Pitch 绝对值 > 13°        | 底盘进入自起 `CHASSIS_RECOVERY`           |

> IMU 倾角保护由宏 `robot_lost_control` 定义：`abs(robot->chassis->imu->Pitch) > 13.0f`

### 3.2 断电时重置

断电时会清零斜坡规划器状态：

- `chassis_ramp.planning_v = 0`
- `chassis_ramp.expected_a = 0`

---

## 四、Ramp Controller 参数

| 参数               | 值       | 说明         |
| ------------------ | -------- | ------------ |
| `max_v`            | 1.0 m/s  | 规划最大速度 |
| `max_accel`        | 2.0 m/s² | 最大加速度   |
| `accel_base_speed` | 0.3 m/s  | 加速基础速度 |
| `max_decel`        | 4.0 m/s² | 最大减速度   |
| `min_decel`        | 1.0 m/s² | 最小减速度   |
| `decel_base_speed` | 0.8 m/s  | 减速基础速度 |

---

## 五、Pitch 软件限位

云台 Pitch 在所有模式下统一限位：

- 上限：`PITCH_MAX_ANGLE`
- 下限：`PITCH_MIN_ANGLE`

> 具体值定义于 `robot_config.h`。

---

## 六、键位速查

### 遥控器布局

#### 摇杆
- **FOLLOW**

|            | 水平 (↔)        | 垂直 (↕)        |
| ---------- | ----------- | ----------- |
| **左摇杆** | 底盘y轴速度分量 | 底盘x轴速度分量 |
| **右摇杆** | 云台 Yaw        | 云台 Pitch      |

- **FREE**

|            | 水平 (↔)  | 垂直 (↕)        |
| ---------- | --------- | --------------- |
| **左摇杆** | 底盘 Roll | 底盘高度        |
| **右摇杆** | 云台 Yaw  | 底盘x轴速度分量 |

#### 拨杆

|            | 上                | 中         | 下       |
| ---------- | ----------------- | ---------- | -------- |
| **右拨杆** | FREE + 跳跃       | FOLLOW     | 底盘失能 |
| **左拨杆** | 开火（单发/连发） | 摩擦轮待命 | 发射失能 |

> - 左右拨杆**同时拨下** = 急停
> - 遥控器**断连** = 急停

#### 拨轮

| 条件        | 效果           |
| ----------- | -------------- |
| 绝对值 > 20 | 触发小陀螺模式 |

---

### 键鼠布局

#### 移动

| 按键 | 站立 FOLLOW / 站立 FREE | 卧倒 FOLLOW / 卧倒 FREE | 其他模式 |
| ---- | ----------------------- | ----------------------- | -------- |
| `W`  | `vy += 2.0`             | `vy += 660.0`           | 无效     |
| `S`  | `vy -= 2.0`             | `vy -= 660.0`           | 无效     |
| `A`  | `vx -= 2.0`             | `vx -= 660.0`           | 无效     |
| `D`  | `vx += 2.0`             | `vx += 660.0`           | 无效     |

#### 鼠标

| 操作            | 功能                                      |
| --------------- | ----------------------------------------- |
| 鼠标移动 X      | 云台 Yaw 增量（灵敏度 `-0.002`）         |
| 鼠标移动 Y      | 云台 Pitch 增量（灵敏度 `-0.002`）       |
| 左键按住        | 摩擦轮开启时请求单发                      |
| 左键长按 > 0.3s | 摩擦轮开启时请求连发                      |
| 右键按住        | 视觉数据有效时请求自瞄，最终由 `CtrlSolve()` 覆盖云台目标 |

#### 功能键（持续按住）

| 按键    | 功能                                                         |
| ------- | ------------------------------------------------------------ |
| `Shift` | 小陀螺判定输入，由 `JoyStickCtrl()` / `CtrlSolve()` 使用     |
| `Q`     | `roll_delta -= 0.4`                                          |
| `E`     | `roll_delta += 0.4`                                          |

#### 功能键（单次触发）

| 按键     | 功能                                   |
| -------- | -------------------------------------- |
| `X`      | 云台掉头 `yaw += 180°`                 |
| `C`      | 超级电容 `BOOST` / `NORMAL` 切换       |
| `R`      | 腿长档位上升，最高到 `0.370f`          |
| `F`      | 腿长档位下降，最低到 `0.20f`           |
| `Fn1`    | 热切换倒立摆平衡态和 `CHASSIS_PROSTRATE` |
| `Ctrl+G` | 热切换倒立摆平衡态和 `CHASSIS_PROSTRATE` |
| `Ctrl+V` | 进入/退出 `CHASSIS_JUMP_READY`         |
| `V`      | 在 `CHASSIS_JUMP_READY` 下触发起跳     |

> 当前 OCD 键鼠未实现飞坡切换和 `Ctrl+Z` 速度档切换。
