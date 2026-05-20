# ctrl 模块说明

本文档对应 `infantry_wheel_legged_sjtu/ctrl` 当前代码，主线按 `USE_OCD_CTRL` 图传链路描述。`USE_RC_CTRL` 是保留的旧遥控器链路，不作为当前主要维护路径。

## 模块边界

`ctrl` 是上层输入仲裁层，只负责把遥控器、键盘鼠标和离散状态机转换成 raw 控制目标：

- `chassis_ctrl_cmd`: `target_yaw / vx / wz / roll / leg_length / jump_force`
- `gimbal_ctrl_cmd`: `yaw / pitch / gimbal_mode / chassis_rotate_wz`
- `shoot_ctrl_cmd`: 发射、摩擦轮、拨弹模式
- `robot->robot_mode`: 整车运动模式
- `chassis_ctrl_cmd->chassis_mode`: 底盘执行模式

底盘速度斜坡、yaw 轨迹平滑、LQR 参考滤波、卧倒轮速换算都在 chassis 层完成，ctrl 层不做连续控制闭环。

## 每帧调用顺序

云台板或单板上每帧按下面顺序调用：

1. `JoyStickCtrl(robot)`: 采样 VT13 摇杆、拨轮、模式开关和扳机，写入 `ocd.js_*`。
2. `MouseKeyCtrl(robot)`: 采样 VT13 键鼠，写入 `ocd.mk_*`，并更新跳跃、腿长、掉头、蹭台阶等事件状态。
3. `CtrlSolve(robot)`: 合并摇杆和键鼠意图，推进状态机，调用 `RobotMotionSolve()` 写 raw 指令。
4. `EmergencyHandler(robot)`: 处理急停、倾倒 recovery 和发射禁用。

底盘板不运行输入解算，只通过双板通信接收已经生成的 `chassis_ctrl_cmd`。

## 主要状态

### robot_mode

| 枚举 | 说明 |
| --- | --- |
| `ROBOT_POWER_OFF` | 全局失能 |
| `ROBOT_CHASSIS_FOLLOW` | 站立跟随，底盘 yaw 跟随运动方向/云台方向 |
| `ROBOT_CHASSIS_ROTATE` | 站立小陀螺，LQR 使用旋转状态估计 |
| `ROBOT_CHASSIS_FREE` | 站立自由，底盘对齐云台，右摇杆或 W/S 控前后，左摇杆调 roll/腿长 |
| `ROBOT_CHASSIS_PROSTRATE_FOLLOW` | 卧倒跟随 |
| `ROBOT_CHASSIS_PROSTRATE_ROTATE` | 卧倒小陀螺，chassis 层走 `ChassisProstrate()` 差速轮速 |

### chassis_mode

| 枚举 | 说明 |
| --- | --- |
| `CHASSIS_POWER_OFF` | 底盘失能 |
| `CHASSIS_RECOVERY` | 自起/恢复 |
| `CHASSIS_ON` | 站立 LQR 控制 |
| `CHASSIS_JUMP_READY` | 跳跃准备 |
| `CHASSIS_JUMP_START` | 起跳 |
| `CHASSIS_PROSTRATE` | 卧倒轮速控制 |

### 持久状态

`update_flag` 保存姿态和本帧模式标志：

- `is_on`: `mode_switch` 不在左侧急停位置。
- `is_stand`: 站立/卧倒 toggle，初始为 0，即默认卧倒。
- `is_free`: FREE toggle。
- `is_rotate`: 本帧由拨轮或 Shift 派生。

`ocd` 保存控制层状态：

- `js_*`: 摇杆侧瞬时意图。
- `mk_*`: 键鼠侧瞬时意图。
- `jump`: 跳跃状态机。
- `leg`: 四档腿长预设。
- `stair`: 蹭台阶模式。
- `reverse`: X 掉头后的反向跟随锁存。
- `speed`: 速度常量，当前为 `vx=2.5f`、`wz=15.0f`、`stair=2.2f`、`vault=1.8f`。

## 模式推导

`ApplyOcdMode()` 根据持久状态写入 `robot_mode` 和 `chassis_mode`。优先级如下：

1. `CHASSIS_RECOVERY` 期间直接返回，不覆盖下层恢复流程。
2. `JUMP_ACTIVE` 强制 `CHASSIS_JUMP_START`，写入 `jump_force = JUMP_FORCE`。
3. `JUMP_READY` 强制 `CHASSIS_JUMP_READY`。
4. 根据 `is_stand` 选择 `CHASSIS_ON` 或 `CHASSIS_PROSTRATE`。
5. `is_rotate` 优先于 FREE/FOLLOW。
6. 站立且 `is_free` 时进入 `ROBOT_CHASSIS_FREE`。
7. 其他情况进入站立 FOLLOW 或卧倒 FOLLOW。

当前实现里 rotate 分支条件是 `update_flag.is_stand || update_flag.is_free` 时进入 `ROBOT_CHASSIS_ROTATE`，否则进入 `ROBOT_CHASSIS_PROSTRATE_ROTATE`。也就是说，`is_free` 会影响 rotate 姿态判断；如果后续要让卧倒 FREE toggle 完全无效，这里应该改成只看 `is_stand`。

## 运动解算

`RobotMotionSolve()` 根据最终 `Ctrl_Intent_s` 和 `robot_mode` 写 raw 指令。

### FOLLOW

输入为 `intent.vx / intent.vy`，当前按 m/s 量级使用。

1. `input_mag = sqrt(vx^2 + vy^2)`。
2. 有平移输入时，运动方向为 `atan2(vy, vx) - 90deg`。
3. `follow_err = wrap180(move_angle - offset_angle)`。
4. 如果后向误差 `rear_err = wrap180(follow_err - 180deg)` 更小，则反向行驶并使用 `rear_err`。
5. 如果 `reverse_follow` 激活，底盘 yaw 不跟随掉头，只反向前进，并暂时忽略 `gimbal_yaw_ff`，等待云台完成 180 度转向。
6. `target_yaw = imu_yaw + follow_err + gimbal_yaw_ff`，`reverse_follow` 期间 `gimbal_yaw_ff` 取 0。
7. `vx = input_mag * max(cos(follow_err), 0)^3`。
8. `wz = 0`，`theta_ff = 0`。

无平移输入时，`move_angle` 取 0，目标 yaw 等价于 `imu_yaw - offset_angle + gimbal_yaw_ff`，用于静止回正。

### ROTATE

站立小陀螺：

- `target_yaw = 当前底盘 yaw`，避免 yaw 位置环拉回。
- `wz = rotate_scale * 15.0f` rad/s。
- `vx = 0`，原地转。
- `chassis_ctrl_cmd->is_rotate = 1`，通知 chassis 使用旋转状态估计。
- 云台 yaw 速度环前馈写在这里：
  `gimbal_ctrl_cmd->chassis_rotate_wz = -0.25f * robot->chassis->imu->Gyro[2]`。

卧倒小陀螺：

- `target_yaw = 当前底盘 yaw`。
- `wz = rotate_scale * 15.0f` rad/s，由 chassis 层换算成卧倒差速轮速。
- `chassis_ctrl_cmd->is_rotate` 不置 1，因为卧倒不走站立 LQR 旋转估计。
- 同样写入 `chassis_rotate_wz` 给云台 yaw 速度环前馈。

退出站立/卧倒 rotate 后，`chassis_rotate_wz` 自动清零。

### FREE

FREE 只在站立下生效：

- 双板下底盘 yaw 对齐云台：`follow_err = wrap180(-offset_angle)`。
- 如果后向误差更小，或 `reverse_follow` 激活，则反向行驶。
- `reverse_follow` 激活时底盘 yaw 锁在当前方向，并暂时忽略 `gimbal_yaw_ff`。
- `target_yaw = imu_yaw + follow_err + gimbal_yaw_ff`，`reverse_follow` 期间 `follow_err` 和 `gimbal_yaw_ff` 均取 0。
- `vx = intent.vy`，右摇杆纵向或键盘 W/S 作为前后速度。
- `roll += intent.roll_delta`。
- `leg_length += intent.leg_length_delta`，并限制到 `[LEG_MIN_LENGTH, LEG_MAX_LENGTH]`。
- `wz = 0`，`theta_ff = 0`。

### 卧倒 FOLLOW

卧倒跟随仍使用 m/s 量级输入，ctrl 层只算 raw `vx / target_yaw`。X 掉头的 `reverse_follow` 与站立 FOLLOW 相同：反向行驶、锁底盘 yaw，并暂时忽略 `gimbal_yaw_ff`。实际轮速换算在 chassis 层完成。

## 摇杆输入

### 模式和姿态

- `mode_switch` 左侧：`is_on = 0`，后续由 `EmergencyHandler()` 全局失能。
- `mode_switch` 中间：机器人可运行，但发射关闭。
- `mode_switch` 右侧：发射系统开启。
- `fn_1` 上升沿：站立/卧倒切换。
- `pause` 上升沿：FREE toggle。
- recovery 或 `JUMP_ACTIVE` 期间禁止站立/卧倒切换。

### 小陀螺

- 拨轮绝对值大于 20 时：`js_rotate_scale = dial / 660.0f`，保留方向。
- Shift 由键鼠链路提供；拨轮未触发时，Shift 作为正向满速小陀螺。

### 发射

只有 `mode_switch` 右侧时开启：

- `shoot_mode = SHOOT_ON`
- `friction_mode = FRICTION_ON`
- 扳机按下：请求单发。
- 扳机持续超过 1.0 s：请求连发。

否则发射、摩擦轮、拨弹全部关闭。

### 云台和移动

- 每帧默认 `gimbal_mode = GIMBAL_ON`，自瞄会在 `CtrlSolve()` 中覆盖。
- 右摇杆横向：`yaw += -0.00035f * rocker_r_`。
- 摇杆 yaw 前馈：`js_yaw_ff = yaw_delta * 30.0f`。
- 非站立 FREE 时，右摇杆纵向：`pitch += -0.00006f * rocker_r1`。
- FOLLOW/ROTATE/卧倒下，左摇杆横向：`js_vx = 0.003f * rocker_l_`。
- FOLLOW/ROTATE/卧倒下，左摇杆纵向：`js_vy = 0.003f * rocker_l1`。
- 站立 FREE 下，右摇杆纵向：`js_vy = 0.003f * rocker_r1`。
- 站立 FREE 下，左摇杆横向：`roll_delta = 0.0003f * rocker_l_`，死区 `abs(rocker_l_) > 10`。
- 站立 FREE 下，左摇杆纵向：`leg_length_delta = 0.0000005f * rocker_l1`。

## 键鼠输入

### 每帧清空项

`MouseKeyCtrl()` 每帧清空键鼠瞬时意图：

- `mk_vx / mk_vy`
- `mk_yaw_ff`
- `mk_rotate_scale`
- `mk_shoot`
- `mk_vision`

跳跃、腿长预设、蹭台阶、掉头反向跟随等跨帧状态不会在这里清空。

### 鼠标

- 鼠标 X：`yaw += -mouse.x * 0.002f`，`mk_yaw_ff = yaw_delta * 10.0f`。
- 鼠标 Y：`pitch += -mouse.y * 0.002f`。
- 左键按住且摩擦轮开启：请求单发。
- 左键持续超过 0.3 s：请求连发。
- 右键按住且视觉数据非零：请求自瞄。真正切到 `GIMBAL_VISION` 在 `CtrlSolve()` 中完成。

### WASD

WASD 写入 `mk_vx / mk_vy`。如果摇杆本帧有平移输入，`CtrlSolve()` 优先使用摇杆；否则使用键盘。

速度档：

| 条件 | WASD 速度 |
| --- | --- |
| 跳跃准备或起跳中 | `1.8f` |
| 站立且蹭台阶模式 | `2.2f` |
| 默认 | `2.5f` |

键位：

- `W`: `mk_vy += scale`
- `S`: `mk_vy -= scale`
- `D`: `mk_vx += scale`
- `A`: `mk_vx -= scale`

### 功能键

| 按键 | 触发 | 行为 |
| --- | --- | --- |
| `Shift` | 按住 | 请求正向满速小陀螺，拨轮优先级更高 |
| `X` | 上升沿 | 云台 `yaw += 180deg`，开启反向跟随锁存 |
| `Z` | 上升沿 | 站立时切换蹭台阶模式；进入时腿长切到最高档，退出时恢复原档 |
| `B` | 上升沿 | 请求裁判 UI 全量刷新 |
| `C` | 上升沿 | 超级电容 `BOOST` / `NORMAL` 切换 |
| `Q` | 按住 | `roll_delta -= 0.4f` |
| `E` | 按住 | `roll_delta += 0.4f` |
| `R` | 上升沿 | 腿长档位上升，最高到 3 |
| `F` | 上升沿 | 腿长档位下降，最低到 0 |
| `Ctrl+G` | 上升沿 | 站立/卧倒切换 |
| `Ctrl+V` | 上升沿 | 进入或退出 `JUMP_READY` |
| `V` | 上升沿 | 在 `JUMP_READY` 下进入 `JUMP_ACTIVE` |

腿长表：

| 档位 | 腿长 |
| --- | --- |
| 0 | `0.117f` |
| 1 | `0.20f` |
| 2 | `0.285f` |
| 3 | `0.370f` |

## CtrlSolve 仲裁规则

`CtrlSolve()` 是摇杆和键鼠的最终仲裁层：

1. 平移输入：摇杆有输入时使用 `js_vx/js_vy`，否则使用 `mk_vx/mk_vy`。
2. 小陀螺：拨轮有输入时使用 `js_rotate_scale`，否则使用 `mk_rotate_scale`。
3. yaw 前馈：`gimbal_yaw_ff = js_yaw_ff + mk_yaw_ff`。
4. 自瞄：`mk_vision` 生效时切 `GIMBAL_VISION`，用视觉 yaw/pitch 覆盖手动目标，并清零 `gimbal_yaw_ff`。
5. 发射：摇杆和鼠标请求 OR 合并，连发优先于单发。
6. 调用 `ApplyOcdMode()` 写入 `robot_mode` 和 `chassis_mode`。
7. 消费腿长 pending，写入绝对腿长。
8. 跳跃准备/起跳期间强制 `pitch = 0`。
9. X 掉头反向跟随：云台转过 `160deg` 后自动退出。
10. 调用 `RobotMotionSolve()` 落地 raw 指令。
11. 跳跃 ACTIVE 结束条件：chassis `jump_state` 离开并回到 `IDLE`，或超过 1.2 s 超时。
12. 保存 `rc_data_last`，供下一帧边沿检测。

## 急停和保护

`EmergencyHandler()` 处理最高优先级安全逻辑：

- `rc_data == NULL` 或 `mode_switch` 左侧：全局失能。
- 全局失能时：
  - `robot_mode = ROBOT_POWER_OFF`
  - `gimbal_mode = GIMBAL_POWER_OFF`
  - `chassis_mode = CHASSIS_POWER_OFF`
  - `shoot_mode = SHOOT_OFF`
  - `friction_mode = FRICTION_OFF`
  - `load_mode = LOAD_STOP`
  - 清空 jump 状态和底盘运动记忆
- pitch 绝对值超过阈值时进入 `CHASSIS_RECOVERY`，但 `CHASSIS_PROSTRATE` 卧倒模式除外。
- `mode_switch` 中间时强制关闭发射系统。

恢复阈值当前为：

- 默认 `abs(Pitch) > 13deg`
- 蹭台阶模式 `abs(Pitch) > 13deg`

## 易混点

- `gimbal_yaw_ff` 是底盘 FOLLOW/FREE 的 yaw 预判量，单位 deg，不是云台电机速度环前馈。
- `chassis_rotate_wz` 才是云台 yaw 电机速度环前馈，当前只在站立/卧倒 rotate 模式写入。
- `vx/vy` 在 OCD 链路中按 m/s 量级统一处理，卧倒轮速换算不在 ctrl 层做。
- 跳跃和 recovery 会覆盖普通模式推导，调试模式问题时先看 `chassis_mode`。
