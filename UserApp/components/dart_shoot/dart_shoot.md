# dart_shoot — 飞镖发射机构组件

## 机构组成

| 执行器 | 型号 | 数量 | 作用 | 闭环 |
|--------|------|------|------|------|
| 同步带电机 | M3508 | 2 | 通过同步带把装载飞镖的滑块拉下来，拉到扳机的位置 | 速度环 |
| 扳机调整电机 | M3508 | 1 | 调整扳机的机械位置（换弹/调整时使用，不参与发射流程） | 速度环 |
| 扳机舵机 | PWM 舵机 | 1 | 把扳机拉下来，释放滑块，滑块飞出并发射飞镖 | 位置（角度） |

> 三个 3508 全部使用速度环：速度环参考值与反馈值单位均为 **度/秒（deg/s）**，即 `dji_motor` 中的
> `measure.speed_aps`（换算关系见 `Modules/general_def.h` 的 `RPM_2_ANGLE_PER_SEC`）。

## 工作流程

```
                  CMD_HOME                 CMD_PULL                 CMD_FIRE
  STOPPED ──► IDLE ──────► HOMING ──► IDLE ──────► PULLING ──► READY ──────► RELEASING
               ▲                                                                   │
               │                                              servo_release_time_ms │
               └───────────────────── RELOADING ◄───────────────────────────────────┘
```

1. **HOMING（回零）**：同步带沿拉滑块的反方向低速运动，直到电流超过阈值或底层 PID 报堵转
   （顶到机械限位），把该位置记为滑块零点。回零不是必须的：拉滑块的到位判定以「本次拉滑块的起点」
   为基准，未回零也能工作，但回零后 `feed.pull_position` 才有统一的参考。
2. **PULLING（拉滑块）**：两个同步带电机以速度环输出 `pull_direction * pull_speed`，
   把滑块往下拉。满足下面任意一条即认为到位：
   - 行程到位：`|当前总角度 - 起点总角度| + 容差 >= pull_travel`（推荐，需实测标定）；
   - 堵转到位：平均电流超过 `pull_current_threshold` 或 PID 报堵转，并连续保持 `pull_stall_time_ms`。
3. **READY（就位）**：滑块被扳机挡住，同步带速度给定 0，等待击发指令。
4. **RELEASING（击发）**：舵机转到 `servo_release_angle` 把扳机拉下来，滑块飞出、飞镖发射，
   保持 `servo_release_time_ms`。
5. **RELOADING（复位）**：舵机回 `servo_lock_angle`，同步带反向以 `reload_speed` 把滑块收回起点，
   到位（或堵转、或超时）后回到 IDLE，可以开始下一次发射。

超时（拉滑块/回零/复位）与电机离线都会进入 **ERROR** 状态：同步带停止输出、舵机回锁止位，
需要上层下发 `DART_SHOOT_CMD_RESET` 才能回到 IDLE。

## API

```c
DartShootInstance *dart = DartShootInit(&dart_shoot_init_config);  // RobotInit()

DartShootSetMode(dart, DART_SHOOT_MODE_AUTO);      // 切换工作模式
DartShootSendCmd(dart, DART_SHOOT_CMD_HOME);       // 一次性指令：回零/拉滑块/击发/复位/清异常
DartShootSetManualOutput(dart, 2000.0f, 0.0f, 45.0f);  // 手动模式：同步带速度 / 扳机电机速度 / 舵机角度
DartShootSetServoAngle(dart, 80.0f);               // 直接设置舵机角度（调试用）

DartShootTask(dart);                               // RobotTask() 中 1kHz 调用
```

- 上层通过 `dart->ctrl_cmd`（`DartShoot_Ctrl_Cmd_s`）下发命令，通过 `dart->feed`（`DartShoot_Feed_s`）
  读取状态：当前状态、异常码、是否回零、是否就位、滑块位置、拉滑块/击发耗时、击发次数等。
- 一次性指令 **只需在需要的时刻下发一次**，组件执行后会自动把 `ctrl_cmd.cmd` 清为
  `DART_SHOOT_CMD_NONE`；不要在每个控制周期重复下发同一条指令。
- 模式语义：
  - `DART_SHOOT_MODE_STOPPED`：电机不输出（`DJIMotorStop`），舵机回锁止位，上电默认状态；
  - `DART_SHOOT_MODE_MANUAL`：直接把 `ctrl_cmd` 中的速度和舵机角度下发给执行器，用于调试；
  - `DART_SHOOT_MODE_AUTO`：由状态机完成回零-拉滑块-击发-复位流程。

## 参数标定要点（robot 层 robot_config.h）

- `pull_direction`：拉滑块时同步带电机的转向，实测确定；回零/复位方向取其反方向。
- `pull_travel`：滑块从起点到扳机位的行程，单位是**电机总角度**（`measure.total_angle`）：

$$
\text{电机总角度} = \frac{\text{滑块行程(mm)}}{\text{同步带轮周长(mm)}} \times 360 \times \text{减速比}
$$

  M3508 编码器在转子侧，减速比约 19.2，即输出轴转一圈约 \(19.2 \times 360 = 6912\) 度。
  调试方法：进入手动测试模式，用左摇杆把滑块拉到扳机位，读取日志里的 `pos` 即为 `pull_travel`。
- `pull_current_threshold`：到位/堵转判定的电流阈值，3508 反馈的 `real_current` 是 C620 原始值
  （约 ±16384 对应 ±20A），默认 6000 约等于 7.3A。
- `servo_lock_angle` / `servo_release_angle`：舵机的锁止位与释放位角度，用手动测试模式的拨轮扫描确定，
  再用右摇杆增量微调找到准确位置。角度到脉宽的映射由 `servo_min_pulse_s` / `servo_max_pulse_s` /
  `servo_period_s` 决定，舵机 PWM 周期必须与定时器配置一致（组件会在运行时重设 ARR 与 CCR）。
