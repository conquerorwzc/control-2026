# dart_shoot — 飞镖发射机构组件（遥操作版）

## 机构组成

| 执行器 | 型号 | 数量 | CAN ID | 作用 | 闭环 |
|--------|------|------|--------|------|------|
| yaw 电机 | M2006 + C610 | 1 | CAN1 / 1 | 瞄准轴 | 位置环（速度环串级） |
| 同步带电机 | M3508 + C620 | 2 | CAN1 / 2、4 | 通过同步带把装载飞镖的滑块拉下来 | 位置环（速度环串级） |
| 扳机位置电机 | M3508 + C620 | 1 | CAN1 / 3 | 调整扳机的机械位置（换弹/标定时使用） | 速度环 |
| 拉扳机舵机 | PWM 舵机 | 1 | TIM1_CH1 (PE9) | 把扳机拉下来，释放滑块 | 角度（脉宽） |

> 单位约定：位置用 `dji_motor` 的 `measure.total_angle`（度），速度用 `measure.speed_aps`（度/秒）。
> M3508 / M2006 的编码器都在减速箱前，所以都是**转子侧**角度：
> 3508 输出轴角度 = 总角度 / 19.2，2006 输出轴角度 = 总角度 / 36。
> 同步带右电机装配相反，配置里 `motor_reverse_flag` 与 `feedback_reverse_flag` 同时取反，
> 因此两个电机的 `total_angle` 方向一致，可以共用一个目标位置。

## 控制逻辑

组件内**没有自动发射流程**，全部由上层（遥控器）按周期驱动。三个档位：

| 档位 | 同步带 | yaw | 扳机位置电机 | 舵机 |
|------|--------|-----|--------------|------|
| `DART_SHOOT_MODE_DISABLED` | 失能 | 失能 | 失能 | 失能（强制） |
| `DART_SHOOT_MODE_AIM` | 位置累加；无指令时**保持**目标位置顶住 | 失能（中档不使用） | 失能 | 由左拨杆指令决定 |
| `DART_SHOOT_MODE_TRIGGER` | **保持**（锁定上次的目标位置） | 位置累加；无指令时**失能** | 速度环；没有方向指令时**失能** | 由左拨杆指令决定 |

遥控器的摇杆映射由 robot 层决定（组件只认 `ctrl_cmd` 里的方向）：

| 档位 | 同步带 | yaw | 扳机位置电机 |
|------|--------|-----|--------------|
| 右拨杆中档 | 左摇杆竖直 | — | — |
| 右拨杆上档 | 锁定不动 | 左摇杆水平 | 右摇杆竖直 |

- **摇杆只给方向**：三个执行器的命令都是 -1/0/+1，摇杆推多少无关；
  同步带/yaw 的速率和扳机位置电机的速度都写在 `param` 里。

- **位置累加**：上层只给方向（`belt_dir` / `yaw_dir` = -1/0/+1），组件按
  `目标位置 += 方向 × pos_rate × dt` 累加，再把目标位置交给角度环。摇杆推多少无关，
  所以手感是"摇杆控制转向、位置环保证不丢步"。
- **保持位置**：同步带在 `belt_dir == 0` 时停止累加，但角度环继续给定同一个目标位置，
  因此同步带顶在原地不动；**在 AIM 与 TRIGGER 档都是这个行为**（中档拉到位后切上档会锁死在该位置），
  只有 `DISABLED` 档才会 `DJIMotorStop()` 让机构自由。
  `yaw_dir == 0` 时 yaw 直接 `DJIMotorStop()`（不给电流，轴可被手动推动）；
  yaw 只有上档才使用，中档 `DartShootYawHandler()` 不会被调用。
- **失能语义**：`DJIMotorStop()` 让 DJI 电机发送 0 电流（电调仍在上电状态），不是速度环给定 0；
  舵机失能是 `ServoStop()`，即停止 PWM 脉冲输出。
- **重同步**：进入 `AIM` 档、以及 yaw 从失能恢复时，都会把累加目标重新同步到当前角度并清空
  PID 积分（`PIDClear`），避免失能期间机构被手推动后使能瞬间跳变。
- **方向**：组件里没有方向参数，累加的正方向就是电机配置出来的正方向；实车方向相反时改
  `robot_config.h` 中对应电机配置的 `_reverse`（同步带两个电机要一起改）。

## API

```c
DartShootInstance *dart = DartShootInit(&dart_shoot_init_config);  // RobotInit()

DartShootSetMode(dart, DART_SHOOT_MODE_AIM);   // 档位: DISABLED / AIM / TRIGGER
DartShootSetBeltDir(dart, 1);                  // 同步带累加方向: -1 / 0 / +1
DartShootSetYawDir(dart, -1);                  // yaw 累加方向: -1 / 0 / +1
DartShootSetTriggerDir(dart, 1);               // 扳机位置电机转向: -1 / 0 / +1, 0 = 失能
DartShootSetServo(dart, DART_SERVO_CMD_ANGLE_MID);  // 舵机: DISABLED / ANGLE_MID / ANGLE_UP
DartShootSetServoAngle(dart, 45.0f);           // 调试用: 直接使能并转到指定角度

DartShootTask(dart);                           // RobotTask() 中 1kHz 调用
```

上层通过 `dart->ctrl_cmd`（`DartShoot_Ctrl_Cmd_s`）下发命令，通过 `dart->feed`（`DartShoot_Feed_s`）
读取状态：档位、4 个电机是否在线、各执行器使能状态、同步带/yaw 的目标与当前位置、
扳机位置电机速度、舵机角度。

## 参数标定要点（robot 层 robot_config.h）

- `belt_pos_rate` / `yaw_pos_rate`：位置累加速率（电机总角度/秒），**必须小于对应电机角度环的
  `MaxOut`（速度限幅）**，否则目标会一直跑在前面追不上。
- `servo_angle_mid` / `servo_angle_up`：左拨杆中档/上档对应的舵机角度，用
  `DartShootSetServoAngle()` 或临时改宏扫描确定；角度按 0~`DART_SERVO_ANGLE_RANGE`
  （实车 270° 舵机，见 `dart_shoot.c`）线性映射到 `servo_min_pulse_s` ~ `servo_max_pulse_s`
  （270° 舵机为 0.5ms~2.5ms，135° = 行程中点 1.5ms），占空比用 `pwm_instance->period` 计算。
- `trigger_speed`：扳机位置电机的速度参考（度/秒），摇杆偏出阈值时按这个速度转。
- 角度环 PID 的 `MaxOut` 是速度环参考限幅（度/秒），也是位置跟踪的能力上限；速度环 PID 的
  `MaxOut` 是电流指令限幅（C620 为 ±16384，C610 为 ±10000）。
