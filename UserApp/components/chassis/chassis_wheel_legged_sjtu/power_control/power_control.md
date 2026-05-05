# chassis_wheel_legged_sjtu 功率控制逻辑说明

本文档整理 `chassis_wheel_legged_sjtu` 中与功率控制相关的代码链路，重点对应：

- `chassis.c`
- `chassis.h`
- `power_control/power_control.c`
- `power_control/power_control.h`
- `Modules/super_cap/super_cap.c`

## 当前调用状态

当前代码里有两套功率相关逻辑：

1. `power_control/` 下的 `PowerControl()`：用于在轮腿 LQR 输出阶段按功率预算重分配轮电机运动扭矩。
2. `super_cap` 下的 `SuperCapModeControl()`：用于根据裁判系统功率限制、超级电容状态和超电指令计算 `chassis_ctrl_cmd.max_power`。

实际运行时，`PowerControl()` 暂未参与闭环控制：

```c
// PowerControl(chassis);
LocomotionController();
LegController();
```

也就是说，`PowerControlInit(chassis_instance)` 会在 `ChassisInit()` 中初始化功率控制参数，但 `PowerControl()` 本体在 `ChassisCtrlUpdate()` 中被注释掉了。当前真正每帧生效的功率逻辑是：

```c
chassis_ctrl_cmd->max_power =
    SuperCapModeControl(chassis->super_cap, referee_data->GameRobotState.chassis_power_limit);

SuperCapSendMessage(chassis->super_cap,
                    (int16_t)referee_data->GameRobotState.chassis_power_limit * (13.0f / 14.0f),
                    referee_data->PowerHeatData.buffer_energy,
                    referee_data->GameRobotState.power_management_chassis_output);

LimitChassisOutput();
```

## 每帧主链路

`ChassisTask()` 的底盘主循环大致如下：

1. 根据 `chassis_mode` 启停电机。
2. 根据模式调用 `ChassisCtrlUpdate()`、`ChassisJump()`、`ChassisRecovery()` 或 `ChassisProstrateMode()`。
3. 从裁判系统读取 `GameRobotState.chassis_power_limit`。
4. 通过 `SuperCapModeControl()` 结合超电状态计算 `chassis_ctrl_cmd.max_power`。
5. 通过 `SuperCapSendMessage()` 把裁判系统功率限制、缓冲能量、底盘电源输出状态发给超电。
6. 调用 `LimitChassisOutput()` 做最终电机输出限幅和下发。

其中 `max_power` 会作为 `PowerControl()` 的预算输入，但因为 `PowerControl()` 当前没有被调用，所以它目前只影响超电通信与后续可扩展逻辑，并不直接限制 `LocomotionController()` 计算出的轮电机扭矩。

## 超级电容功率上限逻辑

`SuperCapModeControl(super_cap, power_limit)` 的输入是裁判系统给出的 `power_limit`，输出是底盘内部使用的 `max_power`。

超电有四个状态：

- `SAFETY_MODE`：安全模式。
- `PASSIVE_MODE`：被动模式，电容电压足够时自动放宽功率。
- `ACTIVE_MODE`：主动模式，由控制指令 `BOOST` 触发。
- `CHARGING_MODE`：充电模式，电压偏低时压低底盘功率。

主要转移规则如下：

| 当前状态 | 条件 | 下一状态 | `max_power` |
| --- | --- | --- | --- |
| 任意 | `error_detect != 0` | `SAFETY_MODE` | `power_limit` |
| `SAFETY_MODE` | `ctrl_cmd == BOOST` | `ACTIVE_MODE` | `180` |
| `SAFETY_MODE` | `cap_v > 18V` | `PASSIVE_MODE` | `power_limit + 40` |
| `SAFETY_MODE` | 其他 | 保持 | `power_limit` |
| `CHARGING_MODE` | `cap_v < 10V` | 保持 | `power_limit * 0.8` |
| `CHARGING_MODE` | `cap_v > 18V` | `PASSIVE_MODE` | `power_limit + 40` |
| `CHARGING_MODE` | 其他 | 保持 | `power_limit * 0.9` |
| `PASSIVE_MODE` | `ctrl_cmd == BOOST` | `ACTIVE_MODE` | `180` |
| `PASSIVE_MODE` | `cap_v < 15V` | `CHARGING_MODE` | `power_limit - power_limit^2 * 0.0025` |
| `PASSIVE_MODE` | `15V <= cap_v <= 18V` | `SAFETY_MODE` | `power_limit` |
| `PASSIVE_MODE` | `cap_v > 18V` | 保持 | `power_limit + 40` |
| `ACTIVE_MODE` | `cap_v < 15V` | `CHARGING_MODE` | `power_limit * 0.9` |
| `ACTIVE_MODE` | `ctrl_cmd == NORMAL` | `PASSIVE_MODE` | `cap_v > 18V ? power_limit + 40 : power_limit` |
| `ACTIVE_MODE` | 其他 | 保持 | `180` |

超电主动模式由上层控制切换。`infantry_wheel_legged_sjtu/ctrl/ctrl.c` 中键盘 `C` 会在 `NORMAL` 和 `BOOST` 之间切换 `super_cap_ctrl_cmd`。双板模式下该指令通过 `Chassis_Fetch_Data_s.super_cap_ctrl_cmd` 从云台板同步到底盘板。

## `PowerControl()` 设计意图

`PowerControl()` 试图把轮腿 LQR 的轮电机输出拆成两部分：

- `T_motion`：由前进速度、航向误差等运动状态产生的轮端运动扭矩。
- `T_balance`：总轮端扭矩中除运动分量以外的部分，主要理解为平衡/姿态维持消耗。

代码先用当前状态和 LQR 矩阵重算运动扭矩：

```c
pc->T_motion[0] -= chassis->LQR_K[2][i] * state_err[i];
pc->T_motion[1] -= chassis->LQR_K[3][i] * state_err[i];
```

再从 `LocomotionController()` 已经写入的总轮端扭矩中扣除运动扭矩：

```c
pc->T_total[i] = chassis->leg[i]->real_model.T;
pc->T_balance[i] = pc->T_total[i] - pc->T_motion[i];
```

然后用扭矩-电流模型和 6 参数功率模型估算运动功率、平衡功率：

```c
pc->I_balance[i] = t2i(pc->T_balance[i]);
pc->I_motion[i] = t2i(pc->T_motion[i]);
pc->w[i] = chassis->leg[i]->wheel_motor->measure.speed_aps * DEGREE_2_RAD;

pc->P_motion[i] = MotorEstimatePower(pc->k, pc->I_motion[i], pc->w[i]);
pc->P_balance[i] = MotorEstimatePower(pc->k, pc->I_balance[i], pc->w[i]);
```

功率预算分配方式是：

```c
pc->P_motion_total_ref = chassis->chassis_ctrl_cmd.max_power - pc->P_balance_total;
```

也就是优先预留平衡功率，剩余功率给运动控制。如果运动功率估计值超过剩余预算，则按当前两个轮子的运动功率占比分配可用功率：

```c
pc->P_motion_ref[i] = pc->P_motion[i] / pc->P_motion_total * pc->P_motion_total_ref;
```

随后通过 `MotorEstimateCurrent()` 从目标功率反解可允许的运动电流，再转回运动扭矩：

```c
pc->I_motion_ref[i] = MotorEstimateCurrent(pc->k, pc->P_motion_ref[i], pc->w[i], pc->I_motion[i]);
pc->T_motion_ref[i] = i2t(pc->I_motion_ref[i]);
```

最后利用 LQR 轮端输出矩阵的伪逆，把限制后的 `T_motion_ref` 反解成新的状态误差，再改写控制指令：

```c
chassis->chassis_ctrl_cmd.vx = sv->x_b_d - new_state_err[1];
chassis->chassis_ctrl_cmd.target_yaw = sv->phi - new_state_err[2];
chassis->chassis_ctrl_cmd.wz = sv->phi_d - new_state_err[3];
```

这相当于不是直接限幅电机输出，而是把超功率后的运动需求折算回 `vx / target_yaw / wz`，希望下一轮 LQR 计算出的轮端扭矩自然变小。

## 6 参数功率模型

模型系数定义在 `power_control.h`：

```c
#define WHEEL_K0 0.7441993412640775f
#define WHEEL_K1 0.0090164284468539646f
#define WHEEL_K2 0.0001988857226262331f
#define WHEEL_K3 0.024694430204543864f
#define WHEEL_K4 0.20160143850678086f
#define WHEEL_K5 3.715221772539512e-05f
```

注释中的模型是：

```text
P = k0 + k1 * |I| + k2 * |w| + k3 * |I| * |w| + k4 * I^2 + k5 * w^2
```

其中：

- `I`：由轮端扭矩经过 `t2i()` 换算得到的电流。
- `w`：`speed_aps * DEGREE_2_RAD` 得到的轮电机转子侧角速度。
- 输出 `P`：估计电功率，并被钳位到非负。

反解电流时，把功率模型视作关于 `|I|` 的一元二次方程：

```text
k4 * x^2 + (k1 + k3 * |w|) * x + (k0 + k2 * |w| + k5 * w^2 - P_target) = 0
```

代码根据当前电流方向选择更接近当前电流的根，并转换回扭矩。

## 当前实现需要注意的问题

1. `PowerControl()` 当前没有接入主循环。

   如果要启用，需要考虑调用顺序。现在注释位置在 `LocomotionController()` 之前，但 `PowerControl()` 内部读取 `leg[i]->real_model.T` 作为总轮端扭矩；这个值通常应先由 `LocomotionController()` 更新。因此直接取消注释可能读到上一帧的 `T_total`。

2. `PowerControl()` 的“改写指令”方式有一拍延迟或顺序依赖。

   它改的是 `chassis_ctrl_cmd.vx / target_yaw / wz`，不是直接改 `leg[i]->real_model.T`。如果放在 `LocomotionController()` 之后，需要再执行一次 LQR 或改成直接合成受限后的轮端扭矩，否则当前帧不一定立刻生效。

3. `MotorEstimatePower()` 注释写的是 `|I|` 和 `|w|`，实际代码没有取绝对值。

   当前实现为：

   ```c
   float P = k[0] + k[1] * I + k[2] * w + k[3] * I * w + k[4] * I * I + k[5] * w * w;
   ```

   如果 `I` 或 `w` 为负，一次项和交叉项会改变符号，和注释模型不一致。其他底盘模块里也有类似 6 参数模型，但这里的注释明确写了绝对值，建议统一。

4. `MotorEstimateCurrent()` 里 `B = k[1] + k[3] * w`，注释期望的是 `|w|`。

   若轮速为负，`B` 可能被减小甚至变号，会影响二次方程根。

5. `P_motion_total_ref` 可能小于 0。

   当 `P_balance_total > max_power` 时：

   ```c
   pc->P_motion_total_ref = max_power - pc->P_balance_total;
   ```

   这会得到负的运动功率预算。后续 `MotorEstimateCurrent()` 又对 `P_target` 做了 `fabsf()`，可能把负预算重新变成正预算，不符合“运动功率应被压到 0”的直觉。

6. `MotorEstimateCurrent()` 的限幅单位疑似不一致。

   `I_motion` 是通过 `t2i()` 得到的电流量级，峰值大约几十安；但反解结果用 `±16000` 限幅，更像 DJI 电流指令原始值，不像安培单位。这里需要确认单位，否则限幅基本不起保护作用。

7. `power_control.h` 中 `Power_Ctrl_t` 的注释和当前实现不一致。

   头文件注释写的是“低通滤波、PI 闭环、速度幅值限制、斜率限制”，但 `power_control.c` 当前实现是“运动/平衡功率拆分 + 伪逆反解控制指令”，没有 PI、低通和速度斜率限制。

## 建议接入方式

如果后续要真正启用 `PowerControl()`，建议先明确采用哪种控制输出方式：

1. 直接限轮端扭矩。

   流程可以是 `LocomotionController()` 先算 `leg[i]->real_model.T`，`PowerControl()` 再把受限后的 `T_motion_ref + T_balance` 写回 `leg[i]->real_model.T`。这样当前帧直接生效，逻辑也更容易验证。

2. 限制上层运动指令。

   如果继续沿用现在“反解状态误差并改写 `vx / target_yaw / wz`”的方式，需要安排为：

   ```text
   LocomotionController()
   PowerControl()
   LocomotionController()  // 用被改写后的指令重新计算本帧输出
   LegController()
   ```

   或者把 `PowerControl()` 的输入改成先验的目标指令和当前状态，避免依赖上一轮输出。

无论采用哪种方式，建议先修正绝对值、负预算、单位限幅和头文件注释，再上车测试。
