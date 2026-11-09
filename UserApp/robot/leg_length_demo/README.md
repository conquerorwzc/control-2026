# SHU 遥控腿长精简移植

位置：队内仓库 `D:/Code-RM/github`，`wheelleg_RL` 分支，构建目标 `leg_length_demo`。
来源：`D:/Code-RM/control-2026` 的 `wheel_legged_SHU`，提交 `170306b7`。

## 功能

- DT7/UART5，拨轮 -660..660 同时控制左右腿长 0.05619..0.18192 m。
- 任一拨杆下位、遥控掉线或非法编码：停止。右拨杆中/上均为腿长模式，上位不再调用平衡。
- 上电或故障/掉线后，先在线拨到下位，再切回中/上位；不会自动恢复出力。
- 目标 500 Hz：读取四关节 → 原 SHU 同心 ACE 几何 → 队内 PIDCalculate + 腿速阻尼 → J^T F → 写原驱动力矩目标。
- 四个电机各自复用原 DMMotorTask 发送最近目标（osDelay(1)），不再是同周期同步发送。
- 从测得腿长按源配置 0.05 m/s 接入拨轮目标。复用原参数 Kp=800、Ki=0、阻尼5、轴向力±40 N、关节力矩±2 Nm。
- 不注册轮电机；无行驶、LQR、Roll、跳跃、自起、云台、发射、RL 推理。
- 按“只保留腿长 PID”去掉源代码的 8.25 N 重力前馈及 IMU 依赖；源气弹簧补偿也不启用。这是台架伸缩应用，不是站立平衡控制器。

## 参数与接线

源配置明确标为迁移占位、几何和传动 gates=0，因此 `robot_config.h` 保留 `LEG_LENGTH_CALIBRATED=0`。
此时只初始化遥控和状态话题，不注册电机，不运行腿长控制，也无法查看电机反馈。确认下面的旧 demo 参数适用于当前机器后才改成 1：

| 关节 | CAN | 发送 / 反馈 ID | 源反馈零偏 rad | 驱动方向 |
|---|---|---|---|---|
| 左前 | 1 | 0x0B / 0x0A | 0.815470527 | 正 |
| 左后 | 1 | 0x09 / 0x08 | -2.469308355 | 正 |
| 右前 | 2 | 0x07 / 0x06 | -3.051547223 | 输出和反馈均反向 |
| 右后 | 2 | 0x05 / 0x04 | -2.642480681 | 输出和反馈均反向 |

四电机按源 SHU 为 J8009P，MIT 映射沿用队内驱动 P±12.5/V±45/T±54。不可将2 Nm应用限幅误设为协议TMAX。
几何 AH=105 mm、HJ=125 mm、CE=62.5 mm，主动轴同心。仅是源 demo 数据，不是新车 CAD 验收结果。
左右方向先在低力矩架空状态核对。没有自动清错或自动保存电机零点。

## 复用与必要适配

复用 `PIDInit/PIDCalculate`、`RemoteControlInit/IsOnline`、`CANRegister/CANTransmit`、DWT、daemon、message_center。
从 SHU 提取 `LegCalculate` 的长度与长度雅可比部分；不搬其数百维状态结构。
DM 直接使用原 `DMMotorInit/SetRef/Enable/Stop/Task`；应用单独创建发送任务，不调用旧 `DMMotorTaskInit` 的任务名拼接路径。
反馈新鲜度由应用观察原 `feed_cnt` 的变化，不给驱动结构增加字段。
腿长控制与空跑任务用已启用的 `vTaskDelay` 补足到下一计划 tick，不依赖已撤销的 `vTaskDelayUntil` 配置。

### 底层恢复后的限制

- `Modules/Bsp/Hardware` 与开发前 `5d86e5a` 完全一致。未通过应用 CMake 重开底层功能或改变底层源文件编译选项。
- 原 `DMMotorInit` 会发送硬件使能帧，应用随后置零并 `DMMotorStop`。标定开关为0时完全跳过电机注册。
- `DMMotorStop` 的含义是持续发零力矩，不是发送 FD 硬件失能。原离线回调仍会重发使能并阻塞 daemon 100 ms；本次没有改变这一行为。
- 原 CAN1/2 各只有1个标准过滤器，而此demo每路注册两个关节；这个配置不匹配仍然存在。UART5保持原9B/无校验设置。未验证实际接收，不保证仅打开标定开关即可运行。
- 应用的遥控离线判断依赖旧daemon，不能保证发生其他设备离线阻塞时仍满足固定的遥控离线响应时间。
- 本轮完成的是应用接口兼容及软件测试，未消除原底层的已知问题，未连接或烧录硬件。

## 构建与查看

```powershell
cmake -S . -B build/leg_length_demo -G Ninja -DROBOT_TYPE=leg_length_demo -DCMAKE_BUILD_TYPE=Debug
cmake --build build/leg_length_demo
```

调试器看 `length_demo`：mode、requested_m、target_m、leg[].length_m/speed_m_s/force_n/torque_nm/pid。
`leg_length_overruns` 统计调度超期。状态通过 `leg_length_status` 话题发布，未额外接 VOFA。
mode：0停止、2伸缩、3故障。去掉等待硬件使能反馈状态，重新接管前仍需在线拨下再切回。

没有烧录或电机实测；软件编译/单元测试不代表参数已标定。

## 应用适配后的验证（2026-11-09）

- `python tests/leg_length_demo/run.py --cc gcc`：数值测试和应用回归测试。硬件输入输出/RTOS使用测试替身，应用状态机与队内 PID 使用真实代码。
- 数值：SHU参考姿态、雅可比有限差分、PID+阻尼、力矩限幅和拨杆映射通过。
- 应用：未标定不注册电机；上电不自动接管、接管斜率、双腿输出、遥控掉线/重连、反馈超时、故障锁存、NaN、tick回卷通过。
- `leg_length_demo` / H723 Debug（默认标定开关0）：FLASH 56,788 B、RAM_D1 65,696 B。
- 仅编译验证开关1：FLASH 68,612 B、RAM_D1 66,952 B。这个产物只用于检查完整代码能链接，没有烧录。
- `wheel_leg_rl`、`motor_sysid` / H723 Debug 均通过：各 FLASH 56,012 B、RAM_D1 65,336 B。
- 完整工程仍有旧日志头等既有警告。真实任务周期、通讯、物理动作和安全性尚未实测。
