# SHU 遥控腿长精简移植

位置：队内仓库 `D:/Code-RM/github`，`wheelleg_RL` 分支，构建目标 `leg_length_demo`。
来源：`D:/Code-RM/control-2026` 的 `wheel_legged_SHU`，提交 `170306b7`。

## 功能

- DT7/UART5，拨轮 -660..660 同时控制左右腿长 0.05619..0.18192 m。
- 任一拨杆下位、遥控掉线或非法编码：停止。右拨杆中/上均为腿长模式，上位不再调用平衡。
- 上电或故障/掉线后，先在线拨到下位，再切回中/上位；不会自动恢复出力。
- 500 Hz：读取四关节 → 原 SHU 同心 ACE 几何 → 队内 PIDCalculate + 腿速阻尼 → J^T F → 原 DM MIT 编码发送。
- 从测得腿长按源配置 0.05 m/s 接入拨轮目标。复用原参数 Kp=800、Ki=0、阻尼5、轴向力±40 N、关节力矩±2 Nm。
- 不注册轮电机；无行驶、LQR、Roll、跳跃、自起、云台、发射、RL 推理。
- 按“只保留腿长 PID”去掉源代码的 8.25 N 重力前馈及 IMU 依赖；源气弹簧补偿也不启用。这是台架伸缩应用，不是站立平衡控制器。

## 参数与接线

源配置明确标为迁移占位、几何和传动 gates=0，因此 `robot_config.h` 保留 `LEG_LENGTH_CALIBRATED=0`。
此时可看遥控和反馈，但不会发送使能帧。确认下面的旧 demo 参数适用于当前机器后才改成 1：

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
DM 原注册接口和异步任务继续服务旧目标；新增显式同步注册接口，在本目标里默认失能，并由同一个500 Hz任务发送。
新接口在停止状态周期发送 FD；反馈/发送错误必须停止，离线回调不自动使能。
仅本目标把 CAN1/2 标准过滤器数量设为2，并将 UART5 配成 DT7 的8位数据+偶校验。

## 构建与查看

```powershell
cmake -S . -B build/leg_length_demo -G Ninja -DROBOT_TYPE=leg_length_demo -DCMAKE_BUILD_TYPE=Debug
cmake --build build/leg_length_demo
```

调试器看 `length_demo`：mode、requested_m、target_m、leg[].length_m/speed_m_s/force_n/torque_nm/pid。
`leg_length_overruns` 统计调度超期。状态通过 `leg_length_status` 话题发布，未额外接 VOFA。
mode：0停止、1等待使能反馈、2伸缩、3故障。停止后才重新使能。

没有烧录或电机实测；软件编译/单元测试不代表参数已标定。

## 本轮验证

- 主机数值测试通过：SHU 参考姿态腿长、雅可比有限差分、真实队内 PID+阻尼、力矩限幅、非有限输入和所有拨杆组合。
- 运行：`python tests/leg_length_demo/run.py --cc gcc`。
- `leg_length_demo` / H723 Debug 编译通过：FLASH 68,904 B，RAM_D1 66,920 B。
- 原 `wheel_leg_rl`、`motor_sysid` / H723，以及 `sentry_omni_gimbal` / F4 Debug 回归编译通过。
- 完整工程仍有旧日志头等既有警告；新增应用没有直接定位到自身的警告。
- 未测试实际 CAN 时序、遥控器接线或电机动作，未烧录；现有源 demo 目录没有修改。
