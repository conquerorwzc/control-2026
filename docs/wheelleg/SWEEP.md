# motor_sysid 台架扫频

实验按 SHU-Train/docs/SWEEP_IDENT.md §4、§6 执行。入口是 UserApp/robot/motor_sysid/robot.c，参数在同目录 robot_config.h。机器人层选工作点、处理遥控和生成参考轨迹。chassis_wheelleg_demo 调用队内 DMMotor 和 DJIMotor，读取反馈、解算髋到轮轴的腿长和腿角并下发指令。vofa_watch 使用已有 VOFAJustFloatSend。测量时间来自 DWT，共用机器人任务以 500 Hz 调度。发送器异步运行，记录的命令时间是发布给驱动的时刻。

## 如何做一组实验

1. 在 robot_config.h 选 SYSID_EXPERIMENT、SYSID_TEST_SIDES、SYSID_TRIAL_DIRECTION。方向分别取 +1 和 −1。髋实验另选 SYSID_HIP_PATTERN 和 kWorkpointId/kCenterRad。默认选择左腿，SYSID_REAL_HARDWARE=0 可先看参考轨迹。
2. 做髋试验时固定机身、保留整条闭链腿、轮子离地。按指南先完成力矩读数标定，拆气弹簧做多工作点辨识，装回后用独立工况验证。做轮试验时轮子离地自由转动。分别记录左右轮、母线电压、温度及轮胎/机械装配。
3. 对应实机参数填入 kCenterRad、kHipLimits 或 kWheelLimits、kRealAmplitude、反馈超时和控制周期超时。SWEEP_IDENT §4.4 的 10/0.5 是本实现髋位置 PD 的初始试验增益。中心角用逻辑关节坐标，允许角度范围、限力矩/限电流、起动门槛需用本台架确认值。左髋零偏采用已确认的 OK/OA 标定值。右髋零位及两轮方向仍需标定。完成配置后编译 SYSID_REAL_HARDWARE=1。
4. UART5 接 DT7，USART1 以 1 Mbaud、8N1 接 VOFA+，协议选 JustFloat、录制 CH0..CH24。在线左下复位，左中准备并用右拨杆选档，记录 READY 配置页，再左上执行。完成、取消或故障后重新左下准备。软件停机向驱动写零，DM 注册本身会使能。实机先确认支撑、独立断电与工作域。
5. 完整录制后按试验开始/结束的有效段导出 CSV 和 JSON。保存一次未参与拟合的独立姿态或方向作为留出数据。检查序号、实际反馈时戳、裁剪标志和各电机温度。

| SYSID_EXPERIMENT | 右拨杆上 / 中 / 下 | 本轮输入 |
|---|---|---|
| SYSID_HIP | 同步 / 两腿反相（须双腿） / 无 | 0.5→5 Hz 正弦扫频，默认 20 s，起始幅值 0.05 rad |
| SYSID_HIP_STEP | 无 / 阶跃 / 无 | 中心→指定偏移→中心 |
| SYSID_HIP_TRIANGLE | 0.2 / 0.5 / 1 rad/s | 同一工作点恒速三角往返 |
| SYSID_HIP_RAMP | 无 / 起动 / 无 | 先用 PD 保持，冻结稳定期实际指令均值，再叠加缓升力矩；未激励轴保持 PD |
| SYSID_WHEEL_STEP | 10 / 30 / 60 rad/s | 空载轮速阶跃 |
| SYSID_WHEEL_STEADY | 10 / 30 / 60 rad/s | 多档匀速保持 |
| SYSID_WHEEL_COAST | 10 / 30 / 60 rad/s | 先升速保持，再记录零电流指令下的减速 |
| SYSID_WHEEL_CURRENT_RAMP | 无 / 起转 / 无 | 正反向缓升电流 |
| SYSID_WHEEL_SWEEP | 无 / 扫频 / 无 | 以配置的轮速幅值扫频 |

同一条腿的前后关节组合由 SYSID_HIP_PATTERN 指定：SYSID_HIP_FRONT、SYSID_HIP_REAR、SYSID_HIP_BOTH_IN_PHASE 或 SYSID_HIP_BOTH_ANTI_PHASE。单轴激励时另一轴在中心角保持。左右腿之间的相位由 SYSID_HIP 的右拨杆选择。工作点按短/中/长腿高与 2–3 个摆角设置，每次运行一个点。轨迹幅值与增益按实际行程、温度和限幅逐步调整。

## 接线与观测

| 关节 | 总线 | DM CAN_ID / Master ID |
|---|---|---|
| 左前 OK / 左后 OA | CAN1 | 0x01/0x11、0x02/0x12 |
| 右前 / 右后 | CAN2 | 0x03/0x13、0x04/0x14 |

C620 左轮为 CAN1 ID 1，右轮为 CAN2 ID 2，减速比 15.76。髋实验仅注册指定侧的髋电机，轮实验仅注册指定轮。H723 底盘在首次 CANRegister 前核对 HAL 状态和 Message RAM 布局，按所选实例通过 HAL 初始化所需的标准过滤槽。机器人/DM/DJI/daemon 的任务归队内 os_task 调度。

调试器可查看 robot.chassis->legs[0/1].length_m 和 angle_rad，分别为虚拟腿长（m）和相对机身向下轴的腿角（rad）。单轮工况没有髋反馈，geometry_valid 区分是否可用。轮速度取 C620 接收帧的转子 RPM，换算为轮端 rad/s。C620 实际下发电流以 A 记录，反馈保留逻辑方向的原始计数，换算为反馈 A 需有独立依据。

## 数据交接

25 通道帧按工况复用，500 Hz 占用约 520 kbit/s。单腿记录两轴角度参考、实测角/速度、限幅前后力矩、反馈力矩及原始码、温度和反馈时间。轮试验记录轮速参考、电流参考、实测轮速、限幅前后电流、原始反馈、温度及反馈时间。四髋模式侧重四路同拍角/速度和限幅后力矩，温度轮换；详细时序和限幅前量优先用单腿工况采集。

READY 轮换 8 个配置页，保存工作点、增益、方向、试验幅值及轨迹时间。母线电压、弹簧装拆、挂载质量和力矩反馈校正系数由实验记录补充。髋 MIT 的原始字段是扭矩码。轮零电流段应按驱动模式解释。空载数据可用于起转阈值、匀速电流、阶跃/滑行响应；完整带载扭矩包络和对地摩擦需要其他证据。

[VOFA 通道、CSV 转换与回放条件](../../tools/hip_ident/README.md)
