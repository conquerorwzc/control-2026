# 轮腿 RL 与辨识初版

本目录说明 `wheelleg_RL` 分支的新增内容。强化学习仓库为 `D:/WHEEL_LEG_RL/SRM_Wheelleg_Rltrain`。
部署、采集、扫频、电机/气弹簧辨识与分析工具都在本队内仓库开发。

## 目前已实现

- `wheel_leg_rl` 与 `motor_sysid` 两个 H723 构建目标，不改变原默认机器人。
- 两目标选择专用 `wheelleg_tasks.c`，替代旧的独立 motor/robot/DM 任务。
- 绝对节拍 500 Hz + 100 Hz 策略调度槽；DWT 记录周期与超时；`wheelleg_timing` 经 message_center 发布。
- 辨识正弦、线性扫频、平滑平台激励的数学内核和主机数值测试。
- 从训练仓库生成的接口头：`WL_DEPLOYMENT_CONFIG_READY=0`。

**这不是能上车的固件。两个目标均不注册电机、不使能、不发送控制帧；没有实际推理或辨识采集。**
尚未接入 IMU/遥控和实车安全门控。当前固件目的仅为 FW0 构建与离线数学验证。
现有 `DMMotorInit` 会自动使能，后续 FW1 必须先处理，再允许新目标使用。

## 构建

```powershell
cmake -S . -B build/wheel_leg_rl -G Ninja -DROBOT_TYPE=wheel_leg_rl -DCMAKE_BUILD_TYPE=Debug
cmake --build build/wheel_leg_rl
cmake -S . -B build/motor_sysid -G Ninja -DROBOT_TYPE=motor_sysid -DCMAKE_BUILD_TYPE=Debug
cmake --build build/motor_sysid
```

确保 `cmake`、`ninja`、`arm-none-eabi-gcc` 在 PATH 中。不烧录，不自动使能。

主机测试（MinGW GCC）：

```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror -I UserApp/robot/motor_sysid tests/wheelleg/test_sysid_signal.c UserApp/robot/motor_sysid/sysid_signal.c -lm -o build/test_sysid_signal.exe
./build/test_sysid_signal.exe
```

## 下一批依赖

1. 安全驱动模式：默认失能、每实例协议范围、非阻塞离线处理、统一 500 Hz 下发。
2. 确认电机 ID、方向、零点和允许工作域后，接入实时状态与应用安全仲裁。
3. 在本仓库实现遥测及分析脚本，再进行完整扫频、力矩、弹簧和延迟辨识。
4. CAD 与辨识数据回写训练配置后开始正式 PPO，随后 ONNX/CubeAI 和实车验证。

左右每条腿各一路 CAN：2 个 8009P + 1 个 M3508/C620。减速比目前按用户确认的 15.76。
450 N / 75 mm 是弹簧标称规格，不是已辨识的力程模型。
新代码按仓库实际 Microsoft clang-format；既有驱动未整体格式化。

## 实际验证记录（2026-11-08）

- `wheel_leg_rl` / H723 Debug：编译成功，链接器报告 FLASH 56,160 B，RAM_D1 65,336 B。
- `motor_sysid` / H723 Debug：编译成功，占用相同。数学内核尚无运行时调用，链接器会删除未使用代码；不能把这个大小当作最终辨识固件大小。
- 原默认 `sentry_omni_gimbal` / F4 Debug 回归编译成功，FLASH 85,396 B，RAM 79,336 B。
- 主机 C 测试通过：正弦、线性扫频相位、包络、边界、NaN 和 Nyquist 检查；使用 `-Wall -Wextra -Werror`。另以 `-Ofast -fno-fast-math` 验证了 NaN 检查。
- 新 C 源没有直接定位到新增代码的编译警告；完整工程仍有既有警告（日志头多行注释、旧模块未使用变量、packed 对齐等），保留在 `build/<target>/build.log`。本轮未顺带清理。
- 新目标在 FreeRTOSConfig.h 的 USER CODE 区按 `WHEELLEG_BOOTSTRAP` 开启 `vTaskDelayUntil`，其他目标保持原设置。
- 修复了原根 CMake 的构建注释换行，否则 CMake 4.1 / Ninja 生成的 rules.ninja 无法解析。
- 编译通过不代表硬件节拍、CAN 负载或安全联锁已实测。未连接硬件、未下载、未使能电机。

当前用户授权直接使用 `wheelleg_RL`；优先于历史指南中从 master 新开 feature 的示例。
原有 `.idea/`、未跟踪 `doc/` 与 `tuning/` 不属于此次提交。
