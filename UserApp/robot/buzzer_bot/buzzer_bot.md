# buzzer_bot — 蜂鸣器音乐演示机器人

最小演示机器人：不挂载任何底盘/云台/发射机构，上电延迟 `BUZZER_BOT_AUTOPLAY_DELAY_MS`
（默认 1 s）后自动播放《鸣潮 黎那汐塔版本组曲》（`Modules/alarm/music/rinascita_suite.c`），
并按 `BUZZER_BOT_SERVICE_PERIOD_MS`（默认 20 ms）周期调用 `BuzzerMusicSuiteService()`
补队后续分段，播完即静音。

## 编译

`CMakeLists.txt` 中把 `ROBOT_TYPE` 切换为 `buzzer_bot`（注释掉其他机器人行）：

```cmake
# set(ROBOT_TYPE "sentry_omni_gimbal")
set(ROBOT_TYPE "buzzer_bot")
```

```bash
cmake -DbuildType=Release -G Ninja -B cmake-build-debug-stm32
cmake --build cmake-build-debug-stm32
```

## 文件

| 文件 | 说明 |
| --- | --- |
| `robot.c` | `RobotInit()` 记录上电时刻；`RobotTask()` 到点自动播放 + 周期补队 |
| `robot_config.h` | `ONE_BOARD`、自动播放延迟、补队周期 |
| `robot.cmake` | 只编译本目录源文件，不挂载机构组件 |

## 行为说明

- 播放起点 = `RobotInit()` 记录的上电时刻 + 1 s。蜂鸣器硬件由守护任务里的
  `BuzzerInit()` 初始化，就绪前提交会返回 `BUZZER_NOT_READY`，`robot.c` 会按
  任务周期重试，直到成功接收。
- 组曲共 23 段（约 16 分钟），超出请求池的分段由 `BuzzerMusicSuiteService()`
  在播放过程中补队；`RobotTask()` 以 1 kHz 运行，因此用 20 ms 节流调用。
- 播放期间报警/提示音可抢占，抢占结束自动续播；想立即停止可调
  `BuzzerMusicSuiteStop()` 或 `BuzzerStop()`。
- 想循环播放：在 `RobotTask()` 里检测 `!BuzzerMusicSuitePlaying()` 后重新调用
  `BuzzerMusicSuitePlay(&kRinascitaSuite, NULL)` 即可。
- 想换曲目：重新运行 `Modules/alarm/music/tools/midi_to_score.py` 生成新的音乐文件，
  或把 `robot.c` 里的 `&kRinascitaSuite` 换成其他 `Buzzer_MusicSuite_s`。

上板验收参见 `Modules/alarm/buzzer.md` 与 `Modules/alarm/music/music.md` 的验证记录；
本机器人未做真实硬件验证。
