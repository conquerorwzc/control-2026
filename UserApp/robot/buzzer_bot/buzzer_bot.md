# buzzer_bot — 蜂鸣器音效演示 + 组曲播放机器人

最小演示机器人：不挂载任何底盘/云台/发射机构。上电后先在蜂鸣器上**展示新模块的报警音效**，
再自动播放组曲，播放期间按周期补队分段，播完即静音。

## 上电时序

```
上电 ──1 s──> ① 5 种内置提示音 ──> ② 5 个报警层级遮蔽演示 ──> ③ 组曲播放（约 16 分钟）
          (等 BuzzerInit)     约 5.2 s              约 7.3 s
```

### ① 内置提示音（`BuzzerNotify()`，一次性音效）

按严重程度递增，每种播完留间隔再进下一个：

| 序 | 模式 | 音效 | 时长 |
| --- | --- | --- | --- |
| 1 | `BUZZER_PATTERN_READY` | 就绪：784 Hz 短音 | 240 ms |
| 2 | `BUZZER_PATTERN_SUCCESS` | 成功：523→659→784 上行 | 560 ms |
| 3 | `BUZZER_PATTERN_WARNING` | 警告：880 Hz 两声 | 960 ms |
| 4 | `BUZZER_PATTERN_ERROR` | 错误：1 kHz 长音 + 523 Hz | 1350 ms |
| 5 | `BUZZER_PATTERN_COMM_LOST` | 通信丢失：784 Hz 三连短音 | 920 ms |

### ② 报警层级遮蔽演示（`BuzzerSetIndicator()` / `BuzzerClearIndicator()`）

一次把 5 个层级全部置位，每级配不同音效：

| 层级 | 音效 |
| --- | --- |
| `ALARM_LEVEL_HIGH` | `BUZZER_PATTERN_ERROR` |
| `ALARM_LEVEL_ABOVE_MEDIUM` | `BUZZER_PATTERN_WARNING` |
| `ALARM_LEVEL_MEDIUM` | `BUZZER_PATTERN_COMM_LOST` |
| `ALARM_LEVEL_BELOW_MEDIUM` | `BUZZER_PATTERN_SUCCESS` |
| `ALARM_LEVEL_LOW` | `BUZZER_PATTERN_READY` |

模块只播**最高等级**，因此一开始只听到 HIGH 的错误音；每隔 `BUZZER_BOT_LEVEL_STEP_MS`
（默认 1450 ms）自高到低清掉一级，下一级的音效就露出来——直观演示"更高等级持续故障
遮蔽低等级"。注意：**层级只决定优先级，不改变声音**，声音由 pattern 决定。

### ③ 组曲播放

5 级全部清干净后调用 `BuzzerMusicSuitePlay(&BUZZER_BOT_SUITE, NULL)` 开始播放，并由
`BuzzerMusicSuiteService()` 按 `BUZZER_BOT_SERVICE_PERIOD_MS`（默认 20 ms）补队后续分段。

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
| `robot.c` | 4 阶段状态机：延时 → 提示音 → 层级遮蔽 → 组曲 + 周期补队 |
| `robot_config.h` | `ONE_BOARD`、起始延时、层级步长、补队周期、来源 ID 基址、播放曲目 |
| `robot.cmake` | 只编译本目录源文件，不挂载机构组件 |

## 可调项（`robot_config.h`）

| 宏 | 默认 | 作用 |
| --- | --- | --- |
| `BUZZER_BOT_START_DELAY_MS` | 1000 | 上电到开始演示的延时，留出 `BuzzerInit()` 时间 |
| `BUZZER_BOT_LEVEL_STEP_MS` | 1450 | 层级演示每级时长，需大于该级音效时长（错误音 1350 ms） |
| `BUZZER_BOT_SERVICE_PERIOD_MS` | 20 | 组曲补队周期 |
| `BUZZER_BOT_LEVEL_SOURCE_BASE` | 0x100 | 层级演示占用的 5 个持续来源 ID 基址 |
| `BUZZER_BOT_SUITE` | `kRinascitaSuite` | 播放的组曲，可改 `kWuciYouciSuite`（《无刺有刺》） |

## 行为说明

- 蜂鸣器硬件由守护任务里的 `BuzzerInit()` 初始化；就绪前提交会返回 `BUZZER_NOT_READY`，
  `robot.c` 会按任务周期重试到成功。
- 组曲 25 段（约 16 分钟），超出请求池的分段由 `BuzzerMusicSuiteService()` 在播放过程中补队；
  `RobotTask()` 以 1 kHz 运行，故用 20 ms 节流调用。
- 进入组曲阶段前会兜底清除全部演示用持续来源，避免持续报警（优先级高于音乐）把曲子遮住。
- 播放期间报警/提示音可抢占，抢占结束自动续播；停止用 `BuzzerMusicSuiteStop()` 或 `BuzzerStop()`。
- 想循环播放：检测 `!BuzzerMusicSuitePlaying()` 后重新调用 `BuzzerMusicSuitePlay()`。
- 未播放时 `BuzzerMusicSuiteService()` 返回 `BUZZER_NOT_FOUND`，是安全的空操作。

上板验收参见 `Modules/alarm/buzzer.md` 与 `Modules/alarm/music/music.md` 的验证记录；
本机器人未做真实硬件验证。
