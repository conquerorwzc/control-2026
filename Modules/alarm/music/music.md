# 蜂鸣器音乐文件

本目录存放由 MIDI 转换而来的蜂鸣器常量曲谱和配套的组曲播放接口。音乐数据完全遵循
`Modules/alarm/buzzer.h` 的曲谱约束，播放由既有 `BuzzerTask()` 推进，本目录不改动
蜂鸣器模块和任何机器人逻辑。

## 文件

| 文件 | 说明 |
| --- | --- |
| `rinascita_suite.c/.h` | 《鸣潮 黎那汐塔版本组曲》常量曲谱，`tools/midi_to_score.py` 生成，勿手工改动 |
| `buzzer_music.c/.h` | 通用组曲播放接口：把多段曲谱按 FIFO 顺序提交给蜂鸣器模块，长组曲流式补队 |
| `tools/midi_to_score.py` | MIDI → 蜂鸣器曲谱转换脚本（仅 Python3 标准库） |
| `tools/score_to_midi.py` | 反向工具：把生成的曲谱 C 文件还原为 MIDI，用于试听/校对音乐性 |
| `music.md` | 本文档 |

## 快速使用

```c
#include "rinascita_suite.h"

BuzzerRequestId_t music_id;
if (BuzzerMusicSuitePlay(&kRinascitaSuite, &music_id) == BUZZER_OK) {
    /* 开始播放；长组曲的后续分段靠 BuzzerMusicSuiteService() 补队 */
}

/* 周期调用（守护任务/RobotTask 循环里，10~100 ms 一次）： */
BuzzerMusicSuiteService();

/* 随时可以： */
BuzzerMusicSuiteStop();          /* 停止整套组曲：取消未播完分段、停止补队 */
BuzzerPauseMusic();              /* 暂停（保持进度） */
BuzzerResumeMusic();             /* 恢复 */
BuzzerSetMusicSpeed(1250);       /* 1.25 倍速，只改时值不改音高 */

/* 播完判断（结束后可再调 Play 循环整首）： */
if (!BuzzerMusicSuitePlaying()) {
    /* 整套播完 */
}
```

只播其中一段（例如试听第 3 段）：

```c
BuzzerPlayScore(&kRinascitaSuiteSegments[2], 1, NULL);
```

## 播放接口

```c
BuzzerResult_e BuzzerMusicSuitePlay(const Buzzer_MusicSuite_s *suite, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerMusicSuiteService(void);
uint8_t BuzzerMusicSuitePlaying(void);
BuzzerResult_e BuzzerMusicSuiteStop(void);
```

- **长组曲必须周期调用 `BuzzerMusicSuiteService()`**：蜂鸣器请求池只有 1 个当前请求加
  `BUZZER_WAITING_CNT`（16）个等待请求，`Play()` 一次只提交装得下的开头若干段，其余分段
  在播放过程中由 `Service()` 补队。段数因此不受请求池限制（本组曲 23 段）；不调用
  `Service()` 时播完已提交的段就会停。
- 分段按 `BUZZER_PRIORITY_MUSIC` 一次性请求依次提交，FIFO 连续播放，段边界不产生停顿；
  报警、提示音、debug 编码仍可抢占，抢占结束后继续播放。
- `Service()` 只在请求池有空位时提交，池满时静默等待下一次调用；无事可做返回 `BUZZER_OK`，
  没有进行中的组曲返回 `BUZZER_NOT_FOUND`。可安全地高频调用。
- `Play()` 首段提交失败（未初始化/应急锁存/池满）即返回错误且不开始播放；重复调用会先
  取消上一次尚未播完的组曲（最新请求优先）。
- `BuzzerMusicSuitePlaying()` 判定偏保守：末段已提交且请求池清空才返回 0；只看一次性
  请求/音乐，不含持续报警。
- `BuzzerMusicSuiteStop()` 只取消本接口跟踪的分段（按请求 ID 撤销仍在队列/播放中的），
  不影响其他蜂鸣器请求和持续报警；播放进度不保留。
- 全部接口仅允许任务上下文调用（与 `BuzzerPlayScore()` 相同），不能在中断里提交。

## 音乐文件格式

每段曲谱就是一个标准 `Buzzer_Score_s`（`note_count <= BUZZER_MAX_SCORE_NOTES = 128`），
`Buzzer_MusicSuite_s` 只是把若干段按顺序串起来：

```c
static const Buzzer_Note_s kXxxSegment00Notes[] = {
    {220000, 691, 84, &kBuzzerVoiceNormal},  /* 频率毫赫兹, 时值 tick, 力度, 音色 */
    {0, 120, 0, NULL},                       /* 休止符 */
    ...
};

const Buzzer_Score_s kXxxSegments[] = {
    {.notes = kXxxSegment00Notes, .note_count = 127, .bpm = 240},
    ...
};
```

- 时值 tick 以 `BUZZER_QUARTER_TICKS = 96` 为四分音符基准；生成文件统一使用 BPM 240 时基
  （1 tick = 2.604 ms），原曲的变速已经折算进每个音符的时值，因此播放速度与原曲一致。
- 频率为预计算毫赫兹常量，换算算法与 `BuzzerMidiToFrequency()` 完全一致（`kMidiOctave[]` 定点表）。
- 休止符频率为 0、力度为 0、音色指针为 `NULL`（走普通预设）。
- 音符数组必须是 `static const` 并在播放结束前保持有效；提交时只复制曲谱描述。

## 从 MIDI 重新生成

```bash
python3 Modules/alarm/music/tools/midi_to_score.py "<曲目.mid>" \
    --out-dir Modules/alarm/music --name rinascita_suite --title "鸣潮 黎那汐塔版本组曲"
```

常用参数（括号内为当前默认值，取向是**保真 + 和弦分解**）：`--voice`（`kBuzzerVoiceNormal`）、
`--time-base-bpm`（240，时基 30~300）、`--melody-track` / `--backup-track`（1 / 2）、
`--legato-ms`（120，小于此空隙连音化）、`--min-note-ms`（30，小于此长度的装饰音并入前音）、
`--fill-gap-ms`（600，主旋律停顿超过该值用伴奏轨填补）、`--chord-roll`（3，旋律音前自低向高
琶音的最多音数，0 关闭）、`--roll-note-ms`（40，琶音单音时值）、`--min-pitch`（`auto` 取主旋律最低音）。

转换策略（单声部化）：

1. 主旋律轨按起音扫描，和弦取顶音；低于当前旋律音的新起音视为内声部忽略，长旋律音不被和弦打断。
2. 主旋律停顿超过 `--fill-gap-ms` 时，用伴奏轨最高声部填补，避免长时间空白。
3. 简化：小空隙连音化、超短装饰音并入前音、同音相邻合并；低于音高下限的音符逐八度上移
   （蜂鸣器低音区几乎听不见）；力度（含和弦内声部）线性归一化到 64~127。
4. 和弦分解：和弦起音在旋律音之前插入最多 `--chord-roll` 个自低向高的短琶音，占用起音前的
   空隙或前一个音的尾巴，旋律音起音保持不变；空间不足自动减少琶音音数，快速段落不加，
   不改变原曲节奏。
5. 按真实毫秒折算 tick，起音按绝对时间对齐（取整残差写进休止符/延长音，误差不累积），
   空隙 ≥10 ms 才写休止符；切段优先落在休止符或长音上，保持乐句完整。
6. 自检保证满足 `BuzzerPlayScore()` 的全部校验：每段 ≤128 音符、音符 ≥10 ms、时值 ≤65535 tick、
   频率 20~8000 Hz、力度 ≤127、BPM 30~300。

## 本次曲目：鸣潮 黎那汐塔版本组曲

| 项目 | 数值 |
| --- | --- |
| 源文件 | `鸣潮 黎那汐塔版本组曲.mid`（3 轨钢琴，26 处变速） |
| 提取结果 | 23 段 / 2858 条目（2733 音 + 125 休止）＝ 旋律保留 1819 音 + 琶音/填补 914 音 |
| 播放时长 | 949.7 s（原曲 952.2 s），变速折算后与原曲一致 |
| 音域 | 155.6 ~ 2489 Hz |
| Flash 占用 | 约 33.7 KiB（12 字节/音符，含分段描述） |

## 试听与音乐性校对

`tools/score_to_midi.py` 把生成的曲谱 C 文件反向打包成标准 MIDI（format 0、单声部），
还原的就是 MCU 将要播放的内容——音高、时值、力度、休止与固件完全一致，段边界写成
MIDI marker，可直接在播放器/打谱软件里和原曲对比：

```bash
python3 Modules/alarm/music/tools/score_to_midi.py Modules/alarm/music/rinascita_suite.c \
    --out 鸣潮_黎那汐塔版本组曲_MCU蜂鸣器版.mid --title "鸣潮 黎那汐塔版本组曲 (MCU 蜂鸣器版)"
```

本次校对结果（从原曲主旋律出发逐音比对）：

| 指标 | 结果 |
| --- | --- |
| 旋律保留 | 原曲主旋律 1956 个起音中 93.0%（1819）以原音高、原起音保留 |
| 起音偏差 | 中位数 0.62 ms、最大 1.3 ms |
| 未保留的 137 音 | 全部是 30 ms 以内的装饰音（并入前音）和同音相邻合并 |
| 附加材料 | 914 音（和弦分解琶音 + 停顿填补），丰富单声部线条 |
| 时值 | 每音独立取整到 1/384 四分音符（±1.3 ms），起音按绝对时间对齐、误差不累积 |

单声部化丢掉的是持续和声与左右手对位——这部分蜂鸣器本来就无法表现，改为在和弦起音处
以琶音把和声"点"出来。旧的精简解析版（15 段 / 1850 音）MIDI 留在原曲同目录
`..._MCU蜂鸣器版_精简解析.mid`，可 A/B 对比。

## 验证情况

- 转换脚本自检通过上述全部 `BuzzerPlayScore()` 校验项。
- 曲谱反向打包 MIDI 与原曲逐音比对通过（旋律 93.0% 保留、起音最大偏差 1.3 ms）。
- 当前 `sentry_omni_gimbal` / F4 配置 Release 整机交叉编译通过，无本目录相关告警；
  `buzzer_music.c` / `rinascita_suite.c` 另过 `-Wall -Wextra -Werror` 严格编译。
- 用 `--undefined=kRinascitaSuite` 强制链接确认曲谱可完整进入固件（Flash 103444 → 137932 字节）。
  未调用时数据被 `--gc-sections` 回收，不占 Flash。
- **尚未进行真实硬件验证**：需要上板确认音色听感、响度、琶音段的听感、长曲目中途的报警
  抢占与续播，以及 `BuzzerMusicSuiteService()` 补队节奏（见 `Modules/alarm/buzzer.md` 的
  上板验收清单）。
