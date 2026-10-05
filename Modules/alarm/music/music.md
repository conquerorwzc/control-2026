# 蜂鸣器音乐文件

本目录存放由 MIDI 转换而来的蜂鸣器常量曲谱和配套的组曲播放接口。音乐数据完全遵循
`Modules/alarm/buzzer.h` 的曲谱约束，播放由既有 `BuzzerTask()` 推进，本目录不改动
蜂鸣器模块和任何机器人逻辑。

## 文件

| 文件 | 说明 |
| --- | --- |
| `rinascita_suite.c/.h` | 《鸣潮 黎那汐塔版本组曲》常量曲谱，`tools/midi_to_score.py` 生成，勿手工改动 |
| `wuci_youci_suite.c/.h` | 《无刺有刺》常量曲谱（由 MXL → MIDI → 曲谱链路生成），勿手工改动 |
| `buzzer_music.c/.h` | 通用组曲播放接口：把多段曲谱按 FIFO 顺序提交给蜂鸣器模块，长组曲流式补队 |
| `tools/midi_to_score.py` | MIDI → 蜂鸣器曲谱转换脚本（仅 Python3 标准库） |
| `tools/musicxml_to_midi.py` | MusicXML/MXL → MIDI：多份谱按顺序拼成一首，输出旋律/伴奏双轨 |
| `tools/score_to_midi.py` | 反向工具：把生成的曲谱 C 文件还原为 MIDI，用于试听/校对音乐性 |
| `tools/score_to_wav.py` | 仿真工具：把曲谱按 `buzzer.c` 合成链模拟成 PWM 方波 WAV，贴近真机听感 |
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

## 从 MXL / MusicXML 生成（OMR 识别谱面）

有 MusicXML/MXL（如 Audiveris 识别 PNG 五线谱的产物）时优先走这条链路，音高时值
零误差，不必做不可靠的光学识别。`tools/musicxml_to_midi.py` 把多份谱按顺序拼成一首
曲子，输出 `format 1` MIDI（轨 0 指挥轨放速度、轨 1 旋律、轨 2 伴奏），可直接接
`midi_to_score.py`：

```bash
python3 Modules/alarm/music/tools/musicxml_to_midi.py 上.mxl 下.mxl \
    --out 无刺有刺.mid --tempo 90 --title "无刺有刺"
python3 Modules/alarm/music/tools/midi_to_score.py 无刺有刺.mid \
    --out-dir Modules/alarm/music --name wuci_youci_suite --title "无刺有刺"
```

- `--tempo` 必填：OMR 导出的 MusicXML 通常**没有速度标记**，需按原曲指定 BPM。
- `--melody-part` / `--accomp-part` 指定每份谱里哪个声部作为旋律/伴奏（默认 0/1）。
- `--octave-shift` 按谱面印刷的 8va/8vb 记号修正音高（见下方"OMR 核对"）。
- 处理了和弦（同时发声）、连音线（合并为长音）、弱起小节、附点和 divisions 变化；
  休止符只推进时间。
- 注意 OMR 小节时值可能略有出入（本例弱起小节为 14/16 单位），音符顺序与相对时值
  仍准确，不影响蜂鸣器单声部播放。

## OMR 核对经验（用原谱面 PNG 校验识别结果）

拿到 MusicXML 后建议用原谱面 PNG 逐项核对，本次《无刺有刺》的核对结果：

| 项目 | 结果 |
| --- | --- |
| 速度 | 谱面 ♩=90，与指定值一致 |
| 调号/拍号 | 1 个升号、4/4 一致 |
| 弱起小节 | 一致 |
| **拍号变更** | 谱面第 16 小节 4/4→6/8、第 19 小节回到 4/4；OMR **漏了 `<time>` 元数据**，但音符时值正确（6/8 小节 = 12 单位），因转换器按累计时值排布，**节奏不受影响** |
| **8va/8vb 八度记号** | OMR 存的是**书面音高**（画在谱表上的位置），且 `octave-shift type` 与印刷记号方向相反。需按印刷记号修正：`type="down"`（印刷 8va）→ +12，`type="up"`（印刷 8vb）→ -12，即 `--octave-shift` |

**A/B 试听定位法**：对存疑处（如八度）各出一版"原样"和"修正"的方波 WAV，只渲染可疑区间
对比，听感定夺。本例 8va 一处影响旋律音域 E4–A5 → E5–A6（330–880 Hz → 659–1760 Hz），
区间外逐音一致。

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

## 方波 WAV 仿真（贴近真机听感）

MIDI 只能校对音符对不对；要听"蜂鸣器实际会发出什么声音"，用 `tools/score_to_wav.py`
把曲谱按 `Modules/alarm/buzzer.c` 的真实合成链逐采样渲染成 PWM 方波 WAV：

```bash
# 全曲（约 16 分钟，84 MB）
python3 Modules/alarm/music/tools/score_to_wav.py Modules/alarm/music/rinascita_suite.c \
    --out 鸣潮_黎那汐塔版本组曲_方波模拟.wav

# 只截一段试听（长曲不必全渲）
python3 Modules/alarm/music/tools/score_to_wav.py Modules/alarm/music/rinascita_suite.c \
    --out 试听_开头60s.wav --start 0 --duration 60
```

仿真的合成链（逐项对应 `buzzer.c`）：

| 环节 | 实现 |
| --- | --- |
| 波形 | 方波，频率 = 音符频率，休止符静音 |
| 脉宽 | `pulse_width_permille × 包络增益 × velocity / 127000`（同 `BuzzerApplyVoice()`），再乘 `BuzzerSetStrength()` 千分比（`--volume`） |
| PWM 量化 | 按 1 MHz 计数时钟取整、占空比钳位 50%（蜂鸣器 TIM 独占约 1 MHz） |
| ADSR | attack/decay/release 未满 10 ms 按下限执行（`BuzzerStageValid`），release 在音符末尾不延长时值 |
| 滑音 | 前一发声频率线性滑入，休止符后不滑音 |
| 颤音/颤幅 | 颤音按**频率比例**（非音分）调制，颤幅调占空比；按实际播放时间推进、不随变速加倍 |
| 采样 | 44.1 kHz / 16 位单声道（`--rate` 可改） |

**局限**：这是电端 PWM 波形。真机蜂鸣器的换能器（型号/驱动电路未确认）频响、谐振和
失真会明显改变听感——尤其 200 Hz 以下和 4 kHz 以上的段落，实际可能更闷或更尖。
低频段真机响度也会显著衰减，WAV 里听得很清楚的低音在板上未必可闻。

验证：对渲染 WAV 做过零测频抽样，10 个长音与曲谱常量偏差 ≤2.6 Hz（0.15 s 窗的
计数量化步长内）；休止符幅度为 0。

## 低频与蜂鸣器可听区

电性能上模块支持 20 Hz~8 kHz（`BUZZER_MIN_FREQUENCY_MILLIHZ`），但**换能器决定听感**：
典型压电蜂鸣器谐振在 2~4 kHz，**500 Hz 以下急剧衰减，200 Hz 以下基本听不见**
（`Modules/alarm/buzzer.md` 注明硬件型号未确认，需上板实测）。因此：

- 合并多声部（`--merge-voices`）时低音声部会把音域拉到 E2（82 Hz），这些音在蜂鸣器上
  会"消失"，听起来节奏断续、音高忽有忽无；
- 用 `--min-pitch` 抬高下限可规避：**低于下限的音逐八度上移**。参数可写 MIDI 音号
  （57=A3）或频率 Hz（220），>127 自动按 Hz 解释。
- **下限应取旋律自身的自然最低音**，而不是固定值：取高了会把旋律低音也抬八度、破坏原曲。
  例：黎那汐塔旋律低至 155.6 Hz，用 `--min-pitch 220` 会把这段旋律改掉，应改用
  `--min-pitch 156`（= MIDI 51，旋律下限）——旋律原样保留，只把低于旋律音域的伴奏低音
  抬进来。《无刺有刺》旋律下限 220 Hz，故其下限取 220 正好。
- 抬升的代价是低音声部进入中音区，会与旋律抢位置——单声部固有限制。若既要保旋律又要
  提升整体可听性，可考虑**整曲上移**（如 +12 半音），保持音程关系不变。

## 本次曲目配置（两套曲谱统一）

最终配置：**合并双声部 + 旋律自然最低音作下限 + 关闭琶音**。

```bash
# 无刺有刺（旋律下限 220 Hz）
python3 tools/midi_to_score.py 无刺有刺.mid --out-dir <目录> --name wuci_youci_suite \
    --title "无刺有刺" --merge-voices --min-pitch 156 --chord-roll 0
# 黎那汐塔组曲（旋律下限 155.6 Hz）
python3 tools/midi_to_score.py 鸣潮_黎那汐塔版本组曲.mid --out-dir <目录> --name rinascita_suite \
    --title "鸣潮 黎那汐塔版本组曲" --merge-voices --min-pitch 156 --chord-roll 0
```

| 曲目 | 段/条目 | 音域 | 时长 |
| --- | --- | --- | --- |
| 无刺有刺 | 3 段 / 282 条目 | 164.8 ~ 1760 Hz | 102.0 s |
| 黎那汐塔组曲 | 25 段 / 3088 条目 | 155.6 ~ 2489 Hz | 952.2 s |

**为何关琶音**：和弦分解插入的短琶音（39~52 ms）虽满足 10 ms 下限，但约一半落在
220~330 Hz 弱区，在压电蜂鸣器上低音补不进去、只剩高音跑动，听感反而碎。关掉后
（`--chord-roll 0`）每起音只出一个顶音，旋律线条干净。

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
