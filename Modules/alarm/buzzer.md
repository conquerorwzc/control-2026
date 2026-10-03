# 蜂鸣器模块

蜂鸣器模块提供单声部、非阻塞的音乐播放、提示音、持续状态指示、debug 编码和致命故障应急音。模块使用 Flash 中的常量曲谱和静态请求池，不在播放路径中调用 `malloc()`。

`buzzer.c` 管理全部播放和调度，直接使用通用 `Bsp/pwm/bsp_pwm`，不再保留独立蜂鸣器硬件适配文件。蜂鸣器通过 `PWMRegisterEx()` 独占板载 TIM，预计算输出帧并在请求版本仍有效时提交；2 kHz/50% 应急帧及应急业务状态只保留在模块中，BSP 仅锁存指定的通用 PWM 输出，不包含蜂鸣器、曲谱、报警或状态指示逻辑。

## 任务接入

系统已经在守护任务中周期调用：

```c
BuzzerInit();

for (;;) {
    DaemonTask();
    BuzzerTask();
    osDelay(10);
}
```

示例仅说明已有接线，本次没有修改 `UserApp/os_task.c`。调用 `BuzzerInit()` 前必须完成 CubeMX 生成的板级定时器、引脚和时钟初始化；F4 使用 TIM4_CH3，H7 使用 TIM12_CH2，不得同时交给其他模块使用。

`BuzzerInit()` 可重复调用且不会清除已经注册的报警实例，初始化失败后可重试。普通播放接口都是非阻塞的；它们只提交请求，实际输出由 `BuzzerTask()` 推进。未初始化时提交播放返回 `BUZZER_NOT_READY`，旧报警实例仍可提前注册。

服务函数使用 FreeRTOS 整数 tick 的实际差值推进，不依赖调用次数；允许 tick 回绕，但两次服务间隔必须短于一个完整 tick 周期。延迟服务时直接跳过已过期音段，不集中补播。普通请求、报警、取消和控制更新在下一次服务时生效；任务停调度时当前 PWM 可能保持，因此不存在脱离任务调度的固定响应上限。

## 优先级

输出优先级从高到低为：应急音、五级持续报警、一次性提示音、debug 编码、音乐。高优先级声音会抢占低优先级声音；音乐被抢占后从原音符剩余位置继续播放，一次性声音重新获得输出权后从头播放。持续报警必须通过 `BuzzerClearIndicator()` 或旧接口 `AlarmSetStatus(..., ALARM_OFF)` 清除。

同级一次性请求按 FIFO 播放。同级持续来源在完整模式结束后轮转，旧报警实例以 1 秒为轮转单位；更高等级持续故障仍会遮蔽低等级。正常静音、输出强度和停止只作用于音乐、debug 和一次性提示，不会清除或静音持续报警。

## 返回值与请求 ID

`request_id` 可以为 `NULL`；不为 `NULL` 时提交函数先置 0。请求 ID 非零只代表已接收的请求身份，不代表已经发声。取消只能使用仍在队列或播放中的请求 ID；完成或取消后的 ID 返回 `BUZZER_NOT_FOUND`。

| 返回值 | 含义 |
| --- | --- |
| `BUZZER_OK` | 提交已接收，或指定控制操作已完成；播放仍需服务任务 |
| `BUZZER_MERGED` | 与现有请求合并，返回原 ID；debug 冷却抑制时 ID 为 0 |
| `BUZZER_INVALID_ARGUMENT` | 空参数、越界枚举或非法参数值 |
| `BUZZER_NOT_READY` | 硬件尚未成功初始化 |
| `BUZZER_QUEUE_FULL` | 一次性等待槽或持续状态槽已满；未覆盖已有请求 |
| `BUZZER_NOT_FOUND` | 指定请求或持续来源不存在 |
| `BUZZER_WRONG_CONTEXT` | 在中断中提交音乐曲谱 |
| `BUZZER_PANIC_LATCHED` | 应急音已锁存，拒绝普通播放及控制操作 |

状态查询在初始化前和应急锁存后仍可使用；旧 `BuzzerBeep*()` 将任何提交失败映射为 0，成功接收映射为 1。

## 音乐播放

音符数组和其引用的音色必须从提交到排队、暂停、抢占、播放结束或取消期间一直有效且不可修改，推荐定义为 `static const`。提交时只复制曲谱描述（指针、音符数量、BPM）和重复次数，不复制音符数组及音色；曲谱描述本身可以是局部变量。

```c
static const Buzzer_Note_s kStartupNotes[] = {
    {261626, 96, 127, &kBuzzerVoiceSoft},
    {329628, 96, 127, &kBuzzerVoiceSoft},
    {391995, 192, 127, &kBuzzerVoiceBright},
    {0, 96, 127, &kBuzzerVoiceNormal},
};

static const Buzzer_Score_s kStartupScore = {
    .notes = kStartupNotes,
    .note_count = sizeof(kStartupNotes) / sizeof(kStartupNotes[0]),
    .bpm = 120,
};

BuzzerRequestId_t request_id;
BuzzerResult_e result = BuzzerPlayScore(&kStartupScore, 1, &request_id);
```

- `duration_ticks` 以 `BUZZER_QUARTER_TICKS` 为四分音符基准，96 表示四分音符。
- `repeat_count == 0` 表示无限循环；大于 0 表示播放指定次数。
- 音符频率使用毫赫兹；休止符的频率为 0。
- `BuzzerMidiToFrequency()` 返回 0 表示 MIDI 音符超出当前硬件支持范围。
- C 常量曲谱必须填写预计算频率，不能在静态初始化表达式里调用 `BuzzerMidiToFrequency()`；此函数用于运行时换算。
- `BuzzerPauseMusic()`、`BuzzerResumeMusic()`、`BuzzerSetMusicSpeed()`、`BuzzerCancel()` 和 `BuzzerStop()` 用于控制请求。
- 曲谱最多 128 个音符，BPM 为 30~300，每个音符在原始 BPM 下至少 10 ms；变速后短于服务间隔的音符可能被跳过。
- `BuzzerSetMusicSpeed()` 接收 250~4000 千分比，1000 表示原速。只改变音符时值，不改变音高或音色调制频率。
- 暂停作用于全部音乐请求且保持暂停标志，恢复后才继续；抢占和暂停均冻结音符进度、包络和调制进度。

预置音色为 `kBuzzerVoiceNormal`、`kBuzzerVoiceSoft`、`kBuzzerVoiceBright` 和 `kBuzzerVoiceSiren`。音色支持脉宽、ADSR、滑音、颤音和颤幅；调制在 10 ms 服务周期上更新，不能替代采样音频，也不保证响度与 PWM 占空比线性对应。

| 参数 | 范围与单位 |
| --- | --- |
| 非休止符频率 | 20000~8000000 毫赫兹，即 20~8000 Hz，不保证全范围听感 |
| `velocity` | 0~127，0 输出静音 |
| `pulse_width_permille` | 0~500，PWM 占空比不超过 50% |
| `attack_ms` / `decay_ms` / `release_ms` / `glide_ms` | 0 禁用；非零必须至少 10 ms，最大 65535 ms |
| `sustain_permille` / `tremolo_depth_permille` | 0~1000 |
| `vibrato_depth_permille` | 0~200，按频率比例调制而非音分 |
| `vibrato_hz` / `tremolo_hz` | 0 禁用；1~10 Hz，按实际播放时间更新，不随音乐速度加倍 |

包络在音符时长内执行，释放阶段位于音符末尾，不额外延长时值；短音符的包络阶段可能截断或重叠。滑音从前一音符到当前音符，休止符后不滑音，循环边界同样适用。音色指针为 `NULL` 时使用普通预设。

## 提示音和持续指示

```c
BuzzerNotify(BUZZER_PATTERN_SUCCESS, NULL);

BuzzerRequestId_t indicator_id;
BuzzerSetIndicator(7, BUZZER_PATTERN_COMM_LOST, ALARM_LEVEL_HIGH, &indicator_id);

BuzzerClearIndicator(7);
```

内置模式为就绪、成功、警告、错误、通信丢失。自定义定长提示可调用 `BuzzerPlayTone()`：

```c
Buzzer_Tone_Config_s tone = {
    .frequency_millihz = 1000000,
    .duration_ms = 120,
    .silence_ms = 80,
    .count = 3,
    .velocity = 100,
    .voice = kBuzzerVoiceSoft,
};
BuzzerPlayTone(&tone, NULL);
```

定长提示的全部配置和音色会复制，允许局部变量；每次响至少 10 ms，停顿为 0 或至少 10 ms，次数为 1~255，最后一次也包含指定停顿。`BuzzerSetStrength()` 接收 0~1000 千分比；静音时普通请求仍消耗播放时间，音乐需要冻结时应使用暂停。

持续指示默认支持 16 个来源。相同来源再次设置相同模式和等级时返回 `BUZZER_MERGED`，不会重复分配槽位。相同优先级的持续指示按最近服务时间轮转。

故障恢复后必须显式调用 `BuzzerClearIndicator()`；`BuzzerCancel()` 只取消一次性请求和音乐，不取消持续指示。

## Debug 编码

```c
BuzzerDebugCode(3, 42, NULL);
```

编码范围是 `00~99`。十位使用约 523 Hz 低音，个位使用约 1046.5 Hz 高音；非零数字播放对应数量的 80 ms 短音，每个短音后停 80 ms；数字 0 响 300 ms、停 80 ms。两个数字额外间隔 300 ms，末尾间隔 200 ms。

相同来源和编码在排队或播放期间合并，完成后 1 秒内重复提交会被抑制，冷却时间直接取 RTOS tick，不要求期间持续调用播放器。历史表保存最近 16 条完成编码；超过该历史容量会淘汰旧条目。`source_id` 为 16 位应用自定义标识，不要求是槽位下标，应用应避免两个故障源复用同一个标识。

## ISR 与致命故障

普通中断服务函数只能使用 `BuzzerPlayToneFromISR()`、`BuzzerNotifyFromISR()`、`BuzzerDebugCodeFromISR()`、`BuzzerSetIndicatorFromISR()` 和 `BuzzerClearIndicatorFromISR()`。音乐曲谱校验和 `BuzzerPlayScore()` 只允许任务上下文。

这些接口不分配堆、不给任务发送通知，不需要 `portYIELD_FROM_ISR()`。debug 的实时冷却查询使用 `xTaskGetTickCountFromISR()`，因此中断优先级必须满足 FreeRTOS 的可调用 API 限制：本仓库 NVIC 抢占优先级数值必须大于等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（当前为 5）。普通接口不应从 NMI、HardFault 等异常调用；致命异常仅使用应急音接口。

```c
void SomeIrqHandler(void) {
    BuzzerNotifyFromISR(BUZZER_PATTERN_ERROR, NULL);
}
```

`BuzzerPanicTone()` 直接锁存硬件输出为固定 2 kHz、50% 占空比，不使用 RTOS、队列、堆或延时；有效参数的普通提交和控制接口返回 `BUZZER_PANIC_LATCHED`，初始化、旧实例操作和状态查询也无法覆盖输出。锁存只在硬件复位后解除。

故障处理器接入示例（本次不修改真实异常处理器）：

```c
void HardFault_Handler(void) {
    __disable_irq();
    BuzzerPanicTone();
    for (;;) {
    }
}
```

应急音要求硬件此前已经初始化，否则返回 `BUZZER_NOT_READY`。在 RTOS 停止或普通中断关闭后仍可依靠硬件计数器保持输出，但无法保证 MCU 时钟、电源、定时器、引脚或 RAM 本身发生故障时继续工作。

## 兼容旧接口

旧接口仍然可用：

```c
Buzzer_config_s config = {
    .alarm_level = ALARM_LEVEL_HIGH,
    .loudness = 0.4f,
    .octave = OCTAVE_1,
};

BuzzzerInstance *alarm = BuzzerRegister(&config);
AlarmSetStatus(alarm, ALARM_ON);
AlarmSetStatus(alarm, ALARM_OFF);
BuzzerBeep(3);
```

旧 `BuzzerBeep*()` 保持 80 ms 响、80 ms 停、最多 6 声和 0/1 返回值。报警实例改为静态池，不接受空指针、越界等级、重复等级、非法音阶或 NaN/Inf 响度；重复注册返回 `NULL`，不会覆盖已有实例。

兼容性变化：
- 默认 `BuzzerBeep()` 仍为 784 Hz、30% 占空比；旧配置的 `loudness` 保持占空比含义，0 为静音，负值拒绝，大于 0.5 限制为 0.5。显式传 0 或负值不再回退到默认响度。
- NaN/Inf 通过浮点位表示检查，Release 快速浮点优化也不能跳过检查；频率 20~8000 Hz 以外拒绝。
- `OCTAVE_1`~`OCTAVE_7` 保留旧音阶映射，`OCTAVE_8` 补齐高音 Do；这些旧枚举并不是 MIDI 八度编号。
- 提示音不再阻挡更高优先级报警，旧 API 也进入同一个播放器；未初始化的 beep 安全失败。
- 旧结构仍可见，但修改字段没有并发保护；推荐通过 `AlarmSetStatus()` 改状态，新功能使用显式 API。无效字段会安全静音，不会作为硬件参数写入。
- 麦轮启动旋律里的重复 `i++` 属于调用方，本次未改；新旋律应采用常量曲谱提交，而不是重复修改实例字段并延时。

## 状态和资源

`BuzzerGetStatus()` 可查询初始化状态、应急锁存、静音、音乐速度、当前请求、当前优先级、等待数量、输出频率/占空比，以及拒绝、合并和完成计数。

- 请求池：1 个当前请求加最多 16 个等待请求。
- 持续指示：16 个来源。
- 旧报警等级：5 个静态实例。
- debug 抑制历史：16 条。
- 当前 F4/H7 交叉编译蜂鸣器自身静态 RAM 为 Debug 1944 / Release 1948 字节；通用 PWM 全部 16 槽和回滚快照另占 1472 字节，两模块合计 3416～3420 字节，不含曲谱、调用方数据或 HAL 句柄。相较原独立适配版本约 1.9 KiB 的记录增加约 1.46 KiB，但 PWM 池供所有 PWM 用户共用，旧 PWM 的动态实例分配已取消。
- PWM 计数频率：独占注册时申请接近 1 MHz，输出周期和比较值使用通用 BSP 的预装载同步更新；初始化始终安全静音，登记失败可重试。
- 普通输出占空比限制为 0~50%；`BuzzerSetStrength()` 只影响音乐、debug 和一次性提示，不影响持续报警。

`waiting_count` 是排队/挂起数量，不包含一个已启动保留位置；`request_count` 是一次性请求和音乐的总数，`indicator_count` 不包含五个旧报警实例。`completed_count` 只统计一次性请求及有限曲谱的自然结束。输出频率和占空比为最后一次提交硬件的控制值，不是外部测量结果；运行中的周期变更在下一次定时器更新时同步生效，低频时可能额外等待一个 PWM 周期，静音和应急音则强制更新。

硬件蜂鸣器型号和驱动电路尚未确认，因此“音色”和“强度”仅表示 PWM 控制参数。最终频率范围、听感和响度需要在目标板上验收。

## 验证记录与上板验收

软件验证在仓库外的隔离临时目录进行，直接编译真实 `buzzer.c` 和 `Bsp/pwm/bsp_pwm.c`，只替换 FreeRTOS 时钟、CMSIS 中断操作、HAL 入口及定时器寄存器；未为仓库新增测试框架。

- F4 与 H7 寄存器模型各 24 个场景，在 ASan/UBSan 常规优化和快速浮点优化下全部通过，共 96 次场景执行。
- PWM 另外覆盖 40 个场景 × 两 MCU × 两种优化，共 160 次；包括共享隔离、登记失败回滚、DMA 及失败恢复、16/32 位计数、APB2/TIMPRE、浮点精度，以及提交/启动/停止中触发固定输出锁存。
- 回归覆盖注册边界、全部五级等级、重复注册、空及伪造实例、NaN/Inf/负响度、零响度、重复初始化和失败重试、旧 beep 次数与 80/80 ms 节奏、队列容量及交错提交/取消、频率/周期/占空比一致性。
- 新功能覆盖曲谱结束、休止符、有限与无限循环、速度和音色时基、暂停与取消、音乐抢占恢复、debug 中断后重播、持续来源轮转及旧实例混合轮转、debug 合并及实际时间冷却、tick 回绕和服务超时、ISR 屏蔽状态保存、ADSR/滑音/颤音/颤幅、低频连续调制、应急锁存及普通寄存器提交中触发故障。
- 当前 `sentry_omni_gimbal` / F4 配置 Debug、Release 整机编译通过，生成 `.elf`、`.hex`、`.bin`；没有为了验证修改机器人或 MCU 选择。仓库既有的非蜂鸣器告警未处理。
- F4/H7 的 PWM BSP 与蜂鸣器各自使用真实 HAL/CMSIS/FreeRTOS 头文件完成 Debug、Release 独立交叉编译，`-Wall -Wextra -Werror` 通过。H7 仅验证模块编译，不代表 H7 整机集成已验证。
- `-fstack-usage` 显示 `BuzzerTask()` 本身的栈帧为 Debug 88 字节、Release 152 字节，分别比迁移前增加 24 字节；这不是完整调用链或中断上下文栈预算。现有守护任务 128-word 栈未修改，仍需上板检查最小剩余栈。

**尚未进行真实硬件验证。** 上板至少检查：
- 20 Hz、261.626 Hz、784 Hz、2 kHz、8 kHz 的实际周期，以及 0%、12.5%、30%、50% 占空比；重点检查换频时 ARR/CCR 一致和 16 位 ARR 范围。
- 音乐期间触发五级报警、多个同级来源和 debug 提示，检查下一次服务抢占、音乐续播和完整 debug 编码；同时检查低频变调是否连续。
- 压力提交、满队列、暂停恢复、清除故障来源及守护任务延迟，检查状态计数与剩余栈；模拟器不证明实际中断延迟或最坏执行时间。
- 在硬件正常初始化后停止 RTOS 服务、关闭普通中断，再调用应急音，确认 2 kHz 输出持续且普通控制不能关闭。
- 验证同 TIM 共享通道的隔离；板载蜂鸣器 TIM 为独占资源，其他模块不得登记其其他通道，不能通过应用层直接改 PSC/ARR 绕过资源检查。
- 确认蜂鸣器类型、驱动电路、温升、功耗、不同预设听感及提示/故障模式的可辨认程度，再决定实际使用的频率和强度范围。
