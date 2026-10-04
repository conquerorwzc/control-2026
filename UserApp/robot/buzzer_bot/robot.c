#include "robot.h"
#include "wuci_youci_suite.h"
#include "bsp_dwt.h"
#include "buzzer.h"
#include "rinascita_suite.h"
#include "robot_config.h"
#include "wuci_youci_suite.h"

/* 第一段：模块内置的 5 种提示音，按严重程度递增演示。
 * step_ms 为该音效时长 + 间隔（就绪 240 / 成功 560 / 警告 960 / 错误 1350 / 通信丢失 920）。 */
typedef struct {
    BuzzerPattern_e pattern;
    uint16_t step_ms;
} BuzzerBotNotifyStep_s;

static const BuzzerBotNotifyStep_s kBuzzerBotNotify[] = {
    {BUZZER_PATTERN_READY, 450},      /* 就绪：784 Hz 短音 */
    {BUZZER_PATTERN_SUCCESS, 800},    /* 成功：523→659→784 上行 */
    {BUZZER_PATTERN_WARNING, 1200},   /* 警告：880 Hz 两声 */
    {BUZZER_PATTERN_ERROR, 1600},     /* 错误：1 kHz 长音 + 523 Hz */
    {BUZZER_PATTERN_COMM_LOST, 1150}, /* 通信丢失：784 Hz 三连短音 */
};

/* 第二段：5 个报警层级同时置位，各自的音效不同；模块只播最高等级，
 * 自高到低逐级清除即可听到"高等级遮蔽低等级"和每一级的音效。 */
static const BuzzerPattern_e kBuzzerBotLevels[] = {
    BUZZER_PATTERN_ERROR,     /* ALARM_LEVEL_HIGH */
    BUZZER_PATTERN_WARNING,   /* ALARM_LEVEL_ABOVE_MEDIUM */
    BUZZER_PATTERN_COMM_LOST, /* ALARM_LEVEL_MEDIUM */
    BUZZER_PATTERN_SUCCESS,   /* ALARM_LEVEL_BELOW_MEDIUM */
    BUZZER_PATTERN_READY,     /* ALARM_LEVEL_LOW */
};

enum {
    kBuzzerBotNotifyCount = sizeof(kBuzzerBotNotify) / sizeof(kBuzzerBotNotify[0]),
    kBuzzerBotLevelCount = sizeof(kBuzzerBotLevels) / sizeof(kBuzzerBotLevels[0]),
};

typedef enum {
    kBotPhaseDelay,   /* 上电等待蜂鸣器就绪 */
    kBotPhaseNotify,  /* 逐一演示内置提示音 */
    kBotPhaseLevel,   /* 演示报警层级遮蔽 */
    kBotPhaseMusic,   /* 播放组曲 */
} BuzzerBotPhase_e;

static BuzzerBotPhase_e bot_phase;
static float phase_mark_ms;
static float service_mark_ms;
static uint32_t step_index;
static uint8_t music_submitted;

static void BuzzerBotEnterPhase(BuzzerBotPhase_e phase, float now_ms) {
    bot_phase = phase;
    phase_mark_ms = now_ms;
    step_index = 0;
}

static void BuzzerBotLevelSet(void) {
    for (uint32_t index = 0; index < kBuzzerBotLevelCount; index++) {
        BuzzerSetIndicator((uint16_t)(BUZZER_BOT_LEVEL_SOURCE_BASE + index), kBuzzerBotLevels[index],
                           (AlarmLevel_e)index, NULL);
    }
}

static void BuzzerBotLevelClear(void) {
    for (uint32_t index = 0; index < kBuzzerBotLevelCount; index++) {
        BuzzerClearIndicator((uint16_t)(BUZZER_BOT_LEVEL_SOURCE_BASE + index));
    }
}

void RobotInit() {
    bot_phase = kBotPhaseDelay;
    phase_mark_ms = DWT_GetTimeline_ms();
    service_mark_ms = phase_mark_ms;
    step_index = 0;
    music_submitted = 0;
}

void RobotTask() {
    float now_ms = DWT_GetTimeline_ms();

    switch (bot_phase) {
        case kBotPhaseDelay:
            /* 蜂鸣器由守护任务 BuzzerInit() 初始化，就绪后再开始演示 */
            if (now_ms - phase_mark_ms >= BUZZER_BOT_START_DELAY_MS) {
                BuzzerBotEnterPhase(kBotPhaseNotify, now_ms);
                BuzzerNotify(kBuzzerBotNotify[0].pattern, NULL);
            }
            break;

        case kBotPhaseNotify:
            if (now_ms - phase_mark_ms >= kBuzzerBotNotify[step_index].step_ms) {
                step_index++;
                phase_mark_ms = now_ms;
                if (step_index < kBuzzerBotNotifyCount) {
                    BuzzerNotify(kBuzzerBotNotify[step_index].pattern, NULL);
                } else {
                    BuzzerBotEnterPhase(kBotPhaseLevel, now_ms);
                    BuzzerBotLevelSet();
                }
            }
            break;

        case kBotPhaseLevel:
            if (now_ms - phase_mark_ms >= BUZZER_BOT_LEVEL_STEP_MS) {
                BuzzerClearIndicator((uint16_t)(BUZZER_BOT_LEVEL_SOURCE_BASE + step_index));
                step_index++;
                phase_mark_ms = now_ms;
                if (step_index >= kBuzzerBotLevelCount) {
                    BuzzerBotLevelClear();  /* 兜底清干净，避免持续报警遮蔽音乐 */
                    BuzzerBotEnterPhase(kBotPhaseMusic, now_ms);
                }
            }
            break;

        case kBotPhaseMusic:
            if (!music_submitted) {
                music_submitted = BuzzerMusicSuitePlay(&BUZZER_BOT_SUITE, NULL) == BUZZER_OK;
            }
            break;

        default:
            break;
    }

    /* 组曲分段按周期补队；无播放任务时该调用是安全的空操作 */
    if (now_ms - service_mark_ms >= BUZZER_BOT_SERVICE_PERIOD_MS) {
        service_mark_ms = now_ms;
        BuzzerMusicSuiteService();
    }
}
