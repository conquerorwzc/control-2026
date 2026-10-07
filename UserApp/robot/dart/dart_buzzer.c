/**
 * @file dart_buzzer.c
 * @brief dart 发射架蜂鸣器提示实现, 封装 Modules/alarm 的 Tone/Score/Notify/Indicator
 */
#include "dart_buzzer.h"

#include <stddef.h>

#include "buzzer.h"
#include "buzzer_music.h"
#include "wuci_youci_suite.h"

/* 持续指示 source_id, 与告警级别共同区分来源 */
#define DART_BUZZER_SRC_FAULT 1
#define DART_BUZZER_SRC_RC_LOST 2

/* ---- 单发短音(毫赫兹 / ms) ---- */
static void PlayTone(uint32_t freq_millihz, uint16_t duration_ms, uint8_t velocity) {
  Buzzer_Tone_Config_s tone = {
      .frequency_millihz = freq_millihz,
      .duration_ms = duration_ms,
      .silence_ms = 0,
      .count = 1,
      .velocity = velocity,
      .voice = kBuzzerVoiceBright,
  };
  BuzzerPlayTone(&tone, NULL);
}

/* ---- 多音序列 ---- */
/* 校准开始: Do-Mi-So 渐高, "启动"感 */
static const Buzzer_Note_s kCaliStartNotes[] = {
    {261626, 24, 90, &kBuzzerVoiceSoft},
    {329628, 24, 100, &kBuzzerVoiceSoft},
    {391995, 40, 110, &kBuzzerVoiceBright},
    {0, 16, 0, NULL},
};
static const Buzzer_Score_s kCaliStartScore = {
    .notes = kCaliStartNotes,
    .note_count = sizeof(kCaliStartNotes) / sizeof(kCaliStartNotes[0]),
    .bpm = 180,
};

/* 校准完成: Mi-So 上行, "就绪"感 */
static const Buzzer_Note_s kCaliDoneNotes[] = {
    {329628, 28, 100, &kBuzzerVoiceSoft},
    {391995, 48, 115, &kBuzzerVoiceBright},
    {0, 12, 0, NULL},
};
static const Buzzer_Score_s kCaliDoneScore = {
    .notes = kCaliDoneNotes,
    .note_count = sizeof(kCaliDoneNotes) / sizeof(kCaliDoneNotes[0]),
    .bpm = 180,
};

/* 储能完成: So-Do' 上行, 比校准完成更亮, 提示"可以发射" */
static const Buzzer_Note_s kChargeDoneNotes[] = {
    {391995, 28, 105, &kBuzzerVoiceSoft},
    {523251, 52, 120, &kBuzzerVoiceBright},
    {0, 12, 0, NULL},
};
static const Buzzer_Score_s kChargeDoneScore = {
    .notes = kChargeDoneNotes,
    .note_count = sizeof(kChargeDoneNotes) / sizeof(kChargeDoneNotes[0]),
    .bpm = 180,
};

void DartBuzzerEnableOk(void) { PlayTone(880000, 50, 90); }

void DartBuzzerDisable(void) {
  DartBuzzerCaliBgmStop();  // 失能/急停即停 BGM
  // 只提示失能, 不清持续指示(失联指示由 RC 恢复清除, 故障指示由校准恢复清除)
  Buzzer_Tone_Config_s tone = {
      .frequency_millihz = 440000,
      .duration_ms = 70,
      .silence_ms = 50,
      .count = 2,
      .velocity = 95,
      .voice = kBuzzerVoiceNormal,
  };
  BuzzerPlayTone(&tone, NULL);
}

void DartBuzzerDebugMode(void) {
  Buzzer_Tone_Config_s tone = {
      .frequency_millihz = 1174659,  // D6
      .duration_ms = 40,
      .silence_ms = 40,
      .count = 2,
      .velocity = 85,
      .voice = kBuzzerVoiceBright,
  };
  BuzzerPlayTone(&tone, NULL);
}

void DartBuzzerCaliStart(void) { BuzzerPlayScore(&kCaliStartScore, 1, NULL); }

/* ---- 校准 BGM(无刺有刺组曲): 校准开始播放, 校准完成/故障/中止/失能即停 ---- */
void DartBuzzerCaliBgmStart(void) {
  BuzzerMusicSuiteStop();  // 防上一次组曲残留
  BuzzerMusicSuitePlay(&kWuciYouciSuite, NULL);
}

void DartBuzzerCaliBgmStop(void) { BuzzerMusicSuiteStop(); }

void DartBuzzerService(void) { BuzzerMusicSuiteService(); }  // 组曲分段补队, 须周期调用

void DartBuzzerCaliDone(void) {
  DartBuzzerCaliBgmStop();  // 校准完成即停 BGM
  BuzzerPlayScore(&kCaliDoneScore, 1, NULL);
}

void DartBuzzerChargeStart(void) { PlayTone(659000, 80, 95); }

void DartBuzzerChargeDone(void) {
  BuzzerPlayScore(&kChargeDoneScore, 1, NULL);
  BuzzerNotify(BUZZER_PATTERN_SUCCESS, NULL);
}

void DartBuzzerFire(void) {
  // 极短高音脉冲, 对应舵机释放瞬间
  PlayTone(1567981, 35, 127);  // G6
}

void DartBuzzerFireDone(void) { PlayTone(880000, 45, 80); }

void DartBuzzerCmdRejected(void) { BuzzerNotify(BUZZER_PATTERN_WARNING, NULL); }

void DartBuzzerFault(void) {
  DartBuzzerCaliBgmStop();  // 故障即停 BGM
  BuzzerNotify(BUZZER_PATTERN_ERROR, NULL);
  BuzzerSetIndicator(DART_BUZZER_SRC_FAULT, BUZZER_PATTERN_ERROR, ALARM_LEVEL_HIGH, NULL);
}

void DartBuzzerFaultCleared(void) { BuzzerClearIndicator(DART_BUZZER_SRC_FAULT); }

void DartBuzzerRcLost(void) {
  BuzzerSetIndicator(DART_BUZZER_SRC_RC_LOST, BUZZER_PATTERN_COMM_LOST, ALARM_LEVEL_MEDIUM, NULL);
}

void DartBuzzerRcOk(void) {
  BuzzerClearIndicator(DART_BUZZER_SRC_RC_LOST);
  PlayTone(880000, 40, 70);
}
