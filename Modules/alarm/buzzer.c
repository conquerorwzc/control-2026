#include "buzzer.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "bsp_pwm.h"
#include "main.h"
#include "task.h"

const Buzzer_Voice_Config_s kBuzzerVoiceNormal = {.pulse_width_permille = 500, .sustain_permille = 1000};
const Buzzer_Voice_Config_s kBuzzerVoiceSoft = {
    .pulse_width_permille = 250, .attack_ms = 30, .decay_ms = 40, .sustain_permille = 700, .release_ms = 40};
const Buzzer_Voice_Config_s kBuzzerVoiceBright = {
    .pulse_width_permille = 125, .attack_ms = 10, .decay_ms = 20, .sustain_permille = 850, .release_ms = 20};
const Buzzer_Voice_Config_s kBuzzerVoiceSiren = {
    .pulse_width_permille = 500,
    .sustain_permille = 1000,
    .glide_ms = 100,
    .vibrato_depth_permille = 150,
    .tremolo_depth_permille = 200,
    .vibrato_hz = 2,
    .tremolo_hz = 4,
};

enum { kRequestCapacity = BUZZER_WAITING_CNT + 1, kDebugHistoryCapacity = 16 };

typedef enum { REQUEST_TONE, REQUEST_PATTERN, REQUEST_DEBUG, REQUEST_SCORE } RequestKind_e;
typedef enum { OWNER_NONE, OWNER_REQUEST, OWNER_INDICATOR, OWNER_LEGACY } OwnerKind_e;

typedef struct {
    BuzzerRequestId_t id;
    uint64_t progress_us;
    uint64_t voice_us;
    uint64_t cycle_us;
    RequestKind_e kind;
    uint8_t started;
    union {
        Buzzer_Tone_Config_s tone;
        BuzzerPattern_e pattern;
        struct {
            uint16_t source_id;
            uint8_t code;
        } debug;
        struct {
            Buzzer_Score_s score;
            uint16_t repeat_count;
        } music;
    } data;
} BuzzerRequest_s;

typedef struct {
    BuzzerRequestId_t id;
    uint64_t progress_us;
    uint32_t last_served;
    uint16_t source_id;
    BuzzerPattern_e pattern;
    AlarmLevel_e level;
} BuzzerIndicator_s;

typedef struct {
    TickType_t completed_tick;
    uint16_t source_id;
    uint8_t code;
    uint8_t valid;
} BuzzerDebugHistory_s;

typedef struct {
    OwnerKind_e kind;
    uint8_t index;
    BuzzerRequestId_t id;
} BuzzerOwner_s;

typedef struct {
    BuzzerOwner_s owner;
    BuzzerRequest_s request;
    BuzzerIndicator_s indicator;
    BuzzzerInstance legacy;
    uint64_t legacy_progress_us;
} BuzzerSnapshot_s;

typedef struct {
    uint32_t frequency_millihz;
    uint16_t duration_ms;
} BuzzerSegment_s;

typedef struct {
    const BuzzerSegment_s *segments;
    uint8_t count;
} BuzzerPattern_s;

static const BuzzerSegment_s kReady[] = {{784000, 120}, {0, 120}};
static const BuzzerSegment_s kSuccess[] = {{523000, 100}, {659000, 100}, {784000, 160}, {0, 200}};
static const BuzzerSegment_s kWarning[] = {{880000, 120}, {0, 120}, {880000, 120}, {0, 600}};
static const BuzzerSegment_s kError[] = {{1000000, 300}, {0, 150}, {523000, 300}, {0, 600}};
static const BuzzerSegment_s kCommLost[] = {
    {784000, 80}, {0, 80}, {784000, 80}, {0, 80}, {784000, 80}, {0, 680}};
static const BuzzerPattern_s kPatterns[] = {{kReady, sizeof(kReady) / sizeof(kReady[0])},
                                          {kSuccess, sizeof(kSuccess) / sizeof(kSuccess[0])},
                                          {kWarning, sizeof(kWarning) / sizeof(kWarning[0])},
                                          {kError, sizeof(kError) / sizeof(kError[0])},
                                          {kCommLost, sizeof(kCommLost) / sizeof(kCommLost[0])}};
static const uint32_t kOctaveFrequencies[] = {523000, 587000, 659000, 698000, 784000, 880000, 988000, 1046502};
static const uint32_t kMidiOctave[] = {261626, 277183, 293665, 311127, 329628, 349228,
                                     369994, 391995, 415305, 440000, 466164, 493883};
static const int16_t kSine[] = {0,    195,  383,  556,  707,  831,  924,  981,  1000, 981,  924,
                               831,  707,  556,  383,  195,  0,    -195, -383, -556, -707, -831,
                               -924, -981, -1000, -981, -924, -831, -707, -556, -383, -195};

static BuzzerRequest_s requests[kRequestCapacity];
static BuzzerIndicator_s indicators[BUZZER_SOURCE_CNT];
static BuzzerDebugHistory_s debug_history[kDebugHistoryCapacity];
static BuzzzerInstance legacy_instances[BUZZER_DEVICE_CNT];
static uint8_t legacy_registered[BUZZER_DEVICE_CNT];
static uint64_t legacy_progress[BUZZER_DEVICE_CNT];
static uint32_t legacy_served[BUZZER_DEVICE_CNT];
static BuzzerRequestId_t legacy_revision[BUZZER_DEVICE_CNT];
static BuzzerOwner_s current_owner;
static BuzzerSnapshot_s service_snapshot;
static BuzzerRequestId_t next_id;
static uint32_t served_serial;
static uint32_t change_epoch;
static TickType_t last_tick;
static uint32_t tick_remainder;
static uint8_t initialized;
static volatile uint8_t panic_latched;
static PWMInstance *pwm_instance;
static PWM_Output_s panic_output;
static uint8_t initializing;
static uint8_t service_busy;
static uint8_t music_paused;
static uint8_t muted;
static uint16_t music_speed = 1000;
static uint16_t strength = 1000;
static uint32_t rejected_count;
static uint32_t merged_count;
static uint32_t completed_count;
static uint32_t output_frequency;
static uint16_t output_duty;

static uint32_t BuzzerLock(void) {
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return interrupt_mask;
}

static void BuzzerUnlock(uint32_t interrupt_mask) {
    __DMB();
    __set_PRIMASK(interrupt_mask);
}

static BuzzerResult_e BuzzerReject(BuzzerResult_e result) {
    uint32_t interrupt_mask = BuzzerLock();
    rejected_count++;
    BuzzerUnlock(interrupt_mask);
    return result;
}

static BuzzerResult_e BuzzerReadiness(void) {
    if (panic_latched) return BUZZER_PANIC_LATCHED;
    return initialized ? BUZZER_OK : BUZZER_NOT_READY;
}

static uint8_t BuzzerFrequencyValid(uint32_t frequency) {
    return frequency >= BUZZER_MIN_FREQUENCY_MILLIHZ && frequency <= BUZZER_MAX_FREQUENCY_MILLIHZ;
}

static uint8_t BuzzerFloatValid(const float *value) {
    uint32_t bits;
    memcpy(&bits, value, sizeof(bits));
    return (bits & 0x7f800000UL) != 0x7f800000UL;
}

static uint16_t BuzzerLegacyDuty(float loudness) {
    if (loudness <= 0) return 0;
    if (loudness >= 0.5f) return 500;
    return (uint16_t)(loudness * 1000.0f);
}

static uint8_t BuzzerStageValid(uint16_t duration) { return duration == 0 || duration >= 10; }

static uint8_t BuzzerVoiceValid(const Buzzer_Voice_Config_s *voice) {
    return voice != NULL && voice->pulse_width_permille <= 500 && voice->sustain_permille <= 1000 &&
           voice->vibrato_depth_permille <= 200 && voice->tremolo_depth_permille <= 1000 && voice->vibrato_hz <= 10 &&
           voice->tremolo_hz <= 10 && BuzzerStageValid(voice->attack_ms) && BuzzerStageValid(voice->decay_ms) &&
           BuzzerStageValid(voice->release_ms) && BuzzerStageValid(voice->glide_ms);
}

static uint64_t BuzzerPatternDuration(BuzzerPattern_e pattern) {
    uint64_t duration = 0;
    for (uint8_t segment = 0; segment < kPatterns[pattern].count; segment++) {
        duration += (uint64_t)kPatterns[pattern].segments[segment].duration_ms * 1000;
    }
    return duration;
}

static uint32_t BuzzerDigitDuration(uint8_t digit) { return digit == 0 ? 380 : (uint32_t)digit * 160; }

static BuzzerPriority_e BuzzerRequestPriority(const BuzzerRequest_s *request) {
    if (request->kind == REQUEST_SCORE) return BUZZER_PRIORITY_MUSIC;
    if (request->kind == REQUEST_DEBUG) return BUZZER_PRIORITY_DEBUG;
    return BUZZER_PRIORITY_NOTIFY;
}

static uint8_t BuzzerSameOwner(BuzzerOwner_s first, BuzzerOwner_s second) {
    return first.kind == second.kind && first.index == second.index && first.id == second.id;
}

static uint8_t BuzzerOwnerValid(BuzzerOwner_s owner) {
    if (owner.kind == OWNER_REQUEST) return requests[owner.index].id == owner.id && owner.id != 0;
    if (owner.kind == OWNER_INDICATOR) return indicators[owner.index].id == owner.id && owner.id != 0;
    if (owner.kind == OWNER_LEGACY) {
        return legacy_registered[owner.index] && legacy_revision[owner.index] == owner.id &&
               legacy_instances[owner.index].alarm_state == ALARM_ON;
    }
    return 0;
}

static BuzzerRequestId_t BuzzerAllocateId(void) {
    uint8_t collision;
    do {
        next_id++;
        if (next_id == 0) next_id++;
        collision = 0;
        for (uint8_t index = 0; index < kRequestCapacity; index++) collision |= requests[index].id == next_id;
        for (uint8_t index = 0; index < BUZZER_SOURCE_CNT; index++) collision |= indicators[index].id == next_id;
    } while (collision);
    return next_id;
}

static BuzzerResult_e BuzzerSubmit(BuzzerRequest_s *request, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    uint8_t free_index = kRequestCapacity;
    uint8_t count = 0;
    uint8_t has_foreground = 0;
    for (uint8_t index = 0; index < kRequestCapacity; index++) {
        if (requests[index].id == 0) {
            if (free_index == kRequestCapacity) free_index = index;
        } else {
            count++;
            has_foreground |= requests[index].started;
        }
    }
    if (result == BUZZER_OK && (free_index == kRequestCapacity || (!has_foreground && count >= BUZZER_WAITING_CNT))) {
        result = BUZZER_QUEUE_FULL;
    }
    if (result == BUZZER_OK) {
        request->id = BuzzerAllocateId();
        requests[free_index] = *request;
        if (request_id != NULL) *request_id = request->id;
        change_epoch++;
    } else {
        rejected_count++;
    }
    BuzzerUnlock(interrupt_mask);
    return result;
}

static BuzzerResult_e BuzzerSubmitTone(const Buzzer_Tone_Config_s *config, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if (config == NULL || !BuzzerFrequencyValid(config->frequency_millihz) || config->count == 0 ||
        config->duration_ms < 10 || !BuzzerStageValid(config->silence_ms) || config->velocity > 127 ||
        !BuzzerVoiceValid(&config->voice)) {
        return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    }
    BuzzerRequest_s request = {
        .kind = REQUEST_TONE,
        .cycle_us = (uint64_t)(config->duration_ms + config->silence_ms) * config->count * 1000,
    };
    request.data.tone = *config;
    return BuzzerSubmit(&request, request_id);
}

void BuzzerInit(void) {
    if (__get_IPSR() != 0) return;
    uint32_t interrupt_mask = BuzzerLock();
    if (initialized || initializing) {
        BuzzerUnlock(interrupt_mask);
        return;
    }
    initializing = 1;
    BuzzerUnlock(interrupt_mask);
    PWM_Ext_Init_Config_s hardware = {
#if defined(STM32F407xx)
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
#elif defined(STM32H723xx)
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
#else
#error "Unsupported board buzzer MCU"
#endif
        .frequency_millihz = 1000000,
        .counter_hz = 1000000,
        .exclusive_timer = 1,
    };
    PWMResult_e result = pwm_instance ? PWM_OK : PWMRegisterEx(&hardware, &pwm_instance);
    uint8_t ready = result == PWM_OK && PWMPrepareOutput(pwm_instance, 2000000, 500, &panic_output) == PWM_OK;
    TickType_t now = xTaskGetTickCount();
    interrupt_mask = BuzzerLock();
    if (ready) {
        last_tick = now;
        initialized = 1;
    }
    initializing = 0;
    BuzzerUnlock(interrupt_mask);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config) {
    if (config == NULL || (uint32_t)config->alarm_level >= BUZZER_DEVICE_CNT || (uint32_t)config->octave > OCTAVE_8 ||
        !BuzzerFloatValid(&config->loudness) || config->loudness < 0) {
        BuzzerReject(BUZZER_INVALID_ARGUMENT);
        return NULL;
    }
    float loudness = config->loudness > 0.5f ? 0.5f : config->loudness;
    uint8_t index = (uint8_t)config->alarm_level;
    uint32_t interrupt_mask = BuzzerLock();
    if (legacy_registered[index]) {
        rejected_count++;
        BuzzerUnlock(interrupt_mask);
        return NULL;
    }
    legacy_instances[index] = (BuzzzerInstance){.loudness = loudness,
                                              .octave = config->octave,
                                              .alarm_level = config->alarm_level,
                                              .alarm_state = ALARM_OFF};
    legacy_registered[index] = 1;
    legacy_revision[index] = 1;
    change_epoch++;
    BuzzerUnlock(interrupt_mask);
    return &legacy_instances[index];
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state) {
    if (buzzer == NULL) return;
    if ((uint32_t)state > ALARM_ON) {
        BuzzerReject(BUZZER_INVALID_ARGUMENT);
        return;
    }
    uint32_t interrupt_mask = BuzzerLock();
    for (uint8_t index = 0; index < BUZZER_DEVICE_CNT; index++) {
        if (legacy_registered[index] && buzzer == &legacy_instances[index]) {
            if (buzzer->alarm_state != state) {
                buzzer->alarm_state = state;
                legacy_progress[index] = 0;
                legacy_revision[index]++;
                change_epoch++;
            }
            BuzzerUnlock(interrupt_mask);
            return;
        }
    }
    rejected_count++;
    BuzzerUnlock(interrupt_mask);
}

uint8_t BuzzerBeep(uint8_t count) { return BuzzerBeepWithFreq(SoFreq, count); }

uint8_t BuzzerBeepWithFreq(uint16_t frequency, uint8_t count) {
    Buzzer_Beep_Config_s config = {.frequency = frequency, .count = count, .loudness = 0.3f};
    return BuzzerBeepWithConfig(&config);
}

uint8_t BuzzerBeepWithConfig(const Buzzer_Beep_Config_s *config) {
    if (config == NULL || config->count == 0 || !BuzzerFloatValid(&config->loudness) || config->loudness < 0) {
        BuzzerReject(BUZZER_INVALID_ARGUMENT);
        return 0;
    }
    Buzzer_Tone_Config_s tone = {.frequency_millihz = (uint32_t)config->frequency * 1000,
                                .duration_ms = 80,
                                .silence_ms = 80,
                                .count = config->count > 6 ? 6 : config->count,
                                .velocity = 127,
                                .voice = kBuzzerVoiceNormal};
    tone.voice.pulse_width_permille = BuzzerLegacyDuty(config->loudness);
    return BuzzerSubmitTone(&tone, NULL) == BUZZER_OK;
}

uint32_t BuzzerMidiToFrequency(uint8_t midi_note) {
    if (midi_note > 127) return 0;
    int32_t octave = (int32_t)(midi_note / 12) - 5;
    uint32_t frequency = kMidiOctave[midi_note % 12];
    if (octave < 0) {
        uint32_t shift = (uint32_t)-octave;
        frequency = (frequency + (1UL << (shift - 1))) >> shift;
    } else {
        frequency <<= (uint32_t)octave;
    }
    return BuzzerFrequencyValid(frequency) ? frequency : 0;
}

BuzzerResult_e BuzzerPlayScore(const Buzzer_Score_s *score, uint16_t repeat_count, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if (__get_IPSR() != 0) return BuzzerReject(BUZZER_WRONG_CONTEXT);
    if (score == NULL || score->notes == NULL || score->note_count == 0 || score->note_count > BUZZER_MAX_SCORE_NOTES ||
        score->bpm < 30 || score->bpm > 300) {
        return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    }
    uint64_t cycle_us = 0;
    for (uint16_t index = 0; index < score->note_count; index++) {
        const Buzzer_Note_s *note = &score->notes[index];
        uint64_t duration = (uint64_t)note->duration_ticks * 60000000 / ((uint32_t)score->bpm * BUZZER_QUARTER_TICKS);
        if (duration < 10000 || (note->frequency_millihz != 0 && !BuzzerFrequencyValid(note->frequency_millihz)) ||
            note->velocity > 127 || (note->voice != NULL && !BuzzerVoiceValid(note->voice))) {
            return BuzzerReject(BUZZER_INVALID_ARGUMENT);
        }
        cycle_us += duration;
    }
    BuzzerRequest_s request = {.kind = REQUEST_SCORE, .cycle_us = cycle_us};
    request.data.music.score = *score;
    request.data.music.repeat_count = repeat_count;
    return BuzzerSubmit(&request, request_id);
}

BuzzerResult_e BuzzerPlayTone(const Buzzer_Tone_Config_s *config, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitTone(config, request_id);
}

BuzzerResult_e BuzzerPlayToneFromISR(const Buzzer_Tone_Config_s *config, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitTone(config, request_id);
}

static BuzzerResult_e BuzzerSubmitPattern(BuzzerPattern_e pattern, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if ((uint32_t)pattern >= BUZZER_PATTERN_COUNT) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    BuzzerRequest_s request = {.kind = REQUEST_PATTERN, .cycle_us = BuzzerPatternDuration(pattern)};
    request.data.pattern = pattern;
    return BuzzerSubmit(&request, request_id);
}

BuzzerResult_e BuzzerNotify(BuzzerPattern_e pattern, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitPattern(pattern, request_id);
}

BuzzerResult_e BuzzerNotifyFromISR(BuzzerPattern_e pattern, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitPattern(pattern, request_id);
}

static BuzzerResult_e BuzzerSubmitDebug(uint16_t source_id, uint8_t code, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if (code > 99) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    BuzzerRequest_s request = {
        .kind = REQUEST_DEBUG,
        .cycle_us = (uint64_t)(BuzzerDigitDuration(code / 10) + BuzzerDigitDuration(code % 10) + 500) * 1000,
    };
    request.data.debug.source_id = source_id;
    request.data.debug.code = code;
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result != BUZZER_OK) {
        rejected_count++;
        BuzzerUnlock(interrupt_mask);
        return result;
    }
    TickType_t submission_tick = __get_IPSR() == 0 ? xTaskGetTickCount() : xTaskGetTickCountFromISR();
    for (uint8_t index = 0; index < kRequestCapacity; index++) {
        if (requests[index].id && requests[index].kind == REQUEST_DEBUG &&
            requests[index].data.debug.source_id == source_id && requests[index].data.debug.code == code) {
            if (request_id != NULL) *request_id = requests[index].id;
            merged_count++;
            BuzzerUnlock(interrupt_mask);
            return BUZZER_MERGED;
        }
    }
    for (uint8_t index = 0; index < kDebugHistoryCapacity; index++) {
        if (debug_history[index].valid && debug_history[index].source_id == source_id &&
            debug_history[index].code == code &&
            (TickType_t)(submission_tick - debug_history[index].completed_tick) < configTICK_RATE_HZ) {
            merged_count++;
            BuzzerUnlock(interrupt_mask);
            return BUZZER_MERGED;
        }
    }
    result = BuzzerSubmit(&request, request_id);
    BuzzerUnlock(interrupt_mask);
    return result;
}

BuzzerResult_e BuzzerDebugCode(uint16_t source_id, uint8_t code, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitDebug(source_id, code, request_id);
}

BuzzerResult_e BuzzerDebugCodeFromISR(uint16_t source_id, uint8_t code, BuzzerRequestId_t *request_id) {
    return BuzzerSubmitDebug(source_id, code, request_id);
}

static BuzzerResult_e BuzzerUpdateIndicator(uint16_t source_id, BuzzerPattern_e pattern, AlarmLevel_e level,
                                           BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if ((uint32_t)pattern >= BUZZER_PATTERN_COUNT || (uint32_t)level >= BUZZER_DEVICE_CNT) {
        return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    }
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    uint8_t selected = BUZZER_SOURCE_CNT;
    for (uint8_t index = 0; index < BUZZER_SOURCE_CNT; index++) {
        if (indicators[index].id == 0 && selected == BUZZER_SOURCE_CNT) selected = index;
        if (indicators[index].id && indicators[index].source_id == source_id) {
            selected = index;
            break;
        }
    }
    if (result == BUZZER_OK && selected == BUZZER_SOURCE_CNT) result = BUZZER_QUEUE_FULL;
    if (result == BUZZER_OK) {
        BuzzerIndicator_s *indicator = &indicators[selected];
        if (indicator->id && indicator->pattern == pattern && indicator->level == level) {
            merged_count++;
            result = BUZZER_MERGED;
        } else {
            *indicator = (BuzzerIndicator_s){.id = BuzzerAllocateId(),
                                            .last_served = served_serial,
                                            .source_id = source_id,
                                            .pattern = pattern,
                                            .level = level};
            change_epoch++;
        }
        if (request_id != NULL) *request_id = indicator->id;
    } else {
        rejected_count++;
    }
    BuzzerUnlock(interrupt_mask);
    return result;
}

BuzzerResult_e BuzzerSetIndicator(uint16_t source_id, BuzzerPattern_e pattern, AlarmLevel_e level,
                                 BuzzerRequestId_t *request_id) {
    return BuzzerUpdateIndicator(source_id, pattern, level, request_id);
}

BuzzerResult_e BuzzerSetIndicatorFromISR(uint16_t source_id, BuzzerPattern_e pattern, AlarmLevel_e level,
                                        BuzzerRequestId_t *request_id) {
    return BuzzerUpdateIndicator(source_id, pattern, level, request_id);
}

BuzzerResult_e BuzzerClearIndicator(uint16_t source_id) {
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result == BUZZER_OK) {
        result = BUZZER_NOT_FOUND;
        for (uint8_t index = 0; index < BUZZER_SOURCE_CNT; index++) {
            if (indicators[index].id && indicators[index].source_id == source_id) {
                indicators[index].id = 0;
                change_epoch++;
                result = BUZZER_OK;
                break;
            }
        }
    }
    if (result != BUZZER_OK) rejected_count++;
    BuzzerUnlock(interrupt_mask);
    return result;
}

BuzzerResult_e BuzzerClearIndicatorFromISR(uint16_t source_id) { return BuzzerClearIndicator(source_id); }

BuzzerResult_e BuzzerCancel(BuzzerRequestId_t request_id) {
    if (request_id == 0) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result == BUZZER_OK) {
        result = BUZZER_NOT_FOUND;
        for (uint8_t index = 0; index < kRequestCapacity; index++) {
            if (requests[index].id == request_id) {
                requests[index].id = 0;
                change_epoch++;
                result = BUZZER_OK;
                break;
            }
        }
    }
    if (result != BUZZER_OK) rejected_count++;
    BuzzerUnlock(interrupt_mask);
    return result;
}

static BuzzerResult_e BuzzerSetControl(uint16_t *control, uint16_t value) {
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result == BUZZER_OK) {
        *control = value;
        change_epoch++;
    } else {
        rejected_count++;
    }
    BuzzerUnlock(interrupt_mask);
    return result;
}

BuzzerResult_e BuzzerSetMusicSpeed(uint16_t speed_permille) {
    if (speed_permille < 250 || speed_permille > 4000) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    return BuzzerSetControl(&music_speed, speed_permille);
}

BuzzerResult_e BuzzerSetStrength(uint16_t strength_permille) {
    if (strength_permille > 1000) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    return BuzzerSetControl(&strength, strength_permille);
}

static BuzzerResult_e BuzzerSetFlag(uint8_t *control, uint8_t value) {
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result == BUZZER_OK) {
        *control = value;
        change_epoch++;
    } else {
        rejected_count++;
    }
    BuzzerUnlock(interrupt_mask);
    return result;
}

BuzzerResult_e BuzzerPauseMusic(void) { return BuzzerSetFlag(&music_paused, 1); }
BuzzerResult_e BuzzerResumeMusic(void) { return BuzzerSetFlag(&music_paused, 0); }
BuzzerResult_e BuzzerSetMuted(uint8_t value) {
    if (value > 1) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    return BuzzerSetFlag(&muted, value);
}

BuzzerResult_e BuzzerStop(void) {
    uint32_t interrupt_mask = BuzzerLock();
    BuzzerResult_e result = BuzzerReadiness();
    if (result == BUZZER_OK) {
        for (uint8_t index = 0; index < kRequestCapacity; index++) requests[index].id = 0;
        change_epoch++;
    } else {
        rejected_count++;
    }
    BuzzerUnlock(interrupt_mask);
    return result;
}

static BuzzerPriority_e BuzzerOwnerPriority(BuzzerOwner_s owner) {
    if (owner.kind == OWNER_REQUEST) return BuzzerRequestPriority(&requests[owner.index]);
    if (owner.kind == OWNER_INDICATOR) return (BuzzerPriority_e)indicators[owner.index].level;
    if (owner.kind == OWNER_LEGACY) return (BuzzerPriority_e)owner.index;
    return BUZZER_PRIORITY_IDLE;
}

BuzzerResult_e BuzzerGetStatus(Buzzer_Status_s *status) {
    if (status == NULL) return BuzzerReject(BUZZER_INVALID_ARGUMENT);
    uint32_t interrupt_mask = BuzzerLock();
    *status = (Buzzer_Status_s){.initialized = initialized,
                               .panic_latched = panic_latched,
                               .music_paused = music_paused,
                               .muted = muted,
                               .strength_permille = strength,
                               .music_speed_permille = music_speed,
                               .current_priority = BUZZER_PRIORITY_IDLE,
                               .output_frequency_millihz = output_frequency,
                               .output_duty_permille = output_duty,
                               .rejected_count = rejected_count,
                               .merged_count = merged_count,
                               .completed_count = completed_count};
    uint8_t has_foreground = 0;
    for (uint8_t index = 0; index < kRequestCapacity; index++) {
        if (requests[index].id) {
            status->request_count++;
            has_foreground |= requests[index].started;
        }
    }
    status->waiting_count = status->request_count - (has_foreground ? 1 : 0);
    for (uint8_t index = 0; index < BUZZER_SOURCE_CNT; index++) status->indicator_count += indicators[index].id != 0;
    if (BuzzerOwnerValid(current_owner)) {
        status->current_request = current_owner.kind == OWNER_LEGACY ? 0 : current_owner.id;
        status->current_priority = BuzzerOwnerPriority(current_owner);
    }
    if (status->panic_latched) {
        status->current_request = 0;
        status->current_priority = BUZZER_PRIORITY_PANIC;
        status->output_frequency_millihz = 2000000;
        status->output_duty_permille = 500;
    }
    BuzzerUnlock(interrupt_mask);
    return BUZZER_OK;
}

BuzzerResult_e BuzzerPanicTone(void) {
    if (!initialized) return BUZZER_NOT_READY;
    panic_latched = 1;
    __DMB();
    return PWMLatchOutput(pwm_instance, &panic_output) == PWM_OK ? BUZZER_OK : BUZZER_NOT_READY;
}

static void BuzzerSnapshot(BuzzerOwner_s owner) {
    service_snapshot.owner = owner;
    if (owner.kind == OWNER_REQUEST) service_snapshot.request = requests[owner.index];
    if (owner.kind == OWNER_INDICATOR) service_snapshot.indicator = indicators[owner.index];
    if (owner.kind == OWNER_LEGACY) {
        service_snapshot.legacy = legacy_instances[owner.index];
        service_snapshot.legacy_progress_us = legacy_progress[owner.index];
    }
}

static void BuzzerRememberDebug(const BuzzerRequest_s *request, TickType_t now) {
    uint8_t selected = 0;
    uint32_t oldest_age = 0;
    for (uint8_t index = 0; index < kDebugHistoryCapacity; index++) {
        if (!debug_history[index].valid ||
            (TickType_t)(now - debug_history[index].completed_tick) >= configTICK_RATE_HZ) {
            selected = index;
            break;
        }
        uint32_t age = (TickType_t)(now - debug_history[index].completed_tick);
        if (age >= oldest_age) {
            oldest_age = age;
            selected = index;
        }
    }
    debug_history[selected] = (BuzzerDebugHistory_s){.completed_tick = now,
                                                   .source_id = request->data.debug.source_id,
                                                   .code = request->data.debug.code,
                                                   .valid = 1};
}

static uint16_t BuzzerLocateNote(const BuzzerRequest_s *request, uint64_t progress, uint64_t *position,
                               uint64_t *duration) {
    const Buzzer_Score_s *score = &request->data.music.score;
    *position = progress % request->cycle_us;
    *duration = 0;
    for (uint16_t index = 0; index < score->note_count; index++) {
        *duration = (uint64_t)score->notes[index].duration_ticks * 60000000 /
                    ((uint32_t)score->bpm * BUZZER_QUARTER_TICKS);
        if (*position < *duration) return index;
        *position -= *duration;
    }
    return 0;
}

static uint8_t BuzzerAdvance(uint32_t elapsed_ms, uint16_t speed, uint8_t paused, TickType_t now) {
    BuzzerOwner_s owner = service_snapshot.owner;
    uint64_t progress;
    uint64_t voice_us = 0;
    uint8_t completed = 0;
    if (owner.kind == OWNER_REQUEST) {
        BuzzerRequest_s *request = &service_snapshot.request;
        uint64_t real_elapsed_us = (uint64_t)elapsed_ms * 1000;
        uint64_t elapsed_us = real_elapsed_us;
        if (request->kind == REQUEST_SCORE) {
            real_elapsed_us = paused ? 0 : real_elapsed_us;
            elapsed_us = paused ? 0 : (uint64_t)elapsed_ms * speed;
        }
        progress = request->progress_us + elapsed_us;
        uint64_t duration = request->cycle_us;
        if (request->kind == REQUEST_SCORE) {
            if (request->data.music.repeat_count == 0) {
                if (progress >= duration) progress = progress % duration + duration;
                duration = UINT64_MAX;
            } else {
                duration *= request->data.music.repeat_count;
            }
        }
        completed = progress >= duration;
        if (request->kind == REQUEST_SCORE && !completed) {
            uint64_t position;
            uint64_t note_duration;
            BuzzerLocateNote(request, progress, &position, &note_duration);
            voice_us = elapsed_us <= position ? request->voice_us + real_elapsed_us : position * 1000 / speed;
        }
    } else if (owner.kind == OWNER_INDICATOR) {
        progress = service_snapshot.indicator.progress_us + (uint64_t)elapsed_ms * 1000;
        completed = progress >= BuzzerPatternDuration(service_snapshot.indicator.pattern);
    } else {
        progress = service_snapshot.legacy_progress_us + (uint64_t)elapsed_ms * 1000;
        completed = progress >= 1000000;
    }

    uint32_t interrupt_mask = BuzzerLock();
    if (BuzzerOwnerValid(owner)) {
        if (owner.kind == OWNER_REQUEST) {
            if (completed) {
                if (service_snapshot.request.kind == REQUEST_DEBUG) BuzzerRememberDebug(&service_snapshot.request, now);
                requests[owner.index].id = 0;
                completed_count++;
            } else {
                requests[owner.index].progress_us = progress;
                requests[owner.index].voice_us = voice_us;
            }
        } else if (owner.kind == OWNER_INDICATOR) {
            indicators[owner.index].progress_us = completed ? 0 : progress;
            if (completed) indicators[owner.index].last_served = ++served_serial;
        } else {
            legacy_progress[owner.index] = completed ? 0 : progress;
            if (completed) legacy_served[owner.index] = ++served_serial;
        }
    }
    BuzzerUnlock(interrupt_mask);
    return completed;
}

static BuzzerOwner_s BuzzerSelectOwner(uint8_t completed) {
    BuzzerOwner_s selected = {0};
    BuzzerPriority_e priority = BUZZER_PRIORITY_IDLE;
    uint32_t last_served = UINT32_MAX;
    for (uint8_t index = 0; index < kRequestCapacity; index++) {
        if (requests[index].id == 0 || (music_paused && requests[index].kind == REQUEST_SCORE)) continue;
        BuzzerPriority_e candidate_priority = BuzzerRequestPriority(&requests[index]);
        if (candidate_priority < priority ||
            (candidate_priority == priority && (int32_t)(requests[index].id - selected.id) < 0)) {
            selected = (BuzzerOwner_s){OWNER_REQUEST, index, requests[index].id};
            priority = candidate_priority;
        }
    }
    for (uint8_t index = 0; index < BUZZER_SOURCE_CNT; index++) {
        if (indicators[index].id == 0) continue;
        BuzzerPriority_e candidate_priority = (BuzzerPriority_e)indicators[index].level;
        if (candidate_priority < priority ||
            (candidate_priority == priority &&
             (uint32_t)(served_serial - indicators[index].last_served) > (uint32_t)(served_serial - last_served))) {
            selected = (BuzzerOwner_s){OWNER_INDICATOR, index, indicators[index].id};
            priority = candidate_priority;
            last_served = indicators[index].last_served;
        }
    }
    for (uint8_t index = 0; index < BUZZER_DEVICE_CNT; index++) {
        if (!legacy_registered[index] || legacy_instances[index].alarm_state != ALARM_ON) continue;
        if (index < (uint32_t)priority ||
            (index == (uint32_t)priority &&
             (uint32_t)(served_serial - legacy_served[index]) > (uint32_t)(served_serial - last_served))) {
            selected = (BuzzerOwner_s){OWNER_LEGACY, index, legacy_revision[index]};
            priority = (BuzzerPriority_e)index;
            last_served = legacy_served[index];
        }
    }
    if (!completed && BuzzerOwnerValid(current_owner) && BuzzerOwnerPriority(current_owner) == priority &&
        !(current_owner.kind == OWNER_REQUEST && music_paused && requests[current_owner.index].kind == REQUEST_SCORE)) {
        selected = current_owner;
    }
    return selected;
}

static void BuzzerRestartInterrupted(BuzzerOwner_s owner) {
    if (!BuzzerOwnerValid(owner)) return;
    if (owner.kind == OWNER_REQUEST && requests[owner.index].kind != REQUEST_SCORE) {
        requests[owner.index].progress_us = 0;
    }
    if (owner.kind == OWNER_INDICATOR) indicators[owner.index].progress_us = 0;
    if (owner.kind == OWNER_LEGACY) legacy_progress[owner.index] = 0;
}

static int32_t BuzzerOscillator(uint32_t elapsed_ms, uint8_t frequency_hz) {
    uint32_t phase = (uint32_t)(((uint64_t)elapsed_ms * frequency_hz * 32 / 1000) % 32);
    return kSine[phase];
}

static void BuzzerApplyVoice(uint32_t frequency, uint32_t previous_frequency, uint32_t position_ms,
                            uint32_t duration_ms, uint8_t velocity, const Buzzer_Voice_Config_s *voice,
                            uint32_t *output_hz, uint16_t *duty) {
    *output_hz = frequency;
    *duty = 0;
    if (frequency == 0 || position_ms >= duration_ms) return;
    uint32_t gain = 1000;
    if (voice->attack_ms && position_ms < voice->attack_ms) {
        gain = position_ms * 1000 / voice->attack_ms;
    } else if (voice->decay_ms && position_ms < (uint32_t)voice->attack_ms + voice->decay_ms) {
        gain = 1000 - (position_ms - voice->attack_ms) * (1000 - voice->sustain_permille) / voice->decay_ms;
    } else {
        gain = voice->sustain_permille;
    }
    if (voice->release_ms && duration_ms - position_ms < voice->release_ms) {
        gain = gain * (duration_ms - position_ms) / voice->release_ms;
    }
    if (voice->glide_ms && previous_frequency && position_ms < voice->glide_ms) {
        int64_t difference = (int64_t)frequency - previous_frequency;
        frequency = (uint32_t)((int64_t)previous_frequency + difference * position_ms / voice->glide_ms);
    }
    if (voice->vibrato_hz) {
        int32_t oscillation = BuzzerOscillator(position_ms, voice->vibrato_hz);
        frequency = (uint32_t)((int64_t)frequency +
                              (int64_t)frequency * oscillation * voice->vibrato_depth_permille / 1000000);
    }
    if (frequency < BUZZER_MIN_FREQUENCY_MILLIHZ) frequency = BUZZER_MIN_FREQUENCY_MILLIHZ;
    if (frequency > BUZZER_MAX_FREQUENCY_MILLIHZ) frequency = BUZZER_MAX_FREQUENCY_MILLIHZ;
    if (voice->tremolo_hz) {
        int32_t oscillation = BuzzerOscillator(position_ms, voice->tremolo_hz);
        uint32_t modulation = 1000 - (uint32_t)(1000 - oscillation) * voice->tremolo_depth_permille / 2000;
        gain = gain * modulation / 1000;
    }
    *output_hz = frequency;
    *duty = (uint16_t)((uint64_t)voice->pulse_width_permille * gain * velocity / 127000);
}

static void BuzzerRenderPattern(BuzzerPattern_e pattern, uint64_t progress, uint32_t *frequency, uint16_t *duty) {
    const BuzzerPattern_s *config = &kPatterns[pattern];
    for (uint8_t index = 0; index < config->count; index++) {
        uint64_t duration_us = (uint64_t)config->segments[index].duration_ms * 1000;
        if (progress < duration_us) {
            *frequency = config->segments[index].frequency_millihz;
            *duty = *frequency ? 500 : 0;
            return;
        }
        progress -= duration_us;
    }
}

static void BuzzerRenderDigit(uint8_t digit, uint32_t position_ms, uint32_t tone_frequency, uint32_t *frequency,
                              uint16_t *duty) {
    uint32_t on_ms = digit == 0 ? 300 : 80;
    uint32_t phase_ms = position_ms % (on_ms + 80);
    if (phase_ms < on_ms) {
        *frequency = tone_frequency;
        *duty = 500;
    }
}

static void BuzzerRenderScore(const BuzzerRequest_s *request, uint16_t speed, uint32_t *frequency, uint16_t *duty) {
    const Buzzer_Score_s *score = &request->data.music.score;
    uint64_t position;
    uint64_t duration;
    uint16_t index = BuzzerLocateNote(request, request->progress_us, &position, &duration);
    uint32_t previous_frequency = 0;
    if (index != 0) {
        previous_frequency = score->notes[index - 1].frequency_millihz;
    } else if (request->progress_us >= request->cycle_us) {
        previous_frequency = score->notes[score->note_count - 1].frequency_millihz;
    }
    const Buzzer_Note_s *note = &score->notes[index];
    uint32_t position_ms = (uint32_t)(request->voice_us / 1000);
    uint32_t duration_ms = position_ms + (uint32_t)((duration - position) / speed);
    BuzzerApplyVoice(note->frequency_millihz, previous_frequency, position_ms, duration_ms, note->velocity,
                     note->voice == NULL ? &kBuzzerVoiceNormal : note->voice, frequency, duty);
}

static void BuzzerRender(uint16_t speed, uint32_t *frequency, uint16_t *duty) {
    *frequency = 0;
    *duty = 0;
    BuzzerOwner_s owner = service_snapshot.owner;
    if (owner.kind == OWNER_NONE) return;
    if (owner.kind == OWNER_LEGACY) {
        const BuzzzerInstance *legacy = &service_snapshot.legacy;
        if ((uint32_t)legacy->octave > OCTAVE_8 || !BuzzerFloatValid(&legacy->loudness) ||
            (uint32_t)legacy->alarm_level >= BUZZER_DEVICE_CNT) return;
        *frequency = kOctaveFrequencies[legacy->octave];
        *duty = BuzzerLegacyDuty(legacy->loudness);
        return;
    }
    if (owner.kind == OWNER_INDICATOR) {
        BuzzerRenderPattern(service_snapshot.indicator.pattern, service_snapshot.indicator.progress_us,
                            frequency, duty);
        return;
    }
    const BuzzerRequest_s *request = &service_snapshot.request;
    if (request->kind == REQUEST_SCORE) {
        BuzzerRenderScore(request, speed, frequency, duty);
    } else if (request->kind == REQUEST_PATTERN) {
        BuzzerRenderPattern(request->data.pattern, request->progress_us, frequency, duty);
    } else if (request->kind == REQUEST_TONE) {
        const Buzzer_Tone_Config_s *tone = &request->data.tone;
        uint32_t position_ms = (uint32_t)(request->progress_us / 1000) % (tone->duration_ms + tone->silence_ms);
        BuzzerApplyVoice(tone->frequency_millihz, 0, position_ms, tone->duration_ms, tone->velocity, &tone->voice,
                         frequency, duty);
    } else {
        uint32_t position_ms = (uint32_t)(request->progress_us / 1000);
        uint32_t first_duration = BuzzerDigitDuration(request->data.debug.code / 10);
        if (position_ms < first_duration) {
            BuzzerRenderDigit(request->data.debug.code / 10, position_ms, 523000, frequency, duty);
        } else if (position_ms >= first_duration + 300) {
            uint32_t second_position = position_ms - first_duration - 300;
            if (second_position < BuzzerDigitDuration(request->data.debug.code % 10)) {
                BuzzerRenderDigit(request->data.debug.code % 10, second_position, 1046502, frequency, duty);
            }
        }
    }
}

void BuzzerTask(void) {
    if (__get_IPSR() != 0) return;
    uint32_t interrupt_mask = BuzzerLock();
    if (!initialized || service_busy || panic_latched) {
        BuzzerUnlock(interrupt_mask);
        return;
    }
    service_busy = 1;
    uint16_t speed = music_speed;
    uint8_t paused = music_paused;
    uint8_t previous_valid = BuzzerOwnerValid(current_owner);
    if (previous_valid) BuzzerSnapshot(current_owner);
    BuzzerUnlock(interrupt_mask);

    TickType_t now = xTaskGetTickCount();
    uint64_t elapsed_numerator = (uint64_t)(TickType_t)(now - last_tick) * 1000 + tick_remainder;
    uint32_t elapsed_ms = (uint32_t)(elapsed_numerator / configTICK_RATE_HZ);
    tick_remainder = (uint32_t)(elapsed_numerator % configTICK_RATE_HZ);
    last_tick = now;
    uint8_t completed = previous_valid ? BuzzerAdvance(elapsed_ms, speed, paused, now) : 0;

    interrupt_mask = BuzzerLock();
    BuzzerOwner_s selected = BuzzerSelectOwner(completed);
    if (!BuzzerSameOwner(selected, current_owner)) BuzzerRestartInterrupted(current_owner);
    current_owner = selected;
    if (selected.kind == OWNER_REQUEST) requests[selected.index].started = 1;
    BuzzerSnapshot(selected);
    uint32_t epoch = change_epoch;
    uint16_t local_strength = strength;
    uint8_t local_muted = muted;
    BuzzerUnlock(interrupt_mask);

    uint32_t frequency;
    uint16_t duty;
    BuzzerRender(speed, &frequency, &duty);
    if (selected.kind == OWNER_REQUEST) duty = local_muted ? 0 : (uint16_t)((uint32_t)duty * local_strength / 1000);
    PWM_Output_s output;
    PWMResult_e prepared = PWMPrepareOutput(pwm_instance, frequency, duty, &output);
    interrupt_mask = BuzzerLock();
    if (prepared == PWM_OK && change_epoch == epoch && !panic_latched) {
        if (PWMApplyOutput(pwm_instance, &output) == PWM_OK) {
            output_frequency = frequency;
            output_duty = duty;
        }
    }
    service_busy = 0;
    BuzzerUnlock(interrupt_mask);
}
