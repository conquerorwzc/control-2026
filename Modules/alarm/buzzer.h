#pragma once

#include <stdint.h>

#define BUZZER_DEVICE_CNT 5
#define BUZZER_WAITING_CNT 16
#define BUZZER_SOURCE_CNT 16
#define BUZZER_MAX_SCORE_NOTES 128
#define BUZZER_QUARTER_TICKS 96
#define BUZZER_MIN_FREQUENCY_MILLIHZ 20000UL
#define BUZZER_MAX_FREQUENCY_MILLIHZ 8000000UL

#define DoFreq 523
#define ReFreq 587
#define MiFreq 659
#define FaFreq 698
#define SoFreq 784
#define LaFreq 880
#define SiFreq 988

typedef enum {
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
} octave_e;

typedef enum {
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
} AlarmLevel_e;

typedef enum { ALARM_OFF = 0, ALARM_ON } AlarmState_e;

typedef struct {
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
} Buzzer_config_s;

typedef struct {
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
} BuzzzerInstance;

typedef struct {
    uint16_t frequency;
    uint8_t count;
    float loudness;
} Buzzer_Beep_Config_s;

typedef uint32_t BuzzerRequestId_t;

typedef enum {
    BUZZER_OK = 0,
    BUZZER_MERGED,
    BUZZER_INVALID_ARGUMENT,
    BUZZER_NOT_READY,
    BUZZER_QUEUE_FULL,
    BUZZER_NOT_FOUND,
    BUZZER_WRONG_CONTEXT,
    BUZZER_PANIC_LATCHED,
} BuzzerResult_e;

typedef enum {
    BUZZER_PRIORITY_HIGH = 0,
    BUZZER_PRIORITY_ABOVE_MEDIUM,
    BUZZER_PRIORITY_MEDIUM,
    BUZZER_PRIORITY_BELOW_MEDIUM,
    BUZZER_PRIORITY_LOW,
    BUZZER_PRIORITY_NOTIFY,
    BUZZER_PRIORITY_DEBUG,
    BUZZER_PRIORITY_MUSIC,
    BUZZER_PRIORITY_IDLE,
    BUZZER_PRIORITY_PANIC,
} BuzzerPriority_e;

typedef enum {
    BUZZER_PATTERN_READY = 0,
    BUZZER_PATTERN_SUCCESS,
    BUZZER_PATTERN_WARNING,
    BUZZER_PATTERN_ERROR,
    BUZZER_PATTERN_COMM_LOST,
    BUZZER_PATTERN_COUNT,
} BuzzerPattern_e;

typedef struct {
    uint16_t pulse_width_permille;
    uint16_t attack_ms;
    uint16_t decay_ms;
    uint16_t sustain_permille;
    uint16_t release_ms;
    uint16_t glide_ms;
    uint16_t vibrato_depth_permille;
    uint16_t tremolo_depth_permille;
    uint8_t vibrato_hz;
    uint8_t tremolo_hz;
} Buzzer_Voice_Config_s;

typedef struct {
    uint32_t frequency_millihz;
    uint16_t duration_ticks;
    uint8_t velocity;
    const Buzzer_Voice_Config_s *voice;
} Buzzer_Note_s;

typedef struct {
    const Buzzer_Note_s *notes;
    uint16_t note_count;
    uint16_t bpm;
} Buzzer_Score_s;

typedef struct {
    uint32_t frequency_millihz;
    uint16_t duration_ms;
    uint16_t silence_ms;
    uint8_t count;
    uint8_t velocity;
    Buzzer_Voice_Config_s voice;
} Buzzer_Tone_Config_s;

typedef struct {
    uint8_t initialized;
    uint8_t panic_latched;
    uint8_t music_paused;
    uint8_t muted;
    uint8_t waiting_count;
    uint8_t request_count;
    uint8_t indicator_count;
    uint16_t strength_permille;
    uint16_t music_speed_permille;
    BuzzerRequestId_t current_request;
    BuzzerPriority_e current_priority;
    uint32_t output_frequency_millihz;
    uint16_t output_duty_permille;
    uint32_t rejected_count;
    uint32_t merged_count;
    uint32_t completed_count;
} Buzzer_Status_s;

extern const Buzzer_Voice_Config_s kBuzzerVoiceNormal;
extern const Buzzer_Voice_Config_s kBuzzerVoiceSoft;
extern const Buzzer_Voice_Config_s kBuzzerVoiceBright;
extern const Buzzer_Voice_Config_s kBuzzerVoiceSiren;

void BuzzerInit(void);
void BuzzerTask(void);
uint8_t BuzzerBeep(uint8_t count);
uint8_t BuzzerBeepWithConfig(const Buzzer_Beep_Config_s *config);
uint8_t BuzzerBeepWithFreq(uint16_t frequency, uint8_t count);
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);

uint32_t BuzzerMidiToFrequency(uint8_t midi_note);
BuzzerResult_e BuzzerPlayScore(const Buzzer_Score_s *score, uint16_t repeat_count, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerPlayTone(const Buzzer_Tone_Config_s *config, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerPlayToneFromISR(const Buzzer_Tone_Config_s *config, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerNotify(BuzzerPattern_e pattern, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerNotifyFromISR(BuzzerPattern_e pattern, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerDebugCode(uint16_t source_id, uint8_t code, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerDebugCodeFromISR(uint16_t source_id, uint8_t code, BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerSetIndicator(uint16_t source_id, BuzzerPattern_e pattern, AlarmLevel_e level,
                                 BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerSetIndicatorFromISR(uint16_t source_id, BuzzerPattern_e pattern, AlarmLevel_e level,
                                        BuzzerRequestId_t *request_id);
BuzzerResult_e BuzzerClearIndicator(uint16_t source_id);
BuzzerResult_e BuzzerClearIndicatorFromISR(uint16_t source_id);
BuzzerResult_e BuzzerCancel(BuzzerRequestId_t request_id);
BuzzerResult_e BuzzerPauseMusic(void);
BuzzerResult_e BuzzerResumeMusic(void);
BuzzerResult_e BuzzerSetMusicSpeed(uint16_t speed_permille);
BuzzerResult_e BuzzerSetStrength(uint16_t strength_permille);
BuzzerResult_e BuzzerSetMuted(uint8_t muted);
BuzzerResult_e BuzzerStop(void);
BuzzerResult_e BuzzerGetStatus(Buzzer_Status_s *status);
BuzzerResult_e BuzzerPanicTone(void);
