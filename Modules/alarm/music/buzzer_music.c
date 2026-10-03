#include "buzzer_music.h"

#include <stddef.h>

enum { kSuiteRequestCapacity = BUZZER_WAITING_CNT + 1 };

static const Buzzer_MusicSuite_s *suite_state;
static uint16_t suite_next_segment;
static BuzzerRequestId_t suite_recent_ids[kSuiteRequestCapacity];
static uint8_t suite_recent_head;
static uint8_t suite_recent_count;
static uint8_t suite_active;

static void BuzzerMusicSuiteTrackRequest(BuzzerRequestId_t request_id) {
    suite_recent_ids[suite_recent_head] = request_id;
    suite_recent_head = (uint8_t)((suite_recent_head + 1) % kSuiteRequestCapacity);
    if (suite_recent_count < kSuiteRequestCapacity) suite_recent_count++;
}

static void BuzzerMusicSuiteCancelAll(void) {
    /* 组曲分段同优先级 FIFO，最多只有请求池容量个仍在队列中，且都是最近提交的
     * 那几个；对更早的请求 ID 取消会得到 BUZZER_NOT_FOUND，不影响其它请求。 */
    for (uint8_t index = 0; index < suite_recent_count; index++) {
        BuzzerCancel(suite_recent_ids[index]);
        suite_recent_ids[index] = 0;
    }
    suite_recent_count = 0;
    suite_recent_head = 0;
    suite_next_segment = 0;
    suite_active = 0;
    suite_state = NULL;
}

static void BuzzerMusicSuiteSubmitPending(void) {
    while (suite_active && suite_next_segment < suite_state->segment_count) {
        BuzzerRequestId_t request_id = 0;
        if (BuzzerPlayScore(&suite_state->segments[suite_next_segment], 1, &request_id) != BUZZER_OK) break;
        BuzzerMusicSuiteTrackRequest(request_id);
        suite_next_segment++;
    }
}

BuzzerResult_e BuzzerMusicSuitePlay(const Buzzer_MusicSuite_s *suite, BuzzerRequestId_t *request_id) {
    if (request_id != NULL) *request_id = 0;
    if (suite == NULL || suite->segments == NULL || suite->segment_count == 0) return BUZZER_INVALID_ARGUMENT;

    BuzzerMusicSuiteCancelAll();
    suite_state = suite;
    suite_active = 1;

    BuzzerRequestId_t first_id = 0;
    BuzzerResult_e result = BuzzerPlayScore(&suite_state->segments[0], 1, &first_id);
    if (result != BUZZER_OK) {
        BuzzerMusicSuiteCancelAll();
        return result;
    }
    BuzzerMusicSuiteTrackRequest(first_id);
    suite_next_segment = 1;
    if (request_id != NULL) *request_id = first_id;

    BuzzerMusicSuiteSubmitPending();
    return BUZZER_OK;
}

BuzzerResult_e BuzzerMusicSuiteService(void) {
    if (!suite_active) return BUZZER_NOT_FOUND;
    BuzzerMusicSuiteSubmitPending();
    return BUZZER_OK;
}

uint8_t BuzzerMusicSuitePlaying(void) {
    if (!suite_active) return 0;
    if (suite_next_segment < suite_state->segment_count) return 1;
    Buzzer_Status_s status;
    if (BuzzerGetStatus(&status) != BUZZER_OK) return 1;
    if (status.request_count != 0) return 1;
    BuzzerMusicSuiteCancelAll();
    return 0;
}

BuzzerResult_e BuzzerMusicSuiteStop(void) {
    if (!suite_active) return BUZZER_NOT_FOUND;
    BuzzerMusicSuiteCancelAll();
    return BUZZER_OK;
}
