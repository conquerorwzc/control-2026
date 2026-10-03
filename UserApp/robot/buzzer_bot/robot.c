#include "robot.h"

#include "bsp_dwt.h"
#include "rinascita_suite.h"
#include "robot_config.h"

static float autoplay_mark_ms;
static float service_mark_ms;
static uint8_t autoplay_submitted;

void RobotInit() {
    autoplay_mark_ms = DWT_GetTimeline_ms();
    service_mark_ms = autoplay_mark_ms;
}

void RobotTask() {
    float now_ms = DWT_GetTimeline_ms();

    /* 上电延迟 BUZZER_BOT_AUTOPLAY_DELAY_MS 后自动播放组曲。蜂鸣器由守护任务
     * BuzzerInit() 初始化，尚未就绪时提交返回 BUZZER_NOT_READY，这里按周期重试。 */
    if (!autoplay_submitted && now_ms - autoplay_mark_ms >= BUZZER_BOT_AUTOPLAY_DELAY_MS) {
        autoplay_submitted = BuzzerMusicSuitePlay(&kRinascitaSuite, NULL) == BUZZER_OK;
    }

    /* 长组曲按 BUZZER_BOT_SERVICE_PERIOD_MS 周期补队后续分段 */
    if (now_ms - service_mark_ms >= BUZZER_BOT_SERVICE_PERIOD_MS) {
        service_mark_ms = now_ms;
        BuzzerMusicSuiteService();
    }
}
