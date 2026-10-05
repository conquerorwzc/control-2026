/**
******************************************************************************
* @file    robot_config.h
* @brief   buzzer_bot 配置：蜂鸣器音效演示 + 组曲播放机器人
******************************************************************************
*/
#pragma once

#define ONE_BOARD  // 单板控制整车

/* 上电后延迟多久开始演示（毫秒），留出守护任务 BuzzerInit() 的时间 */
#define BUZZER_BOT_START_DELAY_MS 1000.0f

/* 报警层级演示每一步的时长（毫秒）；需大于该级音效时长（最长错误音 1350 ms） */
#define BUZZER_BOT_LEVEL_STEP_MS 1450.0f

/* 组曲分段补队周期（毫秒），即 BuzzerMusicSuiteService() 的调用间隔 */
#define BUZZER_BOT_SERVICE_PERIOD_MS 20.0f

/* 报警层级演示占用的持续来源 ID 基址（5 个，避开其他模块的 source_id） */
#define BUZZER_BOT_LEVEL_SOURCE_BASE 0x100u

/* 演示播放的组曲；另一个可选 kWuciYouciSuite（《无刺有刺》） */
//#define BUZZER_BOT_SUITE kWuciYouciSuite
#define BUZZER_BOT_SUITE kRinascitaSuite

