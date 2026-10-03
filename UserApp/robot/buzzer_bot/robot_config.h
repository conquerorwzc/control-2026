/**
******************************************************************************
* @file    robot_config.h
* @brief   buzzer_bot 配置：蜂鸣器音乐演示机器人
******************************************************************************
*/
#pragma once

#define ONE_BOARD  // 单板控制整车

/* 上电后延迟多久自动开始播放组曲（毫秒） */
#define BUZZER_BOT_AUTOPLAY_DELAY_MS 1000.0f
/* 组曲分段补队周期（毫秒），即 BuzzerMusicSuiteService() 的调用间隔 */
#define BUZZER_BOT_SERVICE_PERIOD_MS 20.0f
