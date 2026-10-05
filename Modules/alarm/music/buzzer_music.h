#pragma once

#include "buzzer.h"

/**
 * @brief 组曲：按顺序连续播放的多个曲谱段。
 *
 * 每段就是一个标准 Buzzer_Score_s（最多 BUZZER_MAX_SCORE_NOTES 个音符），
 * 由 tools/midi_to_score.py 生成的音乐文件提供。分段只是为了满足单曲谱
 * 音符上限，段与段之间按 FIFO 连续播放，听感上是一整首曲子。
 */
typedef struct {
    const Buzzer_Score_s *segments;
    uint16_t segment_count;
} Buzzer_MusicSuite_s;

/**
 * @brief 开始顺序播放整套组曲，每段播放一次。仅允许任务上下文调用。
 *
 * 分段作为同优先级（BUZZER_PRIORITY_MUSIC）一次性请求依次提交，播放顺序 FIFO；
 * 高优先级报警/提示音可以抢占，抢占结束后继续播放。蜂鸣器请求池只有 1 个当前
 * 请求加 BUZZER_WAITING_CNT 个等待请求，一次最多装下 BUZZER_WAITING_CNT + 1 段，
 * 因此本次只提交装得下的开头若干段，其余分段由 BuzzerMusicSuiteService() 在
 * 播放过程中补队——**长组曲必须周期调用该函数**（例如守护任务里每 10~100 ms
 * 一次），否则播到已提交的段就会停止。
 *
 * 重复调用本函数会先取消上一次尚未播完的组曲（最新请求优先）。
 *
 * @param suite 组曲描述；分段曲谱描述在提交时复制，但其音符数组和音色必须在
 *              播放全部结束或取消前保持有效（生成的音乐文件均为 static const）。
 * @param request_id 可为 NULL；非 NULL 时返回首段请求 ID。
 * @return BUZZER_OK 至少首段已接收（后续段由 Service 补队）；其余返回值与
 *         BuzzerPlayScore() 一致，首段提交失败时不会开始播放。
 */
BuzzerResult_e BuzzerMusicSuitePlay(const Buzzer_MusicSuite_s *suite, BuzzerRequestId_t *request_id);

/**
 * @brief 补队尚未提交的分段。周期调用（建议 10~100 ms 一次），可安全随时调用。
 *
 * 只在请求池有空位时提交，池满时静默等待下一次调用；不播放、不改播放进度。
 *
 * @return BUZZER_OK 已处理（含无事可做）；BUZZER_NOT_FOUND 当前没有进行中的组曲。
 */
BuzzerResult_e BuzzerMusicSuiteService(void);

/**
 * @brief 组曲是否仍在播放（含已排队未播、待补队）。
 *
 * 判定偏保守：末段播完且请求池清空后才返回 0；只统计一次性请求/音乐，不含持续报警。
 */
uint8_t BuzzerMusicSuitePlaying(void);

/**
 * @brief 停止本模块提交的整套组曲：撤销仍在队列/播放中的分段并停止补队。
 *
 * 只作用于 BuzzerMusicSuitePlay() 跟踪的请求，不影响其他蜂鸣器请求和持续报警；
 * 播放进度不保留，重新播放从第一段开始。
 *
 * @return BUZZER_OK 已撤销；BUZZER_NOT_FOUND 无进行中的组曲。
 */
BuzzerResult_e BuzzerMusicSuiteStop(void);
