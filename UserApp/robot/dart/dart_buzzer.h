/**
 * @file dart_buzzer.h
 * @brief dart 发射架蜂鸣器提示: 各操作环节用不同声音反馈, 故障/失联用持续指示
 *
 * 声音设计(全部非阻塞, 由 BuzzerTask 推进):
 *
 * | 环节           | 提示                         |
 * |----------------|------------------------------|
 * | 使能成功       | 单短高滴                     |
 * | 失能/急停      | 低双鸣                       |
 * | 进入调试档     | 短高滴 x2                    |
 * | 校准开始       | 渐高三连滴 (Do-Mi-So)        |
 * | 校准完成       | 上行双音 (Mi-So)             |
 * | 储能开始       | 单中鸣                       |
 * | 储能完成 READY | 上行双音 (So-Do') + SUCCESS  |
 * | 发射瞬间       | 极短高音脉冲                 |
 * | 发射序列结束   | 短滴                         |
 * | 命令被忽略     | WARNING 双短                 |
 * | 故障 FAULT     | ERROR 一次性 + 持续 ERROR 指示|
 * | 遥控器失联     | COMM_LOST 持续指示           |
 */
#pragma once

/* 一次性提示 */
void DartBuzzerEnableOk(void);
void DartBuzzerDisable(void);
void DartBuzzerDebugMode(void);
void DartBuzzerCaliStart(void);
void DartBuzzerCaliDone(void);
void DartBuzzerChargeStart(void);
void DartBuzzerChargeDone(void);
void DartBuzzerFire(void);
void DartBuzzerFireDone(void);
void DartBuzzerCmdRejected(void);
void DartBuzzerFault(void);
void DartBuzzerFaultCleared(void);

/* 持续指示 */
void DartBuzzerRcLost(void);
void DartBuzzerRcOk(void);
