#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""midi_to_score.py — 把 MIDI 主旋律转换为蜂鸣器模块的常量曲谱 C 文件。

本脚本只依赖 Python3 标准库，输出符合 Modules/alarm/buzzer.h 中
`Buzzer_Score_s` / `Buzzer_Note_s` 约束的曲谱数据：

  * 单声部：取主旋律轨（默认轨 1）的最高声部，和弦只保留顶音；
    主旋律长停顿时用伴奏轨（默认轨 2）的最高声部填补；
  * 简化：短装饰音并入前一个音、小空隙连音化、同音相邻合并；
    低于 --min-pitch 的音符逐八度上移（蜂鸣器低音区几乎听不见）；
  * 每段曲谱最多 128 个音符（BUZZER_MAX_SCORE_NOTES）；
  * 音符频率为预计算毫赫兹常量，换算方式与 `BuzzerMidiToFrequency()` 一致；
  * 每个音符时值 >= 10 ms，BPM 在 30~300，频率在 20~8000 Hz。

节奏处理：MIDI 的变速折算进 duration_ticks（脚本按真实毫秒换算），因此整首
组曲的每个分段使用统一时基 BPM（默认 240，1 tick = 2.604 ms），播放速度与
原曲一致；`BuzzerSetMusicSpeed()` 仍然可以整体变速。

用法：
    python3 midi_to_score.py input.mid --out-dir .. --name rinascita_suite \
        --title "鸣潮 黎那汐塔版本组曲" [--voice kBuzzerVoiceNormal]

重新生成示例（在本目录执行）：
    python3 midi_to_score.py "<原曲.mid>" --out-dir .. --name rinascita_suite \
        --title "鸣潮 黎那汐塔版本组曲"
"""

import argparse
import math
import os
import struct
import sys

# 与 buzzer.h / buzzer.c 保持一致的常量
BUZZER_MAX_SCORE_NOTES = 128
BUZZER_QUARTER_TICKS = 96
BUZZER_MIN_FREQUENCY_MILLIHZ = 20000
BUZZER_MAX_FREQUENCY_MILLIHZ = 8000000
MIN_NOTE_MS = 10

# 与 buzzer.c 的 kMidiOctave[] 完全一致（MIDI 60 = C4 = 261626 mHz）
MIDI_OCTAVE_MILLIHZ = [261626, 277183, 293665, 311127, 329628, 349228,
                       369994, 391995, 415305, 440000, 466164, 493883]


def midi_to_frequency(midi_note):
    """与 BuzzerMidiToFrequency() 相同的定点换算，超出硬件范围返回 0。"""
    if midi_note > 127:
        return 0
    octave = midi_note // 12 - 5
    frequency = MIDI_OCTAVE_MILLIHZ[midi_note % 12]
    if octave < 0:
        shift = -octave
        frequency = (frequency + (1 << (shift - 1))) >> shift
    else:
        frequency <<= octave
    return frequency if BUZZER_MIN_FREQUENCY_MILLIHZ <= frequency <= BUZZER_MAX_FREQUENCY_MILLIHZ else 0


# --------------------------------------------------------------------------- #
# 极简 MIDI 解析（format 0/1，division 为每四分音符 tick 数）
# --------------------------------------------------------------------------- #

def _read_vlq(data, index):
    value = 0
    while True:
        byte = data[index]
        index += 1
        value = (value << 7) | (byte & 0x7F)
        if not byte & 0x80:
            return value, index


def parse_midi(path):
    with open(path, 'rb') as handle:
        data = handle.read()
    if data[:4] != b'MThd':
        raise ValueError('不是 MIDI 文件')
    header_len, _fmt, track_count, division = struct.unpack('>IHHH', data[4:14])
    if division & 0x8000:
        raise ValueError('不支持 SMPTE division')

    position = 8 + header_len
    tracks = []
    for _ in range(track_count):
        if data[position:position + 4] != b'MTrk':
            raise ValueError('缺少 MTrk 块')
        track_len = struct.unpack('>I', data[position + 4:position + 8])[0]
        body = data[position + 8:position + 8 + track_len]
        position += 8 + track_len

        index, tick, running = 0, 0, None
        events = []
        while index < len(body):
            delta, index = _read_vlq(body, index)
            tick += delta
            status = body[index]
            if status & 0x80:
                index += 1
                if status < 0xF0:
                    running = status
            else:
                status = running
            kind = status & 0xF0
            channel = status & 0x0F
            if status == 0xFF:
                meta = body[index]
                index += 1
                length, index = _read_vlq(body, index)
                payload = body[index:index + length]
                index += length
                if meta == 0x51:
                    tempo_us = (payload[0] << 16) | (payload[1] << 8) | payload[2]
                    events.append((tick, 'tempo', tempo_us))
            elif status in (0xF0, 0xF7):
                length, index = _read_vlq(body, index)
                index += length
            elif kind in (0x80, 0x90, 0xA0, 0xB0, 0xE0):
                data1, data2 = body[index], body[index + 1]
                index += 2
                if kind == 0x90 and data2 > 0:
                    events.append((tick, 'on', channel, data1, data2))
                elif kind in (0x80, 0x90):
                    events.append((tick, 'off', channel, data1, data2))
            elif kind in (0xC0, 0xD0):
                index += 1
            else:
                raise ValueError('未知 MIDI 状态字节 %#x' % status)
        tracks.append(events)
    return division, tracks


def track_notes(events):
    """(start_tick, end_tick, pitch, velocity) 列表。"""
    notes, active = [], {}
    for event in events:
        if event[1] == 'on':
            active.setdefault((event[2], event[3]), []).append((event[0], event[4]))
        elif event[1] == 'off':
            pending = active.get((event[2], event[3]))
            if pending:
                start, velocity = pending.pop(0)
                notes.append([start, event[0], event[3], velocity])
    return sorted(notes)


# --------------------------------------------------------------------------- #
# 主旋律提取与简化
# --------------------------------------------------------------------------- #

def top_voice_melody(notes):
    """单声部旋律线：按起音时间扫描，只保留最高声部（和弦取顶音）。

    新起音只有在音高不低于当前旋律音时才"抢入"，否则视为内声部/伴奏被忽略，
    避免和弦中声部破坏长旋律音。
    """
    onsets = {}
    for start, end, pitch, velocity in notes:
        onsets.setdefault(start, []).append((end, pitch, velocity))

    melody = []  # [start_tick, end_tick, pitch, velocity, chord]；chord 为该起音的全部 (音高, 力度)
    for start in sorted(onsets):
        end, pitch, velocity = max(onsets[start], key=lambda item: item[1])
        chord = sorted((p, v) for _e, p, v in onsets[start])
        if melody and start < melody[-1][1]:
            if pitch < melody[-1][2]:
                continue  # 低于当前旋律音，判定为内声部
            melody[-1][1] = start  # 抢入：截断前一个旋律音
        melody.append([start, end, pitch, velocity, chord])
    return melody


def fill_silences(melody, backup_notes, gap_ms, tick_to_ms):
    """主旋律停顿超过 gap_ms 时，用伴奏轨顶音填补，避免长时间空白。"""
    filled = []
    cursor_ms = 0.0
    for note in melody:
        start_ms, end_ms = tick_to_ms(note[0]), tick_to_ms(note[1])
        if start_ms - cursor_ms > gap_ms:
            window = [n for n in backup_notes if tick_to_ms(n[0]) >= cursor_ms and tick_to_ms(n[1]) <= start_ms]
            for sub in top_voice_melody(window):
                filled.append(list(sub))
        filled.append(note)
        cursor_ms = max(cursor_ms, end_ms)
    return filled


def simplify(melody, tick_to_ms, legato_gap_ms, min_note_ms):
    """连音化、剔除超短装饰音、合并同音相邻，输出 [start_ms, end_ms, pitch, velocity, chord]。"""
    line = [[tick_to_ms(s), tick_to_ms(e), p, v, chord] for s, e, p, v, chord in melody]

    bridged = []
    for index, (start, end, pitch, velocity, chord) in enumerate(line):
        if index + 1 < len(line) and line[index + 1][0] - end < legato_gap_ms:
            end = line[index + 1][0]
        bridged.append([start, end, pitch, velocity, chord])

    kept = []
    for start, end, pitch, velocity, chord in bridged:
        if kept and end - start < min_note_ms:
            kept[-1][1] = max(kept[-1][1], end)  # 并入前一个音
            continue
        kept.append([start, end, pitch, velocity, chord])

    merged = []
    for start, end, pitch, velocity, chord in kept:
        if merged and merged[-1][2] == pitch and start - merged[-1][1] < 1e-6:
            merged[-1][1] = end
        else:
            merged.append([start, end, pitch, velocity, chord])
    return merged


def normalize_velocity(melody):
    """把源力度（含和弦内声部）线性映射到 64~127，保持强弱对比但保证蜂鸣器响度。"""
    velocities = [note[3] for note in melody] + [v for note in melody for _p, v in note[4]]
    low, high = min(velocities), max(velocities)
    span = max(high - low, 1)

    def map_velocity(value):
        return 64 + round((value - low) * (127 - 64) / span)

    for note in melody:
        note[3] = map_velocity(note[3])
        note[4] = [(p, map_velocity(v)) for p, v in note[4]]
    return melody


def apply_pitch_floor(melody, min_pitch):
    """低于 min_pitch 的音符（多为伴奏轨填补段的低音）逐八度上移，保证蜂鸣器可听。"""
    for note in melody:
        while note[2] < min_pitch:
            note[2] += 12
        chord = []
        for pitch, velocity in note[4]:
            while pitch < min_pitch:
                pitch += 12
            chord.append((pitch, velocity))
        note[4] = chord
    return melody


# --------------------------------------------------------------------------- #
# 转换为 Buzzer_Note_s / Buzzer_Score_s
# --------------------------------------------------------------------------- #

def build_entries(melody_ms, time_base_bpm, roll_max=0, roll_note_ms=40):
    """[start_ms, end_ms, pitch, velocity, chord] -> [(frequency_millihz, ticks, velocity)]

    音符时值独立取整（节奏误差半个 tick 以内）；起音位置按绝对时间对齐，取整
    残差写进前面的休止符或并入前一个音，误差不沿整首累积。

    和弦分解（roll_max > 0）：和弦起音在旋律音（顶音）之前插入最多 roll_max 个
    自低向高的短琶音，占用起音前的空隙或前一个音的尾巴，旋律音起音保持不变；
    空间不够时减少琶音音数，快速段落自动不加，不改变原曲节奏。
    """
    tick_ms = 60000.0 / (time_base_bpm * BUZZER_QUARTER_TICKS)
    min_ticks = max(1, math.ceil(MIN_NOTE_MS / tick_ms - 1e-9))  # ceil(10ms / tick)
    roll_ticks = max(min_ticks, round(roll_note_ms / tick_ms))
    keep_note_ticks = max(min_ticks, round(40 / tick_ms))  # 琶音截取后前一个音至少保留的时值

    def fill_gap(gap_ticks):
        """把起音前的空隙写成休止符，或并入前一个音。"""
        nonlocal entries, played_ticks
        if gap_ticks >= min_ticks:
            entries.append((0, gap_ticks, 0))
            played_ticks += gap_ticks
        elif entries and gap_ticks > 0:
            previous = entries[-1]
            entries[-1] = (previous[0], previous[1] + gap_ticks, previous[2])
            played_ticks += gap_ticks

    entries = []
    played_ticks = 0
    for start_ms, end_ms, pitch, velocity, chord in melody_ms:
        onset_ticks = round(start_ms / tick_ms)
        ticks = max(min_ticks, round((end_ms - start_ms) / tick_ms))

        roll = []
        if roll_max > 0 and end_ms - start_ms >= 3 * roll_note_ms:
            roll = sorted({p for p, _v in chord if p < pitch})[-roll_max:]
            while roll and roll_ticks * len(roll) > onset_ticks - played_ticks + max(
                    0, (entries[-1][1] - (min_ticks if entries[-1][0] == 0 else keep_note_ticks)) if entries else 0):
                roll = roll[1:]  # 空间不足时从最低音开始砍，保留最靠近旋律音的

        roll_start = onset_ticks - roll_ticks * len(roll)
        if entries and roll_start < played_ticks:  # 截取前一个音的尾巴给琶音腾位置
            cut = played_ticks - roll_start
            previous = entries[-1]
            entries[-1] = (previous[0], previous[1] - cut, previous[2])
            played_ticks -= cut
        fill_gap(roll_start - played_ticks)

        for roll_pitch in roll:
            entries.append((midi_to_frequency(roll_pitch), roll_ticks,
                            dict(chord).get(roll_pitch, velocity)))
            played_ticks += roll_ticks
        entries.append((midi_to_frequency(pitch), ticks, velocity))
        played_ticks += ticks
    return entries


def split_segments(entries, max_notes):
    """按 <=128 个音符切段；切点尽量落在休止符/长音上，保持乐句完整。"""
    segments, current = [], []
    for entry in entries:
        current.append(entry)
        if len(current) == max_notes:
            split_at = len(current)
            for back in range(len(current) - 1, max(len(current) - 8, 0), -1):
                if current[back][0] == 0 or current[back][1] >= 192:  # 休止符或长音
                    split_at = back
                    break
            segments.append(current[:split_at])
            current = current[split_at:]
    if current:
        segments.append(current)
    return segments


def format_c(args, segments, tick_ms):
    note_names = []
    for index, segment in enumerate(segments):
        note_names.append('%sSegment%02dNotes' % (args.symbol, index))

    lines = []
    lines.append('#include "%s.h"' % args.name)
    lines.append('')
    lines.append('#include <stddef.h>')
    lines.append('')
    lines.append('/* 本文件由 tools/midi_to_score.py 生成，请勿手工改动；修改曲目后重新运行脚本。')
    lines.append(' *')
    lines.append(' * 曲目：%s' % args.title)
    lines.append(' * 时基：BPM %d（BUZZER_QUARTER_TICKS=%d，1 tick = %.3f ms），原曲变速已折算进时值。'
                 % (args.time_base_bpm, BUZZER_QUARTER_TICKS, tick_ms))
    lines.append(' * 音色：%s，力度按原曲强弱归一化到 64~127。' % args.voice)
    lines.append(' * 提取参数：legato %g ms / min-note %g ms / fill-gap %g ms / 和弦分解 %d 音 %g ms。'
                 % (args.legato_ms, args.min_note_ms, args.fill_gap_ms, args.chord_roll, args.roll_note_ms))
    total = sum(len(segment) for segment in segments)
    lines.append(' * 共 %d 段 / %d 个音符。' % (len(segments), total))
    lines.append(' */')
    lines.append('')

    cursor_ticks = 0
    for index, segment in enumerate(segments):
        start_s = cursor_ticks * tick_ms / 1000.0
        cursor_ticks += sum(entry[1] for entry in segment)
        end_s = cursor_ticks * tick_ms / 1000.0
        lines.append('/* 第 %02d 段  %02d:%04.1f ~ %02d:%04.1f */' % (
            index, int(start_s // 60), start_s % 60, int(end_s // 60), end_s % 60))
        lines.append('static const Buzzer_Note_s %s[] = {' % note_names[index])
        for frequency, ticks, velocity in segment:
            if frequency == 0:
                lines.append('    {0, %d, 0, NULL},' % ticks)
            else:
                lines.append('    {%d, %d, %d, &%s},' % (frequency, ticks, velocity, args.voice))
        lines.append('};')
        lines.append('')

    lines.append('const Buzzer_Score_s %sSegments[] = {' % args.symbol)
    for index, segment in enumerate(segments):
        lines.append('    {.notes = %s, .note_count = %d, .bpm = %d},' % (
            note_names[index], len(segment), args.time_base_bpm))
    lines.append('};')
    lines.append('')
    lines.append('const Buzzer_MusicSuite_s %s = {' % args.symbol)
    lines.append('    .segments = %sSegments,' % args.symbol)
    lines.append('    .segment_count = sizeof(%sSegments) / sizeof(%sSegments[0]),' % (
        args.symbol, args.symbol))
    lines.append('};')
    lines.append('')
    return '\n'.join(lines)


def format_h(args, segments):
    lines = []
    lines.append('#pragma once')
    lines.append('')
    lines.append('#include "buzzer_music.h"')
    lines.append('')
    lines.append('/* 曲目：%s（由 tools/midi_to_score.py 生成） */' % args.title)
    lines.append('extern const Buzzer_Score_s %sSegments[];' % args.symbol)
    lines.append('extern const Buzzer_MusicSuite_s %s;' % args.symbol)
    lines.append('')
    return '\n'.join(lines)


# --------------------------------------------------------------------------- #

def camel_case(name):
    return ''.join(part[:1].upper() + part[1:] for part in name.replace('-', '_').split('_'))


def main():
    parser = argparse.ArgumentParser(description='MIDI 主旋律 -> 蜂鸣器常量曲谱 C 文件')
    parser.add_argument('midi', help='输入 MIDI 文件')
    parser.add_argument('--out-dir', default='.', help='输出目录')
    parser.add_argument('--name', default='rinascita_suite', help='生成文件/符号前缀，下划线命名')
    parser.add_argument('--title', default='', help='曲目名称（写入注释）')
    parser.add_argument('--voice', default='kBuzzerVoiceNormal', help='音色预设符号')
    parser.add_argument('--melody-track', type=int, default=1, help='主旋律轨下标（默认 1）')
    parser.add_argument('--backup-track', type=int, default=2, help='填补停顿的伴奏轨下标（默认 2）')
    parser.add_argument('--time-base-bpm', type=int, default=240, help='曲谱时基 BPM（30~300）')
    parser.add_argument('--legato-ms', type=float, default=120.0, help='小于此空隙时连音化，不写休止符')
    parser.add_argument('--min-note-ms', type=float, default=30.0, help='小于此长度的装饰音并入前一个音')
    parser.add_argument('--fill-gap-ms', type=float, default=600.0, help='主旋律停顿超过该值时用伴奏轨填补')
    parser.add_argument('--chord-roll', type=int, default=3,
                        help='和弦分解：旋律音前自低向高琶音的最多音数（0 关闭，默认 3）')
    parser.add_argument('--roll-note-ms', type=float, default=40.0, help='和弦分解单个琶音音的时值')
    parser.add_argument('--min-pitch', default='auto',
                        help="音高下限（MIDI 音号）；低于下限的音符逐八度上移。'auto'（默认）取主旋律最低音")
    args = parser.parse_args()
    args.symbol = 'k' + camel_case(args.name)
    if not args.title:
        args.title = os.path.basename(args.midi)

    if not 30 <= args.time_base_bpm <= 300:
        raise SystemExit('时基 BPM 必须在 30~300')

    division, tracks = parse_midi(args.midi)
    tempo_events = sorted(event for events in tracks for event in events if event[1] == 'tempo')

    def tick_to_ms(tick):
        ms, prev_tick, prev_us = 0.0, 0, 500000
        for event in tempo_events:
            if event[0] >= tick:
                break
            ms += (event[0] - prev_tick) / division * prev_us / 1000.0
            prev_tick, prev_us = event[0], event[2]
        ms += (tick - prev_tick) / division * prev_us / 1000.0
        return ms

    tick_ms = 60000.0 / (args.time_base_bpm * BUZZER_QUARTER_TICKS)

    if args.melody_track >= len(tracks) or args.backup_track >= len(tracks):
        raise SystemExit('轨下标越界（该 MIDI 共 %d 轨）' % len(tracks))

    melody = top_voice_melody(track_notes(tracks[args.melody_track]))
    pitch_floor = min(note[2] for note in melody) if args.min_pitch == 'auto' else int(args.min_pitch)
    melody = fill_silences(melody, track_notes(tracks[args.backup_track]), args.fill_gap_ms, tick_to_ms)
    melody_ms = simplify(melody, tick_to_ms, args.legato_ms, args.min_note_ms)
    melody_ms = apply_pitch_floor(melody_ms, pitch_floor)
    melody_ms = normalize_velocity(melody_ms)

    entries = build_entries(melody_ms, args.time_base_bpm, args.chord_roll, args.roll_note_ms)
    segments = split_segments(entries, BUZZER_MAX_SCORE_NOTES)

    # 自检：保证满足 BuzzerPlayScore() 的全部校验
    total_ms = 0.0
    for segment in segments:
        if not 1 <= len(segment) <= BUZZER_MAX_SCORE_NOTES:
            raise SystemExit('段长度越界：%d' % len(segment))
        for frequency, ticks, velocity in segment:
            if ticks * tick_ms < MIN_NOTE_MS:
                raise SystemExit('音符短于 10 ms')
            if ticks > 65535:
                raise SystemExit('时值超出 uint16：%d' % ticks)
            if frequency and not BUZZER_MIN_FREQUENCY_MILLIHZ <= frequency <= BUZZER_MAX_FREQUENCY_MILLIHZ:
                raise SystemExit('频率越界：%d' % frequency)
            if velocity > 127:
                raise SystemExit('力度越界：%d' % velocity)
            total_ms += ticks * tick_ms

    os.makedirs(args.out_dir, exist_ok=True)
    c_path = os.path.join(args.out_dir, '%s.c' % args.name)
    h_path = os.path.join(args.out_dir, '%s.h' % args.name)
    with open(c_path, 'w', encoding='utf-8') as handle:
        handle.write(format_c(args, segments, tick_ms))
    with open(h_path, 'w', encoding='utf-8') as handle:
        handle.write(format_h(args, segments))

    print('生成 %s / %s' % (c_path, h_path))
    print('  段数 %d，音符 %d，总时长 %.1f s（原曲 %.1f s）'
          % (len(segments), sum(len(s) for s in segments), total_ms / 1000.0,
             tick_to_ms(max(e[0] for events in tracks for e in events)) / 1000.0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
