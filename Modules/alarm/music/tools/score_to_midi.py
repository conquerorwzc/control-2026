#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""score_to_midi.py — 把生成的蜂鸣器曲谱 C 文件还原为 MIDI，用于试听/校对音乐性。

反向对应 tools/midi_to_score.py：解析 C 文件里的 `Buzzer_Note_s` 常量数组和
`Buzzer_Score_s` 时基，按 MIDI division 480 重新打包（BUZZER_QUARTER_TICKS=96
对应四分音符，即 1 个曲谱 tick = 5 个 MIDI tick）。还原的是 MCU 将要播放的
单声部结果——音高、时值、力度、休止和段边界都与固件一致，段边界会写成 MIDI
marker，方便在打谱/播放软件里对照。

用法：
    python3 score_to_midi.py ../rinascita_suite.c --out 游戏_mcu.mid \
        [--title "曲目（MCU 蜂鸣器版）"] [--program 0]
"""

import argparse
import math
import re
import struct
import sys

MIDI_DIVISION = 480
BUZZER_QUARTER_TICKS = 96

NOTE_ARRAY_RE = re.compile(
    r'static const Buzzer_Note_s (\w+)\[\] = \{(.*?)\};', re.S)
NOTE_ENTRY_RE = re.compile(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(NULL|&\w+)\}')
SCORE_ARRAY_RE = re.compile(
    r'const Buzzer_Score_s (\w+)\[\] = \{(.*?)\};', re.S)
SCORE_ENTRY_RE = re.compile(
    r'\.notes = (\w+),\s*\.note_count = (\d+),\s*\.bpm = (\d+)')


def parse_score_file(path):
    """返回 [(segment_name, bpm, [(freq_millihz, ticks, velocity, voice), ...]), ...]"""
    text = open(path, encoding='utf-8').read()
    note_arrays = {name: [(int(f), int(t), int(v), voice)
                          for f, t, v, voice in NOTE_ENTRY_RE.findall(body)]
                   for name, body in NOTE_ARRAY_RE.findall(text)}

    segments = []
    for score_name, body in SCORE_ARRAY_RE.findall(text):
        for notes_name, note_count, bpm in SCORE_ENTRY_RE.findall(body):
            notes = note_arrays[notes_name]
            if len(notes) != int(note_count):
                raise ValueError('%s: note_count %s 与实际 %d 不符' % (notes_name, note_count, len(notes)))
            segments.append((notes_name, int(bpm), notes))
        break  # 只取第一个曲谱表（本文件只生成一首）
    if not segments:
        raise ValueError('未找到 Buzzer_Score_s 分段表')
    return segments


def millihz_to_midi(frequency_millihz):
    return int(round(69 + 12 * math.log2(frequency_millihz / 440000.0)))


# --------------------------------------------------------------------------- #
# 标准 MIDI 文件写出（format 0，单轨）
# --------------------------------------------------------------------------- #

def vlq(value):
    if value < 0:
        raise ValueError('负 delta time')
    out = [value & 0x7F]
    value >>= 7
    while value:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    return bytes(reversed(out))


def meta_event(delta, meta, payload):
    return vlq(delta) + b'\xFF' + bytes([meta]) + vlq(len(payload)) + payload


def build_track(segments, title, program):
    events = []  # (tick, order, bytes)；order 保证同刻先关后开
    events.append((0, 0, meta_event(0, 0x03, title.encode('utf-8'))))
    events.append((0, 0, meta_event(0, 0xC0, bytes([program]))))

    cursor_ticks = 0
    for index, (_name, bpm, notes) in enumerate(segments):
        if index == 0 or bpm != segments[index - 1][1]:
            tempo_us = int(round(60_000_000 / bpm))
            events.append((cursor_ticks, 0,
                           meta_event(0, 0x51, struct.pack('>I', tempo_us)[1:])))
        events.append((cursor_ticks, 0,
                       meta_event(0, 0x06, ('第%02d段' % index).encode('utf-8'))))
        for frequency, ticks, velocity, _voice in notes:
            start = cursor_ticks
            cursor_ticks += ticks * (MIDI_DIVISION // BUZZER_QUARTER_TICKS)
            if frequency == 0:
                continue  # 休止符：只推进时间
            key = millihz_to_midi(frequency)
            events.append((start, 2, vlq(0) + b'\x90' + bytes([key, velocity])))
            events.append((cursor_ticks, 1, vlq(0) + b'\x80' + bytes([key, 0])))

    events.sort(key=lambda item: (item[0], item[1]))
    track = bytearray()
    previous = 0
    for tick, _order, payload in events:
        data = bytearray(payload)
        data[0:1] = vlq(tick - previous)
        track += data
        previous = tick
    track += meta_event(0, 0x2F, b'')  # End of Track
    return bytes(track), cursor_ticks


def write_midi(path, track):
    header = b'MThd' + struct.pack('>IHHH', 6, 0, 1, MIDI_DIVISION)
    body = b'MTrk' + struct.pack('>I', len(track)) + track
    with open(path, 'wb') as handle:
        handle.write(header + body)


def main():
    parser = argparse.ArgumentParser(description='蜂鸣器曲谱 C 文件 -> MIDI（试听/校对）')
    parser.add_argument('score', help='midi_to_score.py 生成的 .c 文件')
    parser.add_argument('--out', required=True, help='输出 MIDI 路径')
    parser.add_argument('--title', default='', help='写入 MIDI 的曲名')
    parser.add_argument('--program', type=int, default=0, help='GM 音色号（默认 0 钢琴）')
    args = parser.parse_args()

    segments = parse_score_file(args.score)
    title = args.title or args.score
    track, total_ticks = build_track(segments, title, args.program)
    write_midi(args.out, track)

    note_count = sum(1 for _n, _b, notes in segments for f, _t, _v, _vo in notes if f)
    rest_count = sum(len(notes) for _n, _b, notes in segments) - note_count
    first_bpm = segments[0][1]
    seconds = total_ticks * 60.0 / (first_bpm * MIDI_DIVISION)
    print('生成 %s' % args.out)
    print('  %d 段 / %d 个音符 / %d 个休止符，时长 %.1f s' % (len(segments), note_count, rest_count, seconds))
    return 0


if __name__ == '__main__':
    sys.exit(main())
