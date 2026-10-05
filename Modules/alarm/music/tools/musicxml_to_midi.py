#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""musicxml_to_midi.py — MusicXML / MXL（压缩包）转标准 MIDI。

面向 Audiveris 等 OMR 工具导出的 MusicXML：读取每个声部的音高、时值、连音线、
和弦、休止、弱起小节，按顺序把多份谱拼成一首曲子，输出 format 1 MIDI：

  轨 0：指挥轨（只放速度/拍号）
  轨 1：旋律轨（每份谱的第 1 个声部，按顺序拼接）
  轨 2：伴奏轨（每份谱的第 2 个声部，按顺序拼接）

这样生成的 MIDI 可以直接喂给 midi_to_score.py（默认 --melody-track 1 --backup-track 2）
转成蜂鸣器曲谱。

用法：
    python3 musicxml_to_midi.py out.mid --tempo 90 上.mxl 下.mxl
    python3 musicxml_to_midi.py out.mid --tempo 90 --title "曲名" a.mxl
"""

import argparse
import os
import struct
import sys
import zipfile
import xml.etree.ElementTree as ET

MIDI_DIVISION = 480  # 每四分音符 tick 数

STEP_SEMITONE = {'C': 0, 'D': 2, 'E': 4, 'F': 5, 'G': 7, 'A': 9, 'B': 11}


def load_musicxml(path):
    """MXL 是 zip（内含 container.xml 指向根 XML）；也支持裸 .xml/.musicxml。"""
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            rootfile = None
            if 'META-INF/container.xml' in names:
                container = ET.fromstring(archive.read('META-INF/container.xml'))
                node = container.find('.//rootfile')
                if node is not None:
                    rootfile = node.get('full-path')
            if rootfile is None:
                rootfile = next((n for n in names if n.endswith('.xml') and 'META-INF' not in n), None)
            if rootfile is None:
                raise ValueError('%s: 压缩包内找不到 MusicXML' % path)
            return ET.fromstring(archive.read(rootfile))
    return ET.parse(path).getroot()


def note_to_midi(pitch):
    step = pitch.findtext('step')
    octave = int(pitch.findtext('octave'))
    alter = int(pitch.findtext('alter') or 0)
    return 12 * (octave + 1) + STEP_SEMITONE[step] + alter


def parse_part(part, divisions_default=4, octave_shift=False):
    """返回该声部的音符列表 [start_tick, duration_ticks, midi_pitch]（含和弦，已合并连音线）。

    octave_shift=True 时按谱面印刷的 8va/8vb 记号修正音高：MusicXML 里 Audiveris 写的是
    书面音高（画在谱表上的位置），`octave-shift type` 与印刷记号方向相反，因此
    type="down"（印刷 8va）→ +12，type="up"（印刷 8vb）→ -12。区间取到该小节末
    （Audiveris 未输出 type="stop"，括号基本只跨所在小节）。
    """
    notes = []
    cursor = 0          # 当前声部时间游标（tick）
    divisions = divisions_default
    last_note_ticks = 0
    last_pitch_end = {}  # 用于连音线合并：pitch -> (index, end_tick)
    oct_offset = 0

    for measure in part.findall('measure'):
        oct_offset = 0
        for el in measure:
            if el.tag == 'attributes':
                div = el.findtext('divisions')
                if div:
                    divisions = int(div)
                continue
            if el.tag == 'direction':
                for osc in el.findall('.//octave-shift'):
                    kind = osc.get('type')
                    if octave_shift and kind == 'down':
                        oct_offset = 12   # 印刷 8va：比书面高八度
                    elif octave_shift and kind == 'up':
                        oct_offset = -12  # 印刷 8vb：比书面低八度
                continue
            if el.tag == 'backup':
                cursor -= int(el.findtext('duration')) * MIDI_DIVISION // divisions
                continue
            if el.tag == 'forward':
                cursor += int(el.findtext('duration')) * MIDI_DIVISION // divisions
                continue
            if el.tag != 'note':
                continue

            ticks = int(el.findtext('duration') or 0) * MIDI_DIVISION // divisions
            is_chord = el.find('chord') is not None
            is_rest = el.find('rest') is not None
            ties = [t.get('type') for t in el.findall('tie')]

            if is_rest:
                if not is_chord:
                    cursor += ticks
                continue

            start = cursor if not is_chord else cursor - last_note_ticks
            pitch_el = el.find('pitch')
            if pitch_el is None:
                continue
            pitch = note_to_midi(pitch_el) + oct_offset

            if 'stop' in ties:
                # 连音线：延长上一个同音高音符，不新增音
                prev = last_pitch_end.get(pitch)
                if prev is not None:
                    index, end_tick = prev
                    if notes[index][1] + notes[index][2] >= start:  # 紧邻
                        notes[index][2] = start + ticks - notes[index][1]
                        last_pitch_end[pitch] = (index, notes[index][1] + notes[index][2])
                        if not is_chord:
                            cursor += ticks
                            last_note_ticks = ticks
                        continue

            notes.append([start, ticks, pitch])
            last_pitch_end[pitch] = (len(notes) - 1, start + ticks)
            if not is_chord:
                cursor += ticks
                last_note_ticks = ticks

    return notes


def track_end(notes):
    return max((start + dur for start, dur, _pitch in notes), default=0)


def build_midi(out_path, tracks, tempo_bpm, title):
    """tracks: [(name, [notes])]，每个音符 (start_tick, duration_ticks, pitch, velocity)。"""
    events = []  # (tick, order, bytes)
    events.append((0, 0, meta(0x03, title.encode('utf-8'))))
    events.append((0, 0, meta(0x51, struct.pack('>I', int(round(60_000_000 / tempo_bpm)))[1:])))
    events.append((0, 0, meta(0x58, bytes([4, 2, 24, 8]))))  # 4/4
    conductor = build_track_body(events)

    bodies = [conductor]
    for name, notes in tracks:
        evs = [(0, 0, meta(0x03, name.encode('utf-8')))]
        for start, duration, pitch, velocity in notes:
            evs.append((start, 2, vlq(0) + b'\x90' + bytes([pitch, velocity])))
            evs.append((start + duration, 1, vlq(0) + b'\x80' + bytes([pitch, 0])))
        evs.sort(key=lambda item: (item[0], item[1]))
        bodies.append(build_track_body(evs))

    header = b'MThd' + struct.pack('>IHHH', 6, 1, len(bodies), MIDI_DIVISION)
    with open(out_path, 'wb') as handle:
        handle.write(header)
        for body in bodies:
            handle.write(b'MTrk' + struct.pack('>I', len(body)) + body)


def vlq(value):
    out = [value & 0x7F]
    value >>= 7
    while value:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    return bytes(reversed(out))


def meta(kind, payload):
    return vlq(0) + b'\xFF' + bytes([kind]) + vlq(len(payload)) + payload


def build_track_body(events):
    track = bytearray()
    previous = 0
    for tick, _order, payload in sorted(events, key=lambda item: (item[0], item[1])):
        data = bytearray(payload)
        data[0:1] = vlq(tick - previous)
        track += data
        previous = tick
    track += meta(0x2F, b'')
    return bytes(track)


def main():
    parser = argparse.ArgumentParser(description='MusicXML/MXL -> MIDI')
    parser.add_argument('inputs', nargs='+', help='输入 .mxl/.musicxml/.xml，按拼接顺序')
    parser.add_argument('--out', required=True, help='输出 MIDI 路径')
    parser.add_argument('--tempo', type=float, required=True, help='速度 BPM（MusicXML 常无速度标记）')
    parser.add_argument('--title', default='', help='MIDI 曲名')
    parser.add_argument('--melody-part', type=int, default=0, help='旋律声部下标（默认 0）')
    parser.add_argument('--accomp-part', type=int, default=1, help='伴奏声部下标（默认 1）')
    parser.add_argument('--octave-shift', action='store_true',
                        help='按谱面 8va/8vb 印刷记号修正音高（Audiveris 存的是书面音高）')
    args = parser.parse_args()

    title = args.title or (os.path.splitext(os.path.basename(args.out))[0])
    melody_all, accomp_all = [], []
    offset = 0

    for path in args.inputs:
        root = load_musicxml(path)
        parts = root.findall('part')
        base = os.path.splitext(os.path.basename(path))[0]
        ends = []
        for index, part in enumerate(parts):
            notes = parse_part(part, octave_shift=args.octave_shift)
            if not notes:
                continue
            converted = [(start + offset, dur, pitch, 100) for start, dur, pitch in notes]
            if index == args.melody_part:
                melody_all.extend(converted)
            elif index == args.accomp_part:
                accomp_all.extend(converted)
            ends.append(track_end(notes) + offset)
            print('  %s part%d: %d 音符, %d tick (%.1f s)' % (
                base, index, len(notes), track_end(notes), track_end(notes) / MIDI_DIVISION * 60.0 / args.tempo))
        offset = max(ends) if ends else offset

    total = max((s + d for s, d, _p, _v in melody_all + accomp_all), default=0)
    build_midi(args.out, [('melody', melody_all), ('accomp', accomp_all)], args.tempo, title)
    print('生成 %s' % args.out)
    print('  旋律 %d 音 / 伴奏 %d 音，总时长 %.1f s（%d BPM）'
          % (len(melody_all), len(accomp_all), total / MIDI_DIVISION * 60.0 / args.tempo, args.tempo))
    return 0


if __name__ == '__main__':
    sys.exit(main())
