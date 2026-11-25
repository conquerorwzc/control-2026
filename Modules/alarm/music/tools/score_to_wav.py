#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""score_to_wav.py — 把蜂鸣器曲谱 C 文件模拟成 PWM 方波音频，输出 WAV 试听。

仿真严格按 Modules/alarm/buzzer.c 的合成链实现，尽量贴合 MCU 实际输出：

  * 方波频率 = 音符频率（毫赫兹），休止符静音；
  * 脉宽(占空比) = 音色 pulse_width_permille × 包络增益 × velocity / 127000
    （对应 BuzzerApplyVoice() 的 duty 计算），经 BuzzerSetStrength() 千分比缩放，
    PWM 分辨率按 1 MHz 计数时钟量化（步进 0.1%，蜂鸣器 TIM 独占约 1 MHz）；
  * ADSR 包络：attack/decay/release 未满 10 ms 的按 10 ms 下限执行
    （BuzzerStageValid），sustain 为千分比；释放阶段位于音符末尾、不延长时值；
  * 滑音 glide：从前一发声频率线性滑到当前频率，休止符后不滑音；
  * 颤音 vibrato：按频率比例调制（非音分），颤幅 tremolo 调制占空比，
    调制速率按真实播放时间推进、不随变速加倍，频率 1~10 Hz；
  * 采样率 44.1 kHz、16 位单声道。

与真机的差异（换能器相关，无法完全仿真）：蜂鸣器型号/驱动电路未确认，
实际扬声器或压电片的频响、谐振和失真会改变听感；本输出是"电端"波形。

用法：
    python3 score_to_wav.py ../rinascita_suite.c --out preview.wav
    python3 score_to_wav.py ../rinascita_suite.c --out s3.wav --start 120 --duration 60
"""

import argparse
import math
import re
import struct
import sys
import wave

SAMPLE_RATE = 44100
VOICE_RE = re.compile(
    r'const Buzzer_Voice_Config_s (\w+) = \{(.*?)\};', re.S)
FIELD_RE = re.compile(r'\.(\w+) = (\d+)')
NOTE_ARRAY_RE = re.compile(
    r'static const Buzzer_Note_s (\w+)\[\] = \{(.*?)\};', re.S)
NOTE_ENTRY_RE = re.compile(r'\{(\d+),\s*(\d+),\s*(\d+),\s*(NULL|&\w+)\}')
SCORE_ARRAY_RE = re.compile(
    r'const Buzzer_Score_s (\w+)\[\] = \{(.*?)\};', re.S)
SCORE_ENTRY_RE = re.compile(
    r'\.notes = (\w+),\s*\.note_count = (\d+),\s*\.bpm = (\d+)')


def parse_score_file(path):
    """解析生成的曲谱 C 文件：音色预设、分段曲谱。"""
    text = open(path, encoding='utf-8').read()

    voices = {}
    for name, body in VOICE_RE.findall(text):
        voice = dict(FIELD_RE.findall(body))
        voices[name] = {key: int(value) for key, value in voice.items()}

    note_arrays = {name: [(int(f), int(t), int(v), voice)
                          for f, t, v, voice in NOTE_ENTRY_RE.findall(body)]
                   for name, body in NOTE_ARRAY_RE.findall(text)}

    segments = []
    for _score_name, body in SCORE_ARRAY_RE.findall(text):
        for notes_name, note_count, bpm in SCORE_ENTRY_RE.findall(body):
            segments.append((int(bpm), note_arrays[notes_name]))
        break
    if not segments:
        raise ValueError('未找到 Buzzer_Score_s 分段表')
    return voices, segments


def envelope_gain(voice, position_ms, duration_ms):
    """ADSR 包络增益 0~1，release 位于音符末尾且不延长时值（与 buzzer.c 一致）。"""
    sustain = voice.get('sustain_permille', 1000) / 1000.0
    attack = max(voice.get('attack_ms', 0), 10) if voice.get('attack_ms', 0) else 0
    decay = max(voice.get('decay_ms', 0), 10) if voice.get('decay_ms', 0) else 0
    release = max(voice.get('release_ms', 0), 10) if voice.get('release_ms', 0) else 0

    if release and position_ms >= duration_ms - release:
        return sustain * max(0.0, 1.0 - (position_ms - (duration_ms - release)) / release)
    if attack and position_ms < attack:
        return position_ms / attack
    if decay and position_ms < attack + decay:
        return 1.0 - (1.0 - sustain) * (position_ms - attack) / decay
    return sustain


def render(score_path, out_path, start_s, duration_s, volume, sample_rate):
    voices, segments = parse_score_file(score_path)
    default_voice = {'pulse_width_permille': 500, 'sustain_permille': 1000}

    # 展开为连续事件流（每段连续播放，段边界不停顿）
    events = []  # (frequency_millihz, duration_ms, velocity, voice_dict)
    for bpm, notes in segments:
        tick_ms = 60000.0 / (bpm * 96)
        for freq, ticks, vel, voice_name in notes:
            voice = voices.get(voice_name.lstrip('&'), default_voice) if voice_name != 'NULL' else default_voice
            events.append((freq, ticks * tick_ms, vel, voice))

    total_ms = sum(duration for _f, duration, _v, _voice in events)
    t0_ms = start_s * 1000.0
    t1_ms = total_ms if duration_s <= 0 else min(total_ms, t0_ms + duration_s * 1000.0)
    total_samples = max(0, int((t1_ms - t0_ms) * sample_rate / 1000.0))

    out = [0.0] * total_samples
    clock = 0.0          # 当前时间 (ms)，同时作为颤音/颤幅的调制时基（按实际播放时间推进）
    prev_freq = 0.0      # 上一发声频率 (Hz)，用于滑音
    max_pulse = 0.0

    for freq_millihz, duration_ms, velocity, voice in events:
        if clock >= t1_ms:
            break
        freq = freq_millihz / 1000.0
        sustain_pulse = voice.get('pulse_width_permille', 500) / 1000.0
        glide = max(voice.get('glide_ms', 0), 10) if voice.get('glide_ms', 0) else 0
        vib_depth = voice.get('vibrato_depth_permille', 0) / 1000.0
        vib_hz = voice.get('vibrato_hz', 0)
        trem_depth = voice.get('tremolo_depth_permille', 0) / 1000.0
        trem_hz = voice.get('tremolo_hz', 0)

        note_start, note_end = clock, clock + duration_ms
        if note_end <= t0_ms or note_start >= t1_ms:
            if freq > 0:
                prev_freq = freq
            clock = note_end
            continue

        render_start = max(t0_ms, note_start)
        render_end = min(t1_ms, note_end)
        phase = 0.0
        idx = int((render_start - t0_ms) * sample_rate / 1000.0)
        step_ms = 1000.0 / sample_rate

        for tick in range(idx, int((render_end - t0_ms) * sample_rate / 1000.0)):
            pos = t0_ms + tick * step_ms - note_start  # 音符内位置 (ms)
            if freq > 0:
                f = freq
                if glide and prev_freq > 0 and pos < glide:
                    f = prev_freq + (freq - prev_freq) * pos / glide
                if vib_hz and vib_depth:
                    f *= 1.0 + vib_depth * math.sin(2 * math.pi * vib_hz * (note_start + pos) / 1000.0)
                gain = envelope_gain(voice, pos, duration_ms)
                pulse = sustain_pulse * gain * velocity / 127.0 * volume
                if trem_hz and trem_depth:
                    pulse *= 1.0 - trem_depth * (0.5 - 0.5 * math.cos(2 * math.pi * trem_hz * (note_start + pos) / 1000.0))
                # PWM 比较值按 1 MHz 计数时钟量化，且占空比不超过 50%
                pulse = min(500, round(pulse * 1000)) / 1000.0
                max_pulse = max(max_pulse, pulse)
                phase += f * step_ms / 1000.0
                out[tick] = 1.0 if (phase % 1.0) < pulse else -1.0
            else:
                out[tick] = 0.0
        if freq > 0:
            prev_freq = freq
        clock = note_end

    # 写 16 位单声道 WAV
    with wave.open(out_path, 'wb') as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(sample_rate)
        frames = bytearray()
        for value in out:
            frames += struct.pack('<h', max(-32767, min(32767, int(value * 32767))))
        handle.writeframes(bytes(frames))

    print('生成 %s' % out_path)
    print('  长度 %.1f s（%.1f ~ %.1f s），采样率 %d，最大占空比 %.1f%%'
          % (total_samples / sample_rate, start_s, t0_ms / 1000.0 + total_samples / sample_rate,
             sample_rate, max_pulse * 100))
    print('  提示：这是电端 PWM 波形；真机蜂鸣器换能器频响/谐振会改变听感。')
    return 0


def main():
    parser = argparse.ArgumentParser(description='蜂鸣器曲谱 C 文件 -> 模拟 PWM 方波 WAV')
    parser.add_argument('score', help='midi_to_score.py 生成的 .c 文件')
    parser.add_argument('--out', required=True, help='输出 WAV 路径')
    parser.add_argument('--start', type=float, default=0.0, help='起始秒数（长曲可截取试听）')
    parser.add_argument('--duration', type=float, default=0.0, help='截取时长秒数（0 = 到结尾）')
    parser.add_argument('--volume', type=float, default=1.0, help='音量缩放（对应 BuzzerSetStrength 千分比）')
    parser.add_argument('--rate', type=int, default=SAMPLE_RATE, help='采样率')
    args = parser.parse_args()
    return render(args.score, args.out, args.start, args.duration, args.volume, args.rate)


if __name__ == '__main__':
    sys.exit(main())
