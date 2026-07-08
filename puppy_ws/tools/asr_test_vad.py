#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VAD 版 ASR 测试脚本：自动检测说话开始/结束 + 高增益 + 降噪。
解决固定录音窗口时机难把握的问题。
依赖：vosk, webrtcvad, sox
"""
import argparse
import json
import os
import subprocess
import sys
import time
import wave
from difflib import SequenceMatcher

from vosk import Model, KaldiRecognizer

try:
    import webrtcvad
    HAS_VAD = True
except ImportError:
    HAS_VAD = False
    print('[WARN] 未装 webrtcvad，将回退到固定录音模式')
    print('       安装: pip3 install webrtcvad')


COMMAND_SYNONYMS = {
    '坐下': ['坐下', '坐下来', '坐', '座下', '做下'],
    '站起来': ['站起来', '站立', '站起', '站', '站啦'],
    '前进': ['前进', '向前走', '往前走', '前进走', '千进', '欠进'],
    '左转': ['左转', '向左转', '左拐', '左钻', '左赚'],
    '右转': ['右转', '向右转', '右拐', '右钻', '右赚'],
    '停下': ['停下', '停止', '别动', '挺下'],
    '小狗': ['小狗', '小狗儿', '小购'],
    '后退': ['后退', '向后走', '往后走', '后腿'],
}


def fuzzy_match(text: str, threshold: float = 0.6):
    if not text:
        return None, 0.0
    best_match = None
    best_score = 0.0
    for cmd, synonyms in COMMAND_SYNONYMS.items():
        for syn in synonyms:
            if syn in text:
                return cmd, 1.0
            score = SequenceMatcher(None, syn, text).ratio()
            if score > best_score:
                best_score = score
                best_match = cmd
    if best_score >= threshold:
        return best_match, best_score
    return None, best_score


def record_with_vad(device: str, wav_path: str,
                    max_wait: float = 8.0,
                    max_speech: float = 5.0,
                    silence_dur: float = 0.8,
                    aggressiveness: int = 2,
                    gain_db: int = 10,
                    noise_prof: str = None):
    """
    VAD 录音：自动检测说话开始和结束。
    - max_wait: 最多等多少秒检测到说话
    - max_speech: 单次说话最长多少秒
    - silence_dur: 说话后静音多少秒判定结束
    - aggressiveness: VAD 灵敏度 0-3（3最灵敏）
    """
    if not HAS_VAD:
        # 回退到固定录音
        raw = wav_path.replace('.wav', '_raw.wav')
        subprocess.run(['arecord', '-D', device, '-d', '4',
                        '-f', 'S16_LE', '-r', '16000', '-c', '1', raw],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        _postprocess(raw, wav_path, gain_db, noise_prof)
        return 'fixed-4s'

    vad = webrtcvad.Vad(aggressiveness)
    sample_rate = 16000
    frame_duration = 30  # ms
    frame_size = int(sample_rate * frame_duration / 1000) * 2  # bytes (S16)

    # 用 arecord 流式录音
    proc = subprocess.Popen(
        ['arecord', '-D', device,
         '-f', 'S16_LE', '-r', str(sample_rate), '-c', '1',
         '-t', 'raw', '-q'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )

    raw_path = wav_path.replace('.wav', '_raw.wav')
    raw_frames = []
    state = 'waiting'  # waiting -> speaking -> done
    wait_start = time.time()
    speech_start = 0
    silence_start = 0
    total_speech = 0

    # 连续帧判断：需要连续 N 帧有声音才算说话开始（防误触发）
    speech_frame_count = 0      # 当前连续语音帧数
    min_speech_frames = 6       # 连续 6 帧 ≈ 180ms 才算说话
    # 连续帧判断：需要连续 N 帧静音才算说话结束（防过早截断）
    silence_frame_count = 0     # 当前连续静音帧数
    min_silence_frames = int(silence_dur * 1000 / frame_duration)  # 按时间换算

    try:
        while True:
            frame = proc.stdout.read(frame_size)
            if len(frame) < frame_size:
                break

            is_speech = vad.is_speech(frame, sample_rate)
            now = time.time()

            if state == 'waiting':
                if is_speech:
                    speech_frame_count += 1
                    raw_frames.append(frame)
                    if speech_frame_count >= min_speech_frames:
                        state = 'speaking'
                        speech_start = now
                        silence_start = now
                        silence_frame_count = 0
                        print('  [VAD] 检测到说话开始', end='', flush=True)
                else:
                    # 静音帧：重置计数，丢弃之前缓冲的噪声帧
                    speech_frame_count = 0
                    raw_frames = []
                    if now - wait_start > max_wait:
                        print('  [VAD] 等待超时（没说话）', end='', flush=True)
                        break

            elif state == 'speaking':
                raw_frames.append(frame)
                if is_speech:
                    silence_frame_count = 0
                    silence_start = now
                    total_speech = now - speech_start
                    if total_speech > max_speech:
                        print(' → 说话超时', end='', flush=True)
                        break
                else:
                    silence_frame_count += 1
                    if silence_frame_count >= min_silence_frames:
                        print(f' → 说话结束（{total_speech:.1f}秒）', flush=True)
                        break
    finally:
        proc.terminate()
        proc.wait()

    if not raw_frames:
        # 没录到，写空文件
        with wave.open(wav_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
        return 'empty'

    # 写原始 raw -> wav
    with wave.open(raw_path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(raw_frames))

    _postprocess(raw_path, wav_path, gain_db, noise_prof)
    return 'vad'


def _postprocess(raw_path, wav_path, gain_db, noise_prof):
    """sox 后处理：降噪 + 增益"""
    if gain_db != 0 or (noise_prof and os.path.exists(noise_prof)):
        cmd = ['sox', raw_path, wav_path]
        if noise_prof and os.path.exists(noise_prof):
            cmd += ['noisered', noise_prof, '0.2']
        if gain_db != 0:
            cmd += ['gain', str(gain_db)]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.remove(raw_path)
    else:
        os.rename(raw_path, wav_path)


def asr_recognize(model: Model, wav_path: str):
    wf = wave.open(wav_path, 'rb')
    rec = KaldiRecognizer(model, 16000)
    rec.SetWords(True)
    text = ''
    confidences = []
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            res = json.loads(rec.Result())
            text += res.get('text', '')
            for w in res.get('result', []):
                confidences.append(w.get('conf', 0))
    res = json.loads(rec.FinalResult())
    text += res.get('text', '')
    for w in res.get('result', []):
        confidences.append(w.get('conf', 0))
    wf.close()
    avg_conf = sum(confidences) / len(confidences) if confidences else 0
    return text.strip(), avg_conf


def main():
    parser = argparse.ArgumentParser(description='VAD 版 ASR 测试')
    parser.add_argument('--mic', default='plughw:1,0')
    parser.add_argument('--model',
                        default='/app/puppy_ws/models/vosk-model-small-cn-0.22')
    parser.add_argument('--big-model',
                        default='/app/puppy_ws/models/vosk-model-cn-0.22')
    parser.add_argument('--save', default='/tmp/asr_test')
    parser.add_argument('--denoise', action='store_true')
    parser.add_argument('--noise-prof', default='/tmp/noise.prof')
    parser.add_argument('--gain', type=int, default=10, help='增益dB（默认10）')
    parser.add_argument('--threshold', type=float, default=0.6)
    parser.add_argument('--aggressiveness', type=int, default=2,
                        help='VAD灵敏度0-3，3最灵敏')
    parser.add_argument('--max-wait', type=float, default=8.0,
                        help='最多等多少秒检测到说话')
    parser.add_argument('--silence', type=float, default=0.8,
                        help='说话后静音多少秒判定结束')
    args = parser.parse_args()

    os.makedirs(args.save, exist_ok=True)

    # 优先用大模型
    model_path = args.model
    if os.path.exists(args.big_model):
        model_path = args.big_model
        print(f'[INFO] 使用大模型: {model_path}')
    else:
        print(f'[INFO] 使用小模型: {model_path}')

    if args.denoise:
        if os.path.exists(args.noise_prof):
            print(f'[INFO] 开启降噪')
        else:
            print(f'[WARN] 噪声样本不存在，不降噪')
            args.denoise = False

    if HAS_VAD:
        print(f'[INFO] VAD 灵敏度: {args.aggressiveness} (0-3, 3最灵敏)')

    print(f'[INFO] 增益: {args.gain}dB')
    print('=' * 60)
    print(' VAD 版 ASR 测试')
    print(f' 麦克风: {args.mic}')
    print(f' 模型:   {model_path}')
    print(f' 增益:   {args.gain}dB' +
          (' + 降噪' if args.denoise else '') +
          (' + VAD' if HAS_VAD else ' + 固定4秒'))
    print('=' * 60)
    print('测试词：坐下 / 站起来 / 前进 / 左转 / 右转 / 停下 / 小狗')
    print('看到"请说话"后开始说，说完停 0.8 秒自动结束')
    print('按 Ctrl+C 退出\n')

    print('加载模型...')
    model = Model(model_path)
    print('加载完成\n')

    round_num = 0
    correct = 0
    total = 0

    while True:
        round_num += 1
        wav_path = os.path.join(args.save, f'rec_{round_num:03d}.wav')

        print(f'\n--- 第 {round_num} 轮 ---')
        print('[请说话] 随时开始，最长等 8 秒...', flush=True)
        mode = record_with_vad(
            args.mic, wav_path,
            max_wait=args.max_wait,
            silence_dur=args.silence,
            aggressiveness=args.aggressiveness,
            gain_db=args.gain,
            noise_prof=(args.noise_prof if args.denoise else None)
        )

        text, conf = asr_recognize(model, wav_path)

        if not text:
            print(f'  识别结果: (空)')
            print(f'  录音: {wav_path}')
        else:
            print(f'  识别结果: "{text}"  (置信度: {conf:.2f})')
            matched, score = fuzzy_match(text, args.threshold)
            total += 1
            if matched:
                correct += 1
                print(f'  ✅ [{matched}]  (相似度: {score:.2f})')
            else:
                print(f'  ❌ 未匹配  (最高: {score:.2f})')

        if total > 0:
            print(f'[统计] {correct}/{total} = {correct/total*100:.0f}%')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n退出')
