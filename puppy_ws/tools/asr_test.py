#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小化 ASR 测试脚本：只录音 + Vosk 识别 + 打印。
不发 UDP，不播报，不依赖 sit.py。
用于排查 ASR 识别问题。
"""
import argparse
import json
import os
import subprocess
import sys
import time
import wave
import tempfile

from vosk import Model, KaldiRecognizer


def record_audio(device: str, seconds: int, wav_path: str):
    """用 arecord 录音到 wav 文件"""
    cmd = [
        'arecord', '-D', device,
        '-d', str(seconds),
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        wav_path
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def asr_recognize(model: Model, wav_path: str) -> str:
    """用 Vosk 识别 wav 文件"""
    wf = wave.open(wav_path, 'rb')
    if wf.getframerate() != 16000 or wf.getnchannels() != 1:
        print(f'[WARN] 音频格式异常: {wf.getframerate()}Hz, {wf.getnchannels()}ch')
    rec = KaldiRecognizer(model, 16000)
    text = ''
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            res = json.loads(rec.Result())
            text += res.get('text', '')
    res = json.loads(rec.FinalResult())
    text += res.get('text', '')
    wf.close()
    return text.strip()


def main():
    parser = argparse.ArgumentParser(description='最小化 ASR 测试')
    parser.add_argument('--mic', default='plughw:1,0', help='麦克风 ALSA 设备')
    parser.add_argument('--model', default='/app/puppy_ws/models/vosk-model-small-cn-0.22',
                        help='Vosk 模型路径')
    parser.add_argument('--sec', type=int, default=4, help='每次录音秒数')
    parser.add_argument('--save', default='/tmp/asr_test', help='保存录音的目录')
    args = parser.parse_args()

    os.makedirs(args.save, exist_ok=True)

    print('=' * 60)
    print(' 最小化 ASR 识别测试')
    print(f' 麦克风: {args.mic}')
    print(f' 模型:   {args.model}')
    print(f' 录音时长: {args.sec} 秒/次')
    print(f' 录音保存: {args.save}/')
    print('=' * 60)
    print('测试指令：坐下 / 站起来 / 前进 / 左转 / 右转 / 停下')
    print('按 Ctrl+C 退出\n')

    # 检查模型
    if not os.path.exists(args.model):
        print(f'[ERROR] Vosk 模型不存在: {args.model}')
        sys.exit(1)

    print('加载 Vosk 模型...')
    model = Model(args.model)
    print('模型加载完成，开始测试\n')

    test_words = ['坐下', '站起来', '前进', '左转', '右转', '停下', '小狗']
    round_num = 0

    while True:
        round_num += 1
        wav_path = os.path.join(args.save, f'rec_{round_num:03d}.wav')

        print(f'\n--- 第 {round_num} 轮 ---')
        print(f'[录音] {args.sec} 秒... 请说话！')
        record_audio(args.mic, args.sec, wav_path)

        # 识别
        text = asr_recognize(model, wav_path)

        if not text:
            print('  识别结果: (空) - 没听到或没识别到')
            # 播放录音检查麦克风
            print(f'  录音已保存: {wav_path} (可用 aplay -D plughw:0,0 {wav_path} 回放检查)')
        else:
            print(f'  识别结果: "{text}"')
            # 检查是否匹配测试词
            matched = [w for w in test_words if w in text]
            if matched:
                print(f'  ✅ 匹配指令: {matched}')
            else:
                print(f'  ❌ 未匹配指令词')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n退出测试')
