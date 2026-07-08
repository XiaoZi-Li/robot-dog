#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版 ASR 测试脚本：录音 + 降噪 + Vosk 识别 + 模糊匹配。
不发 UDP，不播报，不依赖 sit.py。
增强点：
1. sox 降噪 + 增益（可选，需要 sox）
2. 模糊匹配指令词（容忍错别字）
3. 显示置信度
4. 支持大模型
"""
import argparse
import json
import os
import subprocess
import sys
import wave
from difflib import SequenceMatcher

from vosk import Model, KaldiRecognizer


# ===== 指令词库（含常见错别字）=====
COMMAND_SYNONYMS = {
    '坐下': ['坐下', '坐下来', '坐', '座下', '做下'],
    '站起来': ['站起来', '站立', '站起', '站', '站啦', '站起来'],
    '前进': ['前进', '向前走', '往前走', '前进走', '千进', '前进', '欠进'],
    '左转': ['左转', '向左转', '左拐', '左', '左钻', '左赚'],
    '右转': ['右转', '向右转', '右拐', '右', '右钻', '右赚'],
    '停下': ['停下', '停止', '别动', '停', '停顿', '挺下'],
    '小狗': ['小狗', '小狗儿', '小购', '小狗'],
    '后退': ['后退', '向后走', '往后走', '后腿'],
}


def fuzzy_match(text: str, threshold: float = 0.6):
    """模糊匹配指令词，返回 (matched_command, similarity)"""
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


def record_audio(device: str, seconds: int, wav_path: str,
                 denoise: bool = False, noise_prof: str = None,
                 gain_db: int = 6):
    """录音，可选降噪+增益"""
    raw_path = wav_path.replace('.wav', '_raw.wav')
    cmd = [
        'arecord', '-D', device,
        '-d', str(seconds),
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        raw_path
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if denoise and noise_prof and os.path.exists(noise_prof):
        # sox 降噪 + 增益
        sox_cmd = ['sox', raw_path, wav_path,
                   'noisered', noise_prof, '0.2',
                   'gain', str(gain_db)]
        subprocess.run(sox_cmd, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        os.remove(raw_path)
    elif gain_db != 0:
        sox_cmd = ['sox', raw_path, wav_path, 'gain', str(gain_db)]
        subprocess.run(sox_cmd, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        os.remove(raw_path)
    else:
        os.rename(raw_path, wav_path)


def asr_recognize(model: Model, wav_path: str):
    """Vosk 识别，返回 (text, confidence)"""
    wf = wave.open(wav_path, 'rb')
    rec = KaldiRecognizer(model, 16000)
    rec.SetWords(True)  # 开启词级置信度
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
    parser = argparse.ArgumentParser(description='增强版 ASR 测试')
    parser.add_argument('--mic', default='plughw:1,0')
    parser.add_argument('--model',
                        default='/app/puppy_ws/models/vosk-model-small-cn-0.22')
    parser.add_argument('--big-model',
                        default='/app/puppy_ws/models/vosk-model-cn-0.22',
                        help='大模型路径（存在就用）')
    parser.add_argument('--sec', type=int, default=4)
    parser.add_argument('--save', default='/tmp/asr_test')
    parser.add_argument('--denoise', action='store_true', help='开启 sox 降噪')
    parser.add_argument('--noise-prof', default='/tmp/noise.prof',
                        help='噪声样本文件')
    parser.add_argument('--gain', type=int, default=6, help='增益 dB（默认6）')
    parser.add_argument('--threshold', type=float, default=0.6,
                        help='模糊匹配阈值')
    args = parser.parse_args()

    os.makedirs(args.save, exist_ok=True)

    # 优先用大模型
    model_path = args.model
    if os.path.exists(args.big_model):
        model_path = args.big_model
        print(f'[INFO] 检测到大模型，使用: {model_path}')
    else:
        print(f'[INFO] 使用小模型: {model_path}')
        print(f'[提示] 大模型准确率更高，下载: '
              f'https://alphacephei.com/vosk/models/vosk-model-cn-0.22.zip')

    if args.denoise:
        if not os.path.exists(args.noise_prof):
            print(f'[WARN] 噪声样本不存在: {args.noise_prof}')
            print('  生成方法: arecord -D plughw:1,0 -d 1 /tmp/noise.wav && '
                  'sox /tmp/noise.wav -n noiseprof /tmp/noise.prof')
            print('  本次不降噪')
            args.denoise = False
        else:
            print(f'[INFO] 开启降噪，噪声样本: {args.noise_prof}')

    print(f'[INFO] 增益: {args.gain}dB')

    print('=' * 60)
    print(' 增强版 ASR 识别测试')
    print(f' 麦克风: {args.mic}')
    print(f' 模型:   {model_path}')
    print(f' 录音:   {args.sec}秒/次' +
          (f' + 降噪' if args.denoise else '') +
          f' + {args.gain}dB增益')
    print(f' 模糊匹配阈值: {args.threshold}')
    print('=' * 60)
    print('测试词：坐下 / 站起来 / 前进 / 左转 / 右转 / 停下 / 小狗')
    print('按 Ctrl+C 退出\n')

    print('加载模型...')
    model = Model(model_path)
    print('加载完成，开始测试\n')

    round_num = 0
    correct = 0
    total = 0

    while True:
        round_num += 1
        wav_path = os.path.join(args.save, f'rec_{round_num:03d}.wav')

        print(f'\n--- 第 {round_num} 轮 ---')
        print(f'[录音] {args.sec} 秒... 请说话！')
        record_audio(args.mic, args.sec, wav_path,
                     args.denoise, args.noise_prof, args.gain)

        text, conf = asr_recognize(model, wav_path)

        if not text:
            print('  识别结果: (空) - 没听到')
            print(f'  录音: {wav_path} (aplay -D plughw:0,0 {wav_path})')
        else:
            print(f'  识别结果: "{text}"  (置信度: {conf:.2f})')
            matched, score = fuzzy_match(text, args.threshold)
            total += 1
            if matched:
                correct += 1
                print(f'  ✅ 匹配指令: [{matched}]  (相似度: {score:.2f})')
            else:
                print(f'  ❌ 未匹配  (最高相似度: {score:.2f})')
                print(f'     录音: {wav_path}')

        if total > 0:
            print(f'\n[统计] 已测 {total} 次, 匹配 {correct} 次, '
                  f'准确率 {correct/total*100:.0f}%')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n退出测试')
