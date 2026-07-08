#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯 Python 版语音控制（不依赖 ROS2）
=====================================
VAD 自动断句 + Vosk 识别 + 模糊匹配 → UDP 发动作给 sit.py + sherpa TTS 反馈

特点：
  - 不需要 ROS2，不需要 source setup.bash，直接 python3 运行
  - TTS 播放时天然不录音（同步阻塞），防自激
  - 招手(wave)有具体动作文件 wave.d6ac，确保兜底

用法：
  python3 voice_control_standalone.py \
      --mic plughw:1,0 \
      --speaker plughw:0,0 \
      --denoise \
      --gain 10 \
      --aggressiveness 2 \
      --silence 1.0

  python3 voice_control_standalone.py --no-tts   # 关闭 TTS，只测 ASR+控制

前置：
  - sit.py 已启动（监听 UDP 5005）
  - Vosk 模型: /app/puppy_ws/models/vosk-model-small-cn-0.22
  - sherpa TTS 模型: /opt/sherpa-models/（--no-tts 可跳过）
  - 依赖: vosk, webrtcvad, sox, sherpa_onnx, soundfile, numpy
"""
import argparse
import json
import os
import socket
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
    print('[WARN] 未装 webrtcvad，回退固定录音模式: pip3 install webrtcvad')


# ============ 指令映射表 ============
# key   = 发给 sit.py 的 UDP 动作名（纯字符串）
# words = Vosk 可能识别出的同义词/近似词
# tts   = TTS 反馈语
COMMAND_MAP = {
    'forward':   {'words': ['前进', '向前走', '往前走', '走', '向前', '直走',
                            '千进', '欠进', '前静'],
                  'tts': '好的，前进'},
    'backward':  {'words': ['后退', '向后走', '往后走', '倒车', '向后',
                            '后腿', '后退走'],
                  'tts': '好的，后退'},
    'turn_left': {'words': ['左转', '向左转', '往左转', '左边走', '左拐',
                            '左钻', '左赚'],
                  'tts': '好的，左转'},
    'turn_right': {'words': ['右转', '向右转', '往右转', '右边走', '右拐',
                             '右钻', '右赚'],
                   'tts': '好的，右转'},
    'stand':     {'words': ['站起来', '站立', '站起', '站', '站啦',
                            '起立', '起来'],
                  'tts': '好的，站起来'},
    'sit':       {'words': ['坐下', '坐下来', '坐', '座下', '做下',
                            '请坐'],
                  'tts': '好的，坐下'},
    'crouch':    {'words': ['趴下', '趴着', '蹲下', '蹲着', '趴', '他下',
                            '塔下', '踏下', '卧倒', '卧', '到下'],
                  'tts': '好的，趴下'},
    'wave':      {'words': ['摇摆', '摇一摇', '摇摇', '招手', '挥手',
                            '挥挥手', '打招呼'],
                  'tts': '好的，摇摆'},
    'stop':      {'words': ['停下', '停止', '别动', '不要动', '停',
                            '挺下', '站住'],
                  'tts': '好的，停止'},
}


def fuzzy_match(text: str, threshold: float = 0.6):
    """匹配指令，返回 (动作名, 置信度) 或 (None, 最高分)"""
    if not text:
        return None, 0.0
    compact = text.replace(' ', '').strip()
    if not compact:
        return None, 0.0

    best_match = None
    best_score = 0.0

    for cmd, info in COMMAND_MAP.items():
        for syn in info['words']:
            if syn in compact:
                return cmd, 1.0
            score = SequenceMatcher(None, syn, compact).ratio()
            if score > best_score:
                best_score = score
                best_match = cmd

    if best_score >= threshold:
        return best_match, best_score
    return None, best_score


# ============ VAD 录音（复用 asr_test_vad 逻辑） ============
def record_with_vad(device, wav_path,
                    max_wait=8.0, max_speech=5.0,
                    silence_dur=0.8, aggressiveness=2,
                    gain_db=10, noise_prof=None):
    if not HAS_VAD:
        raw = wav_path.replace('.wav', '_raw.wav')
        subprocess.run(['arecord', '-D', device, '-d', '4',
                        '-f', 'S16_LE', '-r', '16000', '-c', '1', raw],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        _postprocess(raw, wav_path, gain_db, noise_prof)
        return 'fixed-4s'

    vad = webrtcvad.Vad(aggressiveness)
    sample_rate = 16000
    frame_duration = 30
    frame_size = int(sample_rate * frame_duration / 1000) * 2

    proc = subprocess.Popen(
        ['arecord', '-D', device,
         '-f', 'S16_LE', '-r', str(sample_rate), '-c', '1',
         '-t', 'raw', '-q'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )

    raw_path = wav_path.replace('.wav', '_raw.wav')
    raw_frames = []
    state = 'waiting'
    wait_start = time.time()
    speech_start = 0
    silence_start = 0
    total_speech = 0
    speech_frame_count = 0
    min_speech_frames = 6
    silence_frame_count = 0
    min_silence_frames = int(silence_dur * 1000 / frame_duration)

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
                else:
                    speech_frame_count = 0
                    raw_frames = []
                    if now - wait_start > max_wait:
                        break
            elif state == 'speaking':
                raw_frames.append(frame)
                if is_speech:
                    silence_frame_count = 0
                    silence_start = now
                    total_speech = now - speech_start
                    if total_speech > max_speech:
                        break
                else:
                    silence_frame_count += 1
                    if silence_frame_count >= min_silence_frames:
                        break
    finally:
        proc.terminate()
        proc.wait()

    if not raw_frames:
        with wave.open(wav_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
        return 'empty'

    with wave.open(raw_path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(raw_frames))

    _postprocess(raw_path, wav_path, gain_db, noise_prof)
    return 'vad'


def _postprocess(raw_path, wav_path, gain_db, noise_prof):
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


def asr_recognize(model, wav_path):
    wf = wave.open(wav_path, 'rb')
    rec = KaldiRecognizer(model, 16000)
    rec.SetWords(True)
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


# ============ TTS 引擎（sherpa Vocos 新 API） ============
class TtsEngine:
    """sherpa-onnx Matcha + Vocos 离线 TTS"""

    def __init__(self, speaker_device, cache_dir='/tmp/tts_cache',
                 model_root='/opt/sherpa-models'):
        self.device = speaker_device
        self.cache_dir = cache_dir
        os.makedirs(cache_dir, exist_ok=True)

        import sherpa_onnx
        import numpy as np
        import soundfile as sf
        self._np = np
        self._sf = sf

        matcha_dir = os.path.join(model_root, 'matcha-zh-baker')
        vocoder_dir = os.path.join(model_root, 'vocoder')

        # 自动找模型文件
        acoustic_model = self._find_file(matcha_dir,
            ['model-steps-3.onnx', 'matcha-zh-baker.onnx', 'model.onnx'])
        vocoder = self._find_file(vocoder_dir,
            ['vocos-22khz-univ.onnx', 'hifigan_v2.onnx', 'vocoder.onnx'])
        lexicon = os.path.join(matcha_dir, 'lexicon.txt')
        tokens = os.path.join(matcha_dir, 'tokens.txt')

        if not acoustic_model or not vocoder:
            raise FileNotFoundError(
                f'TTS 模型不完整: acoustic={acoustic_model}, vocoder={vocoder}'
            )

        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=acoustic_model,
                    vocoder=vocoder,
                    lexicon=lexicon,
                    tokens=tokens,
                ),
            ),
        )
        self.tts = sherpa_onnx.OfflineTts(config)
        print(f'[TTS] 初始化成功: {acoustic_model}')
        print(f'[TTS] Vocoder: {vocoder}')

    @staticmethod
    def _find_file(directory, candidates):
        for name in candidates:
            path = os.path.join(directory, name)
            if os.path.exists(path):
                return path
        return None

    def speak(self, text):
        """合成 + 播放（阻塞，天然防自激）"""
        import hashlib
        h = hashlib.md5(text.encode('utf-8')).hexdigest()[:16]
        wav_path = os.path.join(self.cache_dir, f'{h}.wav')

        if not os.path.exists(wav_path):
            audio = self.tts.generate(text)
            samples = self._np.array(audio.samples, dtype=self._np.float32)
            self._sf.write(wav_path, samples, audio.sample_rate,
                           subtype='FLOAT')

        subprocess.run(
            ['aplay', '-D', self.device, wav_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        print(f'  [TTS] {text}')


# ============ UDP 动作发送 ============
def send_action(action_name, ip='127.0.0.1', port=5005):
    """发纯动作名字符串给 sit.py"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(action_name.encode('utf-8'), (ip, port))
    sock.close()
    print(f'  [UDP] → {action_name}  (→ {ip}:{port})')


# ============ 主循环 ============
def main():
    parser = argparse.ArgumentParser(
        description='纯 Python 版语音控制（VAD+Vosk+UDP+TTS）'
    )
    parser.add_argument('--mic', default='plughw:1,0', help='麦克风设备')
    parser.add_argument('--speaker', default='plughw:0,0', help='音响设备')
    parser.add_argument('--model',
                        default='/app/puppy_ws/models/vosk-model-small-cn-0.22')
    parser.add_argument('--save', default='/tmp/voice_control')
    parser.add_argument('--denoise', action='store_true')
    parser.add_argument('--noise-prof', default='/tmp/noise.prof')
    parser.add_argument('--gain', type=int, default=10)
    parser.add_argument('--threshold', type=float, default=0.6)
    parser.add_argument('--aggressiveness', type=int, default=2)
    parser.add_argument('--silence', type=float, default=1.0)
    parser.add_argument('--max-wait', type=float, default=8.0)
    parser.add_argument('--udp-ip', default='127.0.0.1')
    parser.add_argument('--udp-port', type=int, default=5005)
    parser.add_argument('--no-tts', action='store_true', help='关闭 TTS 反馈')
    parser.add_argument('--tts-root', default='/opt/sherpa-models')
    args = parser.parse_args()

    os.makedirs(args.save, exist_ok=True)

    # 打印配置
    print('=' * 60)
    print(' 纯 Python 版语音控制')
    print(f' 麦克风:   {args.mic}')
    print(f' 音响:     {args.speaker}')
    print(f' Vosk模型: {args.model}')
    print(f' 增益:     {args.gain}dB' +
          (' + 降噪' if args.denoise else '') +
          (' + VAD' if HAS_VAD else ' + 固定4秒'))
    print(f' UDP:      {args.udp_ip}:{args.udp_port} (sit.py)')
    print(f' TTS:      {"关闭" if args.no_tts else "开启"}')
    print('=' * 60)
    print('支持指令:')
    for cmd, info in COMMAND_MAP.items():
        print(f'  {cmd:10s} ← {", ".join(info["words"][:4])}...')
    print('=' * 60)
    print('按 Ctrl+C 退出\n')

    # 加载 Vosk 模型
    print('加载 Vosk 模型...')
    model = Model(args.model)
    print('加载完成\n')

    # 初始化 TTS（可选）
    tts = None
    if not args.no_tts:
        try:
            tts = TtsEngine(args.speaker, model_root=args.tts_root)
        except Exception as e:
            print(f'[WARN] TTS 初始化失败，将无语音反馈: {e}')
            print('       可加 --no-tts 跳过 TTS')

    # 降噪检查
    if args.denoise and not os.path.exists(args.noise_prof):
        print(f'[WARN] 噪声样本不存在: {args.noise_prof}，不降噪')
        args.denoise = False

    # 主循环
    round_num = 0
    while True:
        round_num += 1
        wav_path = os.path.join(args.save, f'rec_{round_num:03d}.wav')

        print(f'\n--- 第 {round_num} 轮 ---')
        print('[听...] 随时说话', flush=True)

        mode = record_with_vad(
            args.mic, wav_path,
            max_wait=args.max_wait,
            silence_dur=args.silence,
            aggressiveness=args.aggressiveness,
            gain_db=args.gain,
            noise_prof=(args.noise_prof if args.denoise else None)
        )

        if mode == 'empty':
            print('  (没听到声音)')
            continue

        text = asr_recognize(model, wav_path)
        if not text:
            print('  识别结果: (空)')
            continue

        print(f'  识别结果: "{text}"')

        matched, score = fuzzy_match(text, args.threshold)
        if not matched:
            print(f'  未匹配指令  (最高相似度: {score:.2f})')
            continue

        print(f'  ✅ [{matched}]  (相似度: {score:.2f})')

        # 1. 发 UDP 动作给 sit.py
        send_action(matched, args.udp_ip, args.udp_port)

        # 2. TTS 反馈（阻塞播放，天然防自激）
        if tts:
            tts.speak(COMMAND_MAP[matched]['tts'])


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n退出')
