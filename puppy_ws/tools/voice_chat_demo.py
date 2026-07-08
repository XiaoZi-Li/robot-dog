#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中文语音对话 demo（独立运行，不依赖 ROS2）

验证全链路：麦克风录音 → Vosk ASR → 控制指令分流 → TTS 播报
LLM 部分用 echo 兜底（说"你叫什么"→ 回"我还没接 LLM，但你问的是：xxx"）
真接 LLM 时让 chat_with_llm() 返回 Qwen2.5 的回复即可。

用法：
  python3 voice_chat_demo.py
  python3 voice_chat_demo.py --mic plughw:2,0 --speaker plughw:1,0
  python3 voice_chat_demo.py --tts edge          # 强制走 edge-tts（联网）
  python3 voice_chat_demo.py --tts sherpa         # 强制走 sherpa 离线
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import wave
from difflib import SequenceMatcher

import audioop

# ---------- Vosk ASR ----------
try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    print('[ERROR] pip3 install vosk')
    sys.exit(1)

# ---------- VAD ----------
try:
    import webrtcvad
    HAS_VAD = True
except ImportError:
    HAS_VAD = False

# ---------- TTS ----------
try:
    import sherpa_onnx
    import numpy as np
    import soundfile as sf
    HAS_SHERPA = True
except ImportError:
    HAS_SHERPA = False

try:
    import edge_tts
    import asyncio
    HAS_EDGE = True
except ImportError:
    HAS_EDGE = False


# ---------- 控制指令关键词 ----------
CONTROL_PHRASES = {
    '坐下': 'sit', '坐下来': 'sit', '坐下吧': 'sit',
    '站立': 'stand', '站起来': 'stand', '起来': 'stand', '站立起来': 'stand',
    '停下': 'stop', '停止': 'stop', '别动': 'stop', '不要动': 'stop', '停': 'stop',
    '前进': 'forward', '向前走': 'forward', '往前走': 'forward', '直走': 'forward',
    '后退': 'backward', '向后走': 'backward', '倒车': 'backward',
    '左转': 'turn_left', '向左转': 'turn_left', '左拐': 'turn_left',
    '右转': 'turn_right', '向右转': 'turn_right', '右拐': 'turn_right',
}

WAKEUP_KEYWORDS = ['小狗', '小狗狗', '晓狗', '小够', '小苟', '小古']


# ---------- 录音（VAD 版，自动检测说话开始/结束） ----------
def record_wav(device: str, wav_path: str, seconds: int, rate: int = 16000,
               use_vad: bool = True, vad_aggressiveness: int = 2,
               silence_dur: float = 1.0, gain_db: int = 0,
               noise_prof: str = None):
    """录音：优先用 VAD 自动检测说话，回退到固定秒数"""
    if use_vad and HAS_VAD:
        return _record_with_vad(device, wav_path, rate, vad_aggressiveness,
                                silence_dur, gain_db, noise_prof)
    # 回退：固定秒数录音
    raw = wav_path + '.raw.wav'
    subprocess.run(
        ['arecord', '-D', device, '-d', str(seconds),
         '-f', 'S16_LE', '-r', str(rate), '-c', '1', '-t', 'wav', raw],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    _postprocess_audio(raw, wav_path, gain_db, noise_prof)


def _record_with_vad(device: str, wav_path: str, sample_rate: int,
                     aggressiveness: int, silence_dur: float,
                     gain_db: int, noise_prof: str):
    """VAD 录音：需要连续 6 帧才算说话开始"""
    vad = webrtcvad.Vad(aggressiveness)
    frame_duration = 30  # ms
    frame_size = int(sample_rate * frame_duration / 1000) * 2

    proc = subprocess.Popen(
        ['arecord', '-D', device,
         '-f', 'S16_LE', '-r', str(sample_rate), '-c', '1',
         '-t', 'raw', '-q'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )

    raw_path = wav_path + '.raw.wav'
    raw_frames = []
    state = 'waiting'
    wait_start = time.time()
    max_wait = 8.0
    speech_start = 0
    silence_start = 0
    total_speech = 0

    speech_frame_count = 0
    min_speech_frames = 6  # 连续 6 帧 ≈ 180ms
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
                        print('[VAD] 说话开始', end='', flush=True)
                else:
                    speech_frame_count = 0
                    raw_frames = []
                    if now - wait_start > max_wait:
                        print('[VAD] 超时', end='', flush=True)
                        break

            elif state == 'speaking':
                raw_frames.append(frame)
                if is_speech:
                    silence_frame_count = 0
                    silence_start = now
                    total_speech = now - speech_start
                    if total_speech > 5.0:
                        print(' → 超时', end='', flush=True)
                        break
                else:
                    silence_frame_count += 1
                    if silence_frame_count >= min_silence_frames:
                        print(f' → 结束({total_speech:.1f}s)', end='', flush=True)
                        break
    finally:
        proc.terminate()
        proc.wait()

    if not raw_frames:
        with wave.open(wav_path, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
        return

    with wave.open(raw_path, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(raw_frames))

    _postprocess_audio(raw_path, wav_path, gain_db, noise_prof)


def _postprocess_audio(raw_path, wav_path, gain_db, noise_prof):
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


def wav_to_text(model: Model, wav_path: str) -> str:
    """Vosk 识别 wav → 文本"""
    wf = wave.open(wav_path, 'rb')
    if wf.getframerate() != 16000:
        # 重采样
        pcm = wf.readframes(wf.getnframes())
        pcm, _ = audioop.ratecv(pcm, 2, 1, wf.getframerate(), 16000, None)
        wf.close()
        # 写临时文件
        tmp = wav_path + '.16k.wav'
        with wave.open(tmp, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(16000)
            w.writeframes(pcm)
        wf = wave.open(tmp, 'rb')
        tmp_path = tmp
    else:
        tmp_path = None

    rec = KaldiRecognizer(model, 16000)
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        rec.AcceptWaveform(data)
    result = json.loads(rec.FinalResult())
    wf.close()
    if tmp_path:
        os.remove(tmp_path)
    return result.get('text', '').strip()


# ---------- TTS ----------
class SherpaTts:
    def __init__(self, model_root='/opt/sherpa-models'):
        tts_dir = os.path.join(model_root, 'matcha-zh-baker')
        vocoder_dir = os.path.join(model_root, 'vocoder')

        # matcha 模型文件名候选（官方文档确认主文件名是 model-steps-3.onnx）
        model_candidates = [
            os.path.join(tts_dir, 'model-steps-3.onnx'),
            os.path.join(tts_dir, 'matcha-zh-baker.onnx'),
            os.path.join(tts_dir, 'model.onnx'),
        ]
        model_path = next((p for p in model_candidates if os.path.exists(p)), None)

        # vocoder 文件名候选（官方推荐 vocos-22khz-univ.onnx）
        vocoder_candidates = [
            os.path.join(vocoder_dir, 'vocos-22khz-univ.onnx'),
            os.path.join(vocoder_dir, 'hifigan_v2.onnx'),
            os.path.join(vocoder_dir, 'hifigan_v1.onnx'),
        ]
        vocoder_path = next((p for p in vocoder_candidates if os.path.exists(p)), None)

        if not model_path:
            raise FileNotFoundError(f'Matcha 模型不存在: {tts_dir}/model-steps-3.onnx，请运行 setup_sherpa.sh')
        if not vocoder_path:
            raise FileNotFoundError(f'Vocoder 不存在: {vocoder_dir}/vocos-22khz-univ.onnx')

        tokens_path = os.path.join(tts_dir, 'tokens.txt')
        lexicon_path = os.path.join(tts_dir, 'lexicon.txt')
        lexicon_val = lexicon_path if os.path.exists(lexicon_path) else ''

        # 新版 sherpa-onnx (1.10+) API: 需要嵌套 config 对象
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=model_path,
                    vocoder=vocoder_path,
                    lexicon=lexicon_val,
                    tokens=tokens_path,
                ),
            ),
        )
        self.tts = sherpa_onnx.OfflineTts(tts_config)
        print(f'[TTS] sherpa Matcha zh-baker 加载成功（model={os.path.basename(model_path)}, vocoder={os.path.basename(vocoder_path)}）')

    def speak(self, text: str, device: str, volume_db='+0'):
        audio = self.tts.generate(text)
        samples = np.array(audio.samples, dtype=np.float32)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            raw = f.name
        out = raw.replace('.wav', '.out.wav')
        sf.write(raw, samples, audio.sample_rate)
        # 用 ffmpeg 调音量
        subprocess.run(
            ['ffmpeg', '-y', '-i', raw,
             '-af', f'volume={volume_db}dB',
             '-ar', '44100', '-ac', '1', out],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ['aplay', '-D', device, out],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        os.remove(raw)
        os.remove(out)


class EdgeTts:
    def __init__(self, voice='zh-CN-XiaoxiaoNeural'):
        self.voice = voice
        print(f'[TTS] edge-tts 加载成功（联网），voice={voice}')

    def speak(self, text: str, device: str, volume_db='+0'):
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            mp3_path = f.name
        wav_path = mp3_path.replace('.mp3', '.wav')
        asyncio.run(self._save(text, mp3_path))
        subprocess.run(
            ['ffmpeg', '-y', '-i', mp3_path,
             '-af', f'volume={volume_db}dB',
             '-ar', '44100', '-ac', '1', wav_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ['aplay', '-D', device, wav_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        os.remove(mp3_path)
        os.remove(wav_path)

    async def _save(self, text, mp3_path):
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(mp3_path)


def init_tts(backend: str):
    if backend in ('sherpa', 'auto'):
        if HAS_SHERPA:
            try:
                return SherpaTts()
            except Exception as e:
                print(f'[WARN] sherpa 初始化失败: {e}')
                if backend == 'sherpa':
                    return None
    if backend in ('edge', 'auto'):
        if HAS_EDGE:
            return EdgeTts()
        print('[ERROR] edge-tts 未安装：pip3 install edge-tts')
    return None


# ---------- LLM 占位 ----------
def chat_with_llm(text: str) -> str:
    """LLM 回复占位。真接 Qwen2.5 时改这里。

    最简接法（不依赖 ROS2）：
      result = subprocess.run(
          ['llama-cli', '-m', '/app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf',
           '-p', f'用户：{text}\n助手：', '-n', '128'],
          capture_output=True, text=True, timeout=30
      )
      return result.stdout.strip()
    """
    return f'你刚才说的是：{text}。我还没接大模型，等会儿接上 Qwen 就能正常回答了。'


import socket as _socket


# ---------- UDP 直发 sit.py（绕过 ROS2，最稳定） ----------
# sit.py 监听 127.0.0.1:5005，接收 {"action":"walk"} 等 JSON
# voice_chat_demo command -> sit.py action 映射
SIT_PY_ACTION_MAP = {
    'sit':        'sit',
    'stand':      'stand',
    'stop':       'stop',
    'forward':    'walk',
    'turn_left':  'turn_left',
    'turn_right': 'turn_right',
    # backward: sit.py 无对应动作，忽略
}

# 持续型动作（需要定时自动停）：walk / turn_left / turn_right
CONTINUOUS_ACTIONS = {'walk', 'turn_left', 'turn_right'}


def send_udp_action(action: str, udp_ip='127.0.0.1', udp_port=5005):
    """直接发 UDP 指令到 sit.py"""
    payload = json.dumps({'action': action, 'source': 'voice_demo'}, ensure_ascii=False)
    try:
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        sock.sendto(payload.encode('utf-8'), (udp_ip, udp_port))
        sock.close()
        print(f'  -> UDP 发送: {action} -> {udp_ip}:{udp_port}')
    except Exception as e:
        print(f'  -> [WARN] UDP 发送失败: {e}')


def execute_puppy_command(command: str, move_sec=2.5):
    """根据 voice command 执行 sit.py 对应动作"""
    sit_action = SIT_PY_ACTION_MAP.get(command)
    if sit_action is None:
        print(f'  -> [WARN] sit.py 不支持该动作: {command}')
        return

    # 离散动作（sit/stand/stop）：发一次即可
    if sit_action in ('sit', 'stand', 'stop'):
        send_udp_action(sit_action)
        return

    # 持续动作（walk/turn_left/turn_right）：发动作 → 等 move_sec 秒 → 发 stop
    send_udp_action(sit_action)
    print(f'  -> 持续 {move_sec} 秒...')
    time.sleep(move_sec)
    send_udp_action('stop')


# ---------- 主循环 ----------
def send_voice_command(command: str):
    """保留 ROS2 接口（--ros-control 用），默认走 UDP 直发"""
    payload = json.dumps({
        'source': 'voice',
        'sub_source': 'voice_chat_demo',
        'command': command,
        'timestamp': time.time(),
    }, ensure_ascii=False)
    try:
        subprocess.run(
            ['ros2', 'topic', 'pub', '--once',
             '/voice/result_json', 'std_msgs/String',
             f"{{data: '{payload}'}}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=5
        )
        print(f'  -> 已发送到 /voice/result_json: {command}')
    except Exception as e:
        print(f'  -> [WARN] 发送 ROS2 指令失败: {e}')


# ---------- 主循环 ----------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mic', default='plughw:2,0', help='麦克风 ALSA 设备')
    parser.add_argument('--speaker', default='plughw:1,0', help='音响 ALSA 设备')
    parser.add_argument('--vosk-model', default='/app/puppy_ws/models/vosk-model-small-cn-0.22')
    parser.add_argument('--tts', choices=['auto', 'sherpa', 'edge'], default='auto')
    parser.add_argument('--record-sec', type=int, default=4)
    parser.add_argument('--no-wakeup', action='store_true', help='关闭唤醒词，直接识别')
    parser.add_argument('--volume', default='-5', help='TTS 音量(dB)，-5 较小，-10 很小，+0 原始')
    parser.add_argument('--udp-control', action='store_true', default=True,
                        help='识别到控制指令时直接发 UDP 到 sit.py(127.0.0.1:5005)，默认开启')
    parser.add_argument('--no-udp-control', dest='udp_control', action='store_false',
                        help='关闭 UDP 控制（只播报不动）')
    parser.add_argument('--ros-control', action='store_true',
                        help='改用 ROS2 /voice/result_json 控制机器狗（需 decision_node 运行）')
    parser.add_argument('--move-sec', type=float, default=2.5,
                        help='前进/左转/右转持续秒数，默认 2.5')
    parser.add_argument('--udp-ip', default='127.0.0.1', help='sit.py UDP 地址')
    parser.add_argument('--udp-port', type=int, default=5005, help='sit.py UDP 端口')
    # VAD / 音频增强参数
    parser.add_argument('--no-vad', action='store_true', help='关闭 VAD，用固定秒数录音')
    parser.add_argument('--vad-aggressiveness', type=int, default=2,
                        help='VAD 灵敏度 0-3（3最灵敏但易误触发）')
    parser.add_argument('--silence', type=float, default=1.0,
                        help='说话后静音多少秒判定结束')
    parser.add_argument('--gain', type=int, default=10, help='录音增益 dB（默认10）')
    parser.add_argument('--denoise', action='store_true', help='开启 sox 降噪')
    parser.add_argument('--noise-prof', default='/tmp/noise.prof',
                        help='噪声样本文件路径')
    args = parser.parse_args()

    print('=' * 60)
    print(' 中文语音对话 demo（VAD 增强版）')
    print(f' 麦克风: {args.mic}')
    print(f' 音响:   {args.speaker}')
    print(f' TTS:    {args.tts}')
    print(f' 音量:   {args.volume}dB')
    print(f' 录音:   {"VAD自动检测" if (HAS_VAD and not args.no_vad) else f"固定{args.record_sec}秒"}' +
          (f' + {args.gain}dB增益' if args.gain else '') +
          (' + 降噪' if args.denoise else ''))
    print(f' 唤醒词: {"关闭" if args.no_wakeup else "小狗"}')
    if args.ros_control:
        print(f' 机器狗控制: ROS2 (/voice/result_json)')
    elif args.udp_control:
        print(f' 机器狗控制: UDP 直发 sit.py ({args.udp_ip}:{args.udp_port})')
        print(f' 移动持续: {args.move_sec} 秒')
    else:
        print(f' 机器狗控制: 关闭（只播报）')
    print('=' * 60)

    # 初始化 Vosk
    if not os.path.exists(args.vosk_model):
        print(f'[ERROR] Vosk 模型不存在: {args.vosk_model}')
        return
    print('[ASR] 加载 Vosk 模型...', end=' ', flush=True)
    asr_model = Model(args.vosk_model)
    print('OK')

    # 初始化 TTS
    tts = init_tts(args.tts)
    if tts is None:
        print('[ERROR] 没有 TTS 后端可用，退出')
        return

    # 开场白
    print('\n>>> 说 "小狗" 唤醒（或加 --no-wakeup 跳过）')
    if tts:
        tts.speak('你好，我是机器狗，请说小狗唤醒我。', args.speaker, args.volume)

    wakeup_mode = not args.no_wakeup
    use_vad = HAS_VAD and not args.no_vad
    noise_prof = args.noise_prof if args.denoise else None

    while True:
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                wav_path = f.name

            if use_vad:
                print('\n[请说话] 随时开始...', end=' ', flush=True)
            else:
                print(f'\n[录音] {args.record_sec} 秒...', end=' ', flush=True)
            record_wav(args.mic, wav_path, args.record_sec,
                       use_vad=use_vad,
                       vad_aggressiveness=args.vad_aggressiveness,
                       silence_dur=args.silence,
                       gain_db=args.gain,
                       noise_prof=noise_prof)
            text = wav_to_text(asr_model, wav_path)
            os.remove(wav_path)

            if not text:
                print('（没听到）')
                continue
            print(f'识别: "{text}"')

            # 唤醒词检测
            if wakeup_mode:
                if any(kw in text for kw in WAKEUP_KEYWORDS):
                    print('  -> 唤醒成功！')
                    if tts:
                        tts.speak('我在，请说。', args.speaker, args.volume)
                    wakeup_mode = False
                    continue
                else:
                    print('  -> 未唤醒，忽略（说"小狗"唤醒）')
                    continue

            # 控制指令
            matched = None
            for phrase, cmd in CONTROL_PHRASES.items():
                if phrase in text:
                    matched = (phrase, cmd)
                    break

            if matched:
                phrase, cmd = matched
                print(f'  -> 控制指令: {cmd}')

                responses = {
                    'sit': '好的，我坐下了',
                    'stand': '好的，我站起来了',
                    'stop': '好的，我停下了',
                    'forward': '好的，我前进',
                    'backward': '好的，我后退',
                    'turn_left': '好的，我左转',
                    'turn_right': '好的，我右转',
                }
                reply_text = responses.get(cmd, '好的')

                # 持续动作（前进/左转/右转）：先播报，再移动
                #   理由：execute_puppy_command 里有 time.sleep(move_sec)，
                #   如果先动，机器狗走完才播报，体验差；先说再做更自然。
                # 离散动作（坐下/站立/停止）：先动，再播报
                is_continuous = cmd in ('forward', 'turn_left', 'turn_right', 'backward')

                def _do_motion():
                    if args.ros_control:
                        send_voice_command(cmd)
                    elif args.udp_control:
                        execute_puppy_command(cmd, args.move_sec)

                if is_continuous:
                    if tts:
                        tts.speak(reply_text, args.speaker, args.volume)
                    _do_motion()
                else:
                    _do_motion()
                    if tts:
                        tts.speak(reply_text, args.speaker, args.volume)
                continue

            # 对话 → LLM
            print('  -> 对话，调 LLM...')
            reply = chat_with_llm(text)
            print(f'  LLM: {reply}')
            if tts:
                tts.speak(reply, args.speaker, args.volume)

        except KeyboardInterrupt:
            print('\n再见！')
            break
        except subprocess.CalledProcessError as e:
            print(f'[ERROR] 命令失败: {e}')
            time.sleep(1)
        except Exception as e:
            print(f'[ERROR] {repr(e)}')
            time.sleep(1)


if __name__ == '__main__':
    main()
