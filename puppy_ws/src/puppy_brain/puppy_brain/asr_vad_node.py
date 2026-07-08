#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 版 ASR 节点（VAD 自动断句 + Vosk 识别）
=============================================
识别出文本后发 /asr/text（JSON），由 intent_router 路由到控制/对话链。

Topic:
  发布: /asr/text (std_msgs/String)  格式: {"source":"vad_asr","text":"前进","timestamp":...}
  订阅: /tts_busy  (std_msgs/Bool)   TTS 播放时暂停录音（防自激）

特点：
  - VAD 自动检测说话开始/结束，不需要固定录音窗口
  - TTS 播放时暂停 ASR（防自激）
  - 不做指令匹配，交给 intent_router（复用现有路由逻辑）

用法（launch 里配）：
  ros2 run puppy_brain asr_vad_node --ros-args -p mic_device:=plughw:1,0
"""
import json
import os
import subprocess
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from vosk import Model, KaldiRecognizer

try:
    import webrtcvad
    HAS_VAD = True
except ImportError:
    HAS_VAD = False


class AsrVadNode(Node):
    def __init__(self):
        super().__init__('asr_vad_node')

        # ============ 参数 ============
        self.declare_parameter('mic_device', 'plughw:1,0')
        self.declare_parameter('model_path',
            '/app/puppy_ws/models/vosk-model-small-cn-0.22')
        self.declare_parameter('save_dir', '/tmp/asr_vad')
        self.declare_parameter('denoise', False)
        self.declare_parameter('noise_prof', '/tmp/noise.prof')
        self.declare_parameter('gain_db', 10)
        self.declare_parameter('aggressiveness', 2)
        self.declare_parameter('silence_dur', 1.0)
        self.declare_parameter('max_wait', 8.0)
        self.declare_parameter('max_speech', 5.0)
        self.declare_parameter('min_text_length', 1)
        self.declare_parameter('loop_sleep_sec', 0.1)

        self.mic = str(self.get_parameter('mic_device').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.denoise = bool(self.get_parameter('denoise').value)
        self.noise_prof = str(self.get_parameter('noise_prof').value)
        self.gain_db = int(self.get_parameter('gain_db').value)
        self.aggressiveness = int(self.get_parameter('aggressiveness').value)
        self.silence_dur = float(self.get_parameter('silence_dur').value)
        self.max_wait = float(self.get_parameter('max_wait').value)
        self.max_speech = float(self.get_parameter('max_speech').value)
        self.min_text_length = int(self.get_parameter('min_text_length').value)
        self.loop_sleep_sec = float(self.get_parameter('loop_sleep_sec').value)

        os.makedirs(self.save_dir, exist_ok=True)

        # ============ TTS 防自激 ============
        self.tts_busy = False
        self.tts_busy_sub = self.create_subscription(
            Bool, '/tts_busy', self.on_tts_busy, 10
        )

        # ============ 发布 /asr/text ============
        self.asr_pub = self.create_publisher(String, '/asr/text', 10)

        # ============ 加载 Vosk 模型 ============
        self.get_logger().info(f'加载 Vosk 模型: {self.model_path}')
        self.model = Model(self.model_path)
        self.get_logger().info('Vosk 模型加载完成')

        if self.denoise and not os.path.exists(self.noise_prof):
            self.get_logger().warn(f'噪声样本不存在: {self.noise_prof}，不降噪')
            self.denoise = False

        if not HAS_VAD:
            self.get_logger().warn('未装 webrtcvad，回退固定4秒录音')

        self.get_logger().info(f'asr_vad_node 启动: mic={self.mic}, '
                               f'gain={self.gain_db}dB, '
                               f'VAD={HAS_VAD}')

        # ============ 录音循环（独立线程） ============
        self.round_num = 0
        self.busy = False
        self.timer = self.create_timer(self.loop_sleep_sec, self.loop_once)

    def on_tts_busy(self, msg: Bool):
        """TTS 播放时暂停 ASR"""
        self.tts_busy = msg.data
        if self.tts_busy:
            self.get_logger().info('[ASR] TTS 播放中，暂停录音')

    def loop_once(self):
        if self.busy or self.tts_busy:
            return

        self.busy = True
        try:
            self.round_num += 1
            wav_path = os.path.join(self.save_dir, f'rec_{self.round_num:03d}.wav')

            # TTS 播放期间可能变 busy，录音前再检查一次
            if self.tts_busy:
                return

            mode = self._record_with_vad(wav_path)
            if mode == 'empty':
                return

            text = self._recognize(wav_path)
            if not text or len(text.replace(' ', '')) < self.min_text_length:
                return

            self.get_logger().info(f'识别: "{text}"')

            # 发 /asr/text（JSON 格式，兼容 intent_router）
            payload = {
                'source': 'vad_asr',
                'text': text,
                'timestamp': time.time(),
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.asr_pub.publish(msg)
            self.get_logger().info(f'发布 /asr/text: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'ASR 失败: {repr(e)}')
            time.sleep(1.0)
        finally:
            self.busy = False

    # ============ VAD 录音 ============
    def _record_with_vad(self, wav_path):
        if not HAS_VAD:
            raw = wav_path.replace('.wav', '_raw.wav')
            subprocess.run(
                ['arecord', '-D', self.mic, '-d', '4',
                 '-f', 'S16_LE', '-r', '16000', '-c', '1', raw],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self._postprocess(raw, wav_path)
            return 'fixed-4s'

        vad = webrtcvad.Vad(self.aggressiveness)
        sample_rate = 16000
        frame_duration = 30
        frame_size = int(sample_rate * frame_duration / 1000) * 2

        proc = subprocess.Popen(
            ['arecord', '-D', self.mic,
             '-f', 'S16_LE', '-r', str(sample_rate), '-c', '1',
             '-t', 'raw', '-q'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )

        raw_path = wav_path.replace('.wav', '_raw.wav')
        raw_frames = []
        state = 'waiting'
        wait_start = time.time()
        speech_start = 0
        total_speech = 0
        speech_frame_count = 0
        min_speech_frames = 6
        silence_frame_count = 0
        min_silence_frames = int(self.silence_dur * 1000 / frame_duration)

        try:
            while True:
                # TTS 变 busy 时立即停止录音
                if self.tts_busy:
                    proc.terminate()
                    proc.wait()
                    return 'empty'

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
                            silence_frame_count = 0
                    else:
                        speech_frame_count = 0
                        raw_frames = []
                        if now - wait_start > self.max_wait:
                            break
                elif state == 'speaking':
                    raw_frames.append(frame)
                    if is_speech:
                        silence_frame_count = 0
                        total_speech = now - speech_start
                        if total_speech > self.max_speech:
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

        self._postprocess(raw_path, wav_path)
        return 'vad'

    def _postprocess(self, raw_path, wav_path):
        if self.gain_db != 0 or (self.denoise and os.path.exists(self.noise_prof)):
            cmd = ['sox', raw_path, wav_path]
            if self.denoise and os.path.exists(self.noise_prof):
                cmd += ['noisered', self.noise_prof, '0.2']
            if self.gain_db != 0:
                cmd += ['gain', str(self.gain_db)]
            subprocess.run(cmd, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)
            os.remove(raw_path)
        else:
            os.rename(raw_path, wav_path)

    def _recognize(self, wav_path):
        wf = wave.open(wav_path, 'rb')
        rec = KaldiRecognizer(self.model, 16000)
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


def main(args=None):
    rclpy.init(args=args)
    node = AsrVadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
