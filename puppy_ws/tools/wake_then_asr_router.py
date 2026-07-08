#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import audioop
import json
import queue
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import sherpa_onnx
from vosk import Model, KaldiRecognizer


KWS_BASE = "/app/puppy_ws/models/sherpa_kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
KWS_TOKENS = f"{KWS_BASE}/tokens.txt"
KWS_ENCODER = f"{KWS_BASE}/encoder-epoch-13-avg-2-chunk-16-left-64.onnx"
KWS_DECODER = f"{KWS_BASE}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
KWS_JOINER = f"{KWS_BASE}/joiner-epoch-13-avg-2-chunk-16-left-64.onnx"
KWS_KEYWORDS_FILE = "/app/puppy_ws/models/sherpa_kws/keywords_tokenized.txt"

ASR_MODEL_PATH = "/app/puppy_ws/models/vosk-model-small-cn-0.22"

MIC_SAMPLE_RATE = 44100
TARGET_SAMPLE_RATE = 16000
CHANNELS = 1

# 原来 2205(50ms) 太紧，这里改成 4410(100ms)
BLOCKSIZE = 4410
SD_DEVICE_INDEX = 0

STATE_WAKE = "WAKE_LISTEN"
STATE_RECORD = "RECORDING"
STATE_ASR = "ASR_BUSY"
STATE_COOLDOWN = "COOLDOWN"

PRE_ROLL_SEC = 0.5
MAX_RECORD_SEC = 5.0
MIN_RECORD_SEC = 0.8
END_SILENCE_SEC = 0.9
COOLDOWN_SEC = 1.5

RMS_SPEECH_THRESHOLD = 700
CONTROL_COOLDOWN_SEC = 2.0


class WakeThenAsrRouterNode(Node):
    def __init__(self):
        super().__init__('wake_then_asr_router_node')

        self.voice_pub = self.create_publisher(String, '/voice/result_json', 10)
        self.raw_asr_pub = self.create_publisher(String, '/voice/raw_asr_text', 10)
        self.chat_pub = self.create_publisher(String, '/chat/input_text', 10)

        self.get_logger().info('loading sherpa-onnx keyword spotter...')
        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=KWS_TOKENS,
            encoder=KWS_ENCODER,
            decoder=KWS_DECODER,
            joiner=KWS_JOINER,
            keywords_file=KWS_KEYWORDS_FILE,
            num_threads=1,
            provider='cpu',
            max_active_paths=4,
            num_trailing_blanks=1,
            keywords_score=1.0,
            keywords_threshold=0.35,
        )
        self.kws_stream = self.kws.create_stream()

        self.get_logger().info(f'loading Vosk ASR model: {ASR_MODEL_PATH}')
        self.asr_model = Model(ASR_MODEL_PATH)

        self.input_stream = None
        self.audio_queue = queue.Queue(maxsize=256)

        self.state = STATE_WAKE
        self.cooldown_until = 0.0

        self.rate_state = None
        self.last_control_command = None
        self.last_control_time = 0.0

        self.overflow_warn_time = 0.0
        self.queue_full_warn_time = 0.0

        self.chunk_sec = BLOCKSIZE / MIC_SAMPLE_RATE
        self.pre_roll_chunks = max(1, int(PRE_ROLL_SEC / self.chunk_sec))
        self.pre_roll_buffer = deque(maxlen=self.pre_roll_chunks)

        self.record_chunks = []
        self.record_start_time = 0.0
        self.voice_started = False
        self.silence_start_time = None

        self.get_logger().info('wake_then_asr_router_node started')
        self.get_logger().info(f'麦克风原生采样率={MIC_SAMPLE_RATE}, 算法采样率={TARGET_SAMPLE_RATE}')
        self.get_logger().info(f'BLOCKSIZE={BLOCKSIZE}, chunk_sec={self.chunk_sec:.3f}s')
        self.get_logger().info('当前状态：待命监听唤醒词“小狗”')

        self.start_stream()
        self.timer = self.create_timer(0.02, self.main_loop)

    def start_stream(self):
        if self.input_stream is not None:
            return

        self.audio_queue = queue.Queue(maxsize=256)

        self.input_stream = sd.InputStream(
            device=SD_DEVICE_INDEX,
            channels=1,
            samplerate=MIC_SAMPLE_RATE,
            dtype='int16',
            callback=self.audio_callback,
            blocksize=BLOCKSIZE,
            latency='high',
        )
        self.input_stream.start()
        self.get_logger().info('麦克风监听已启动（单流常开模式）')

    def stop_stream(self):
        if self.input_stream is None:
            return
        try:
            self.input_stream.stop()
            self.input_stream.close()
        finally:
            self.input_stream = None
        self.get_logger().info('麦克风监听已停止')

    def audio_callback(self, indata, frames, time_info, status):
        now = time.time()

        if status:
            if 'overflow' in str(status).lower():
                if now - self.overflow_warn_time > 1.0:
                    self.get_logger().warn(f'input status: {status}')
                    self.overflow_warn_time = now

        if self.state in (STATE_ASR, STATE_COOLDOWN):
            return

        try:
            chunk = indata[:, 0].copy()
            self.audio_queue.put_nowait(chunk)
        except queue.Full:
            if now - self.queue_full_warn_time > 1.0:
                self.get_logger().warn('audio_queue full, dropping audio chunks')
                self.queue_full_warn_time = now

    def reset_kws(self):
        self.kws_stream = self.kws.create_stream()
        self.rate_state = None

    def reset_record_state(self):
        self.record_chunks = []
        self.record_start_time = 0.0
        self.voice_started = False
        self.silence_start_time = None

    def resample_bytes_to_16k(self, pcm_bytes: bytes):
        converted, self.rate_state = audioop.ratecv(
            pcm_bytes, 2, 1,
            MIC_SAMPLE_RATE, TARGET_SAMPLE_RATE,
            self.rate_state
        )
        return converted

    def resample_full_to_16k(self, pcm_bytes: bytes) -> bytes:
        converted, _ = audioop.ratecv(
            pcm_bytes, 2, 1,
            MIC_SAMPLE_RATE, TARGET_SAMPLE_RATE,
            None
        )
        return converted

    def rms_of_chunk(self, chunk: np.ndarray) -> int:
        return audioop.rms(chunk.astype(np.int16).tobytes(), 2)

    def route_intent(self, text: str):
        compact = text.replace(' ', '')

        if any(kw in compact for kw in ['停下', '停止', '别动', '不要动']):
            return {'type': 'control', 'command': 'stop'}

        if any(kw in compact for kw in ['坐下', '坐下来', '请坐下']):
            return {'type': 'control', 'command': 'sit'}

        if any(kw in compact for kw in ['站立', '站起来', '起来', '请站起来']):
            return {'type': 'control', 'command': 'stand'}

        if any(kw in compact for kw in ['开始跟随', '跟着我', '跟随我']):
            return {'type': 'control', 'command': 'follow_start'}

        if any(kw in compact for kw in ['停止跟随', '不要跟了', '别跟了']):
            return {'type': 'control', 'command': 'follow_stop'}

        return {'type': 'chat'}

    def publish_control(self, text: str, command: str):
        now = time.time()

        if self.last_control_command == command and (now - self.last_control_time) < CONTROL_COOLDOWN_SEC:
            self.get_logger().info(f'ignore repeated control command within cooldown: {command}')
            return

        payload = {
            'source': 'voice',
            'sub_source': 'usb_wake_asr',
            'result_id': int(now * 1000),
            'command': command,
            'text': text,
            'timestamp': now,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.voice_pub.publish(msg)

        self.last_control_command = command
        self.last_control_time = now

        self.get_logger().info(f'route to control: {msg.data}')

    def publish_raw_asr(self, text: str):
        msg = String()
        msg.data = text
        self.raw_asr_pub.publish(msg)
        self.get_logger().info(f'publish /voice/raw_asr_text: {text}')

    def publish_chat(self, text: str):
        msg = String()
        msg.data = text
        self.chat_pub.publish(msg)
        self.get_logger().info(f'publish /chat/input_text: {text}')

    def process_wake_chunk(self, chunk: np.ndarray):
        self.pre_roll_buffer.append(chunk.copy())

        pcm_44k = chunk.astype(np.int16).tobytes()
        pcm_16k = self.resample_bytes_to_16k(pcm_44k)
        samples = np.frombuffer(pcm_16k, dtype=np.int16).astype(np.float32) / 32768.0

        self.kws_stream.accept_waveform(TARGET_SAMPLE_RATE, samples)

        while self.kws.is_ready(self.kws_stream):
            self.kws.decode_stream(self.kws_stream)

        result = self.kws.get_result(self.kws_stream)
        if result:
            self.get_logger().info(f'WAKE DETECTED: {result}')
            self.enter_recording()

    def enter_recording(self):
        self.state = STATE_RECORD
        self.record_start_time = time.time()
        self.voice_started = False
        self.silence_start_time = None
        self.record_chunks = list(self.pre_roll_buffer)
        self.get_logger().info('进入正式收音态（同一条流，不重开麦克风）')

    def process_record_chunk(self, chunk: np.ndarray):
        now = time.time()
        elapsed = now - self.record_start_time

        self.record_chunks.append(chunk.copy())

        rms = self.rms_of_chunk(chunk)

        if rms >= RMS_SPEECH_THRESHOLD:
            self.voice_started = True
            self.silence_start_time = None
        else:
            if self.voice_started and self.silence_start_time is None:
                self.silence_start_time = now

        if elapsed >= MAX_RECORD_SEC:
            self.get_logger().info('到达最大录音时长，结束本轮收音')
            self.finish_recording_and_asr()
            return

        if elapsed >= MIN_RECORD_SEC and self.voice_started and self.silence_start_time is not None:
            if (now - self.silence_start_time) >= END_SILENCE_SEC:
                self.get_logger().info('检测到尾部静音，结束本轮收音')
                self.finish_recording_and_asr()
                return

    def recognize_pcm_bytes(self, pcm_16k_bytes: bytes) -> str:
        rec = KaldiRecognizer(self.asr_model, TARGET_SAMPLE_RATE)
        parts = []

        step = 4000 * 2
        for i in range(0, len(pcm_16k_bytes), step):
            data = pcm_16k_bytes[i:i + step]
            if not data:
                break
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get('text', '').strip()
                if text:
                    parts.append(text)

        final_result = json.loads(rec.FinalResult())
        final_text = final_result.get('text', '').strip()
        if final_text:
            parts.append(final_text)

        return ''.join(parts).strip()

    def finish_recording_and_asr(self):
        self.state = STATE_ASR

        try:
            if not self.record_chunks:
                self.get_logger().info('record_chunks 为空，返回待命')
                self.enter_cooldown()
                return

            raw_44k = b''.join([c.astype(np.int16).tobytes() for c in self.record_chunks])
            raw_16k = self.resample_full_to_16k(raw_44k)

            text = self.recognize_pcm_bytes(raw_16k).strip()
            self.get_logger().info(f'ASR text: "{text}"')

            if text:
                self.publish_raw_asr(text)

                intent = self.route_intent(text)
                if intent['type'] == 'control':
                    self.publish_control(text, intent['command'])
                else:
                    self.publish_chat(text)
            else:
                self.get_logger().info('本轮 ASR 为空')

        except Exception as e:
            self.get_logger().error(f'finish_recording_and_asr failed: {repr(e)}')
        finally:
            self.enter_cooldown()

    def enter_cooldown(self):
        self.reset_record_state()
        self.pre_roll_buffer.clear()
        self.reset_kws()
        self.cooldown_until = time.time() + COOLDOWN_SEC
        self.state = STATE_COOLDOWN
        self.get_logger().info(f'进入冷却态 {COOLDOWN_SEC}s')

    def leave_cooldown_if_needed(self):
        if self.state == STATE_COOLDOWN and time.time() >= self.cooldown_until:
            self.state = STATE_WAKE
            self.get_logger().info('回到待命监听唤醒词“小狗”')

    def main_loop(self):
        self.leave_cooldown_if_needed()

        if self.state in (STATE_ASR, STATE_COOLDOWN):
            return

        processed = 0
        while (not self.audio_queue.empty()) and processed < 24:
            chunk = self.audio_queue.get_nowait()
            processed += 1

            if self.state == STATE_WAKE:
                self.process_wake_chunk(chunk)
            elif self.state == STATE_RECORD:
                self.process_record_chunk(chunk)

def main(args=None):
    rclpy.init(args=args)
    node = WakeThenAsrRouterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_stream()
        except Exception:
            pass
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