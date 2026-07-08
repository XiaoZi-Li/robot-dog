#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import audioop
import json
import os
import subprocess
import tempfile
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer


MODEL_PATH = "/app/puppy_ws/models/vosk-model-small-cn-0.22"
DEVICE = "plughw:0,0"

RECORD_RATE = 44100
VOSK_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2

# 一句式模式：每轮录一整句
RECORD_SECONDS = 3
LOOP_SLEEP_SEC = 0.15

# USB 只做对话，不做控制
CONTROL_ONLY_PHRASES = [
    "坐下", "坐下来",
    "站立", "站起来", "起来",
    "停下", "停止", "别动", "不要动",
]

WAKEUP_KEYWORDS = [
    "小狗", "小狗狗", "晓狗", "小够", "小苟", "小古"
]


def record_wav(device: str, wav_path: str, seconds: int, rate: int, channels: int):
    cmd = [
        "arecord",
        "-D", device,
        "-d", str(seconds),
        "-f", "S16_LE",
        "-r", str(rate),
        "-c", str(channels),
        "-t", "wav",
        wav_path,
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def read_wav_pcm(wav_path: str):
    with wave.open(wav_path, "rb") as wf:
        nchannels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        framerate = wf.getframerate()
        frames = wf.readframes(wf.getnframes())

    if nchannels != 1:
        raise RuntimeError(f"音频不是单声道: {nchannels}")
    if sampwidth != 2:
        raise RuntimeError(f"音频不是16bit: {sampwidth}")

    return frames, framerate


def resample_pcm_44100_to_16000(pcm_bytes: bytes) -> bytes:
    converted, _ = audioop.ratecv(
        pcm_bytes,
        SAMPLE_WIDTH,
        CHANNELS,
        RECORD_RATE,
        VOSK_RATE,
        None
    )
    return converted


def recognize_wav(model: Model, wav_path: str) -> str:
    pcm_bytes, src_rate = read_wav_pcm(wav_path)

    if src_rate == VOSK_RATE:
        pcm_16k = pcm_bytes
    elif src_rate == RECORD_RATE:
        pcm_16k = resample_pcm_44100_to_16000(pcm_bytes)
    else:
        raise RuntimeError(f"不支持的音频采样率: {src_rate}")

    rec = KaldiRecognizer(model, VOSK_RATE)
    parts = []

    step = 4000 * 2
    for i in range(0, len(pcm_16k), step):
        data = pcm_16k[i:i + step]
        if not data:
            break

        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "").strip()
            if text:
                parts.append(text)

    final_result = json.loads(rec.FinalResult())
    final_text = final_result.get("text", "").strip()
    if final_text:
        parts.append(final_text)

    return "".join(parts).strip()


def normalize_text(text: str) -> str:
    return text.replace(" ", "").strip()


def extract_after_wakeup(text: str):
    compact = normalize_text(text)
    for kw in WAKEUP_KEYWORDS:
        idx = compact.find(kw)
        if idx != -1:
            remain = compact[idx + len(kw):].strip()
            return True, remain
    return False, compact


def is_control_only_text(text: str) -> bool:
    compact = normalize_text(text)
    if not compact:
        return False
    return any(kw in compact for kw in CONTROL_ONLY_PHRASES)


class AsrWakeupLoopRouterNode(Node):
    def __init__(self):
        super().__init__("asr_wakeup_loop_router_node")

        self.chat_pub = self.create_publisher(String, "/chat/input_text", 10)
        self.raw_pub = self.create_publisher(String, "/voice/raw_asr_text", 10)

        self.model = Model(MODEL_PATH)

        self.get_logger().info("asr_wakeup_loop_router_node started")
        self.get_logger().info(f"model={MODEL_PATH}")
        self.get_logger().info(f"device={DEVICE}")
        self.get_logger().info(f"record_rate={RECORD_RATE}, vosk_rate={VOSK_RATE}")
        self.get_logger().info("USB语音链仅做对话入口，不走控制链")
        self.get_logger().info("当前模式：一句式唤醒。请直接说：小狗 + 问题")

        self.busy = False
        self.timer = self.create_timer(0.1, self.loop_once)

    def publish_raw_asr(self, text: str):
        msg = String()
        msg.data = text
        self.raw_pub.publish(msg)
        self.get_logger().info(f'publish /voice/raw_asr_text: "{text}"')

    def publish_chat_text(self, text: str):
        msg = String()
        msg.data = text
        self.chat_pub.publish(msg)
        self.get_logger().info(f'publish /chat/input_text: "{text}"')

    def record_and_asr(self, seconds: int) -> str:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wav_path = f.name

        try:
            self.get_logger().info(f"record: start recording {seconds}s")
            record_wav(DEVICE, wav_path, seconds, RECORD_RATE, CHANNELS)
            text = recognize_wav(self.model, wav_path)
            self.get_logger().info(f'record: text="{text}"')
            return text
        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def loop_once(self):
        if self.busy:
            return

        self.busy = True
        try:
            text = self.record_and_asr(RECORD_SECONDS)

            if not normalize_text(text):
                time.sleep(LOOP_SLEEP_SEC)
                return

            matched, remain = extract_after_wakeup(text)
            self.get_logger().info(f'matched={matched}, remain="{remain}"')

            if not matched:
                time.sleep(LOOP_SLEEP_SEC)
                return

            if not normalize_text(remain):
                self.get_logger().info("检测到唤醒词，但后面没有有效问题，忽略")
                time.sleep(LOOP_SLEEP_SEC)
                return

            if is_control_only_text(remain):
                self.get_logger().info(f'USB侧过滤固定控制词，不转聊天："{remain}"')
                time.sleep(LOOP_SLEEP_SEC)
                return

            self.publish_raw_asr(remain)
            self.publish_chat_text(remain)
            time.sleep(0.2)

        except Exception as e:
            self.get_logger().error(f"asr_wakeup_loop_router_node failed: {repr(e)}")
            time.sleep(1.0)
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = AsrWakeupLoopRouterNode()
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


if __name__ == "__main__":
    main()