#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
RECORD_SECONDS = 4
SAMPLE_RATE = 16000
CHANNELS = 1

WAKEUP_KEYWORDS = [
    "小狗",
    "你好小狗",
]


def record_wav(wav_path: str, seconds: int):
    cmd = [
        "arecord",
        "-D", DEVICE,
        "-d", str(seconds),
        "-f", "S16_LE",
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-t", "wav",
        wav_path,
    ]
    subprocess.run(cmd, check=True)


def recognize_wav(model: Model, wav_path: str) -> str:
    wf = wave.open(wav_path, "rb")

    if wf.getnchannels() != 1:
        raise RuntimeError(f"音频不是单声道: {wf.getnchannels()}")
    if wf.getsampwidth() != 2:
        raise RuntimeError(f"音频不是16bit: {wf.getsampwidth()}")
    if wf.getframerate() != SAMPLE_RATE:
        raise RuntimeError(f"音频采样率不对: {wf.getframerate()}")

    rec = KaldiRecognizer(model, wf.getframerate())
    parts = []

    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
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


def extract_after_wakeup(text: str):
    compact = text.replace(" ", "")
    for kw in WAKEUP_KEYWORDS:
        idx = compact.find(kw)
        if idx != -1:
            remain = compact[idx + len(kw):].strip()
            return True, remain
    return False, compact


def route_intent(text: str):
    compact = text.replace(" ", "")

    if any(kw in compact for kw in ["停下", "停止", "别动", "不要动"]):
        return {"type": "control", "command": "stop"}

    if any(kw in compact for kw in ["坐下", "坐下来", "请坐下"]):
        return {"type": "control", "command": "sit"}

    if any(kw in compact for kw in ["站立", "站起来", "起来", "请站起来"]):
        return {"type": "control", "command": "stand"}

    if any(kw in compact for kw in ["开始跟随", "跟着我", "跟随我"]):
        return {"type": "control", "command": "follow_start"}

    if any(kw in compact for kw in ["停止跟随", "不要跟了", "别跟了"]):
        return {"type": "control", "command": "follow_stop"}

    return {"type": "chat"}


class WakeupRouterNode(Node):
    def __init__(self):
        super().__init__("asr_wakeup_once_router_node")
        self.voice_pub = self.create_publisher(String, "/voice/result_json", 10)
        self.chat_pub = self.create_publisher(String, "/chat/input_text", 10)

    def publish_control(self, text: str, command: str):
        payload = {
            "source": "voice",
            "sub_source": "usb_asr_wakeup_once",
            "command": command,
            "text": text,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.voice_pub.publish(msg)
        print(f"[ROUTE] control -> {msg.data}")

    def publish_chat(self, text: str):
        payload = {
            "source": "usb_asr_wakeup_once",
            "text": text,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.chat_pub.publish(msg)
        print(f"[ROUTE] chat -> {msg.data}")


def main():
    print(f"[INFO] loading model: {MODEL_PATH}")
    model = Model(MODEL_PATH)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav_path = f.name

    try:
        print(f"[INFO] start recording {RECORD_SECONDS}s ...")
        print("[INFO] 请直接说：小狗 + 指令/问题，例如：小狗坐下 / 小狗你叫什么名字")
        record_wav(wav_path, RECORD_SECONDS)

        text = recognize_wav(model, wav_path)
        print(f'[ASR] full_text="{text}"')

        if not text.strip():
            print("[INFO] empty text, exit")
            return

        matched, remain = extract_after_wakeup(text)
        print(f'[WAKEUP] matched={matched}, remain="{remain}"')

        if not matched:
            print("[INFO] wakeup keyword not matched, exit")
            return

        if not remain:
            print("[INFO] wakeup matched but no command/chat content, exit")
            return

        rclpy.init()
        node = WakeupRouterNode()

        intent = route_intent(remain)
        if intent["type"] == "control":
            node.publish_control(remain, intent["command"])
        else:
            node.publish_chat(remain)

        rclpy.spin_once(node, timeout_sec=0.3)
        node.destroy_node()
        rclpy.shutdown()

    finally:
        try:
            os.remove(wav_path)
        except Exception:
            pass


if __name__ == "__main__":
    main()
