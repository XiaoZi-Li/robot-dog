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


class UsbAsrTextNode(Node):
    def __init__(self):
        super().__init__('usb_asr_text_node')

        self.declare_parameter('device', 'plughw:0,0')
        self.declare_parameter('record_seconds', 3)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('model_path', '/app/puppy_ws/models/vosk-model-small-cn-0.22')
        self.declare_parameter('loop_sleep_sec', 0.5)
        self.declare_parameter('min_text_length', 1)

        self.device = str(self.get_parameter('device').value)
        self.record_seconds = int(self.get_parameter('record_seconds').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.loop_sleep_sec = float(self.get_parameter('loop_sleep_sec').value)
        self.min_text_length = int(self.get_parameter('min_text_length').value)

        self.pub = self.create_publisher(String, '/asr/text', 10)

        self.get_logger().info(f'Loading Vosk model: {self.model_path}')
        self.model = Model(self.model_path)
        self.get_logger().info(
            f'usb_asr_text_node started: device={self.device}, '
            f'record_seconds={self.record_seconds}, sample_rate={self.sample_rate}'
        )

        self.busy = False
        self.timer = self.create_timer(0.1, self.loop_once)

    def loop_once(self):
        if self.busy:
            return

        self.busy = True
        try:
            text = self.record_and_recognize_once().strip()
            self.get_logger().info(f'ASR text: "{text}"')

            if len(text.replace(' ', '')) < self.min_text_length:
                time.sleep(self.loop_sleep_sec)
                return

            payload = {
                'source': 'usb_asr',
                'text': text,
                'timestamp': time.time(),
            }

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub.publish(msg)

            self.get_logger().info(f'publish /asr/text: {msg.data}')
            time.sleep(self.loop_sleep_sec)

        except Exception as e:
            self.get_logger().error(f'usb_asr_text_node failed: {repr(e)}')
            time.sleep(1.0)
        finally:
            self.busy = False

    def record_and_recognize_once(self) -> str:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            wav_path = f.name

        try:
            cmd = [
                'arecord',
                '-D', self.device,
                '-d', str(self.record_seconds),
                '-f', 'S16_LE',
                '-r', str(self.sample_rate),
                '-c', str(self.channels),
                '-t', 'wav',
                wav_path,
            ]

            self.get_logger().info(f'start recording {self.record_seconds}s...')
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            return self.recognize_wav(wav_path)

        finally:
            try:
                os.remove(wav_path)
            except Exception:
                pass

    def recognize_wav(self, wav_path: str) -> str:
        wf = wave.open(wav_path, 'rb')

        if wf.getnchannels() != 1:
            raise RuntimeError(f'音频不是单声道: {wf.getnchannels()}')
        if wf.getsampwidth() != 2:
            raise RuntimeError(f'音频不是16bit: {wf.getsampwidth()}')
        if wf.getframerate() != self.sample_rate:
            raise RuntimeError(f'音频采样率不对: {wf.getframerate()}')

        rec = KaldiRecognizer(self.model, wf.getframerate())
        text_parts = []

        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break

            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                part = result.get('text', '').strip()
                if part:
                    text_parts.append(part)

        final_result = json.loads(rec.FinalResult())
        final_text = final_result.get('text', '').strip()
        if final_text:
            text_parts.append(final_text)

        return ''.join(text_parts).strip()


def main(args=None):
    rclpy.init(args=args)
    node = UsbAsrTextNode()
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
