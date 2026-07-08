#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smbus import SMBus


class ASR:
    ASR_RESULT_ADDR = 100
    ASR_WORDS_ERASE_ADDR = 101
    ASR_MODE_ADDR = 102
    ASR_ADD_WORDS_ADDR = 160

    def __init__(self, address: int, bus_id: int):
        self.address = address
        self.bus = SMBus(bus_id)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def write_byte(self, val: int) -> bool:
        try:
            self.bus.write_byte(self.address, val)
            return True
        except Exception:
            return False

    def get_result(self) -> int:
        try:
            ok = self.write_byte(self.ASR_RESULT_ADDR)
            if not ok:
                return -1
            value = self.bus.read_byte(self.address)
            return int(value)
        except Exception:
            return -1

    def erase_words(self) -> bool:
        try:
            self.bus.write_byte_data(self.address, self.ASR_WORDS_ERASE_ADDR, 0)
            time.sleep(0.06)
            return True
        except Exception:
            return False

    def set_mode(self, mode: int) -> bool:
        try:
            self.bus.write_byte_data(self.address, self.ASR_MODE_ADDR, mode)
            time.sleep(0.05)
            return True
        except Exception:
            return False

    def add_words(self, id_num: int, words: str) -> bool:
        try:
            buf = [id_num]
            for ch in words:
                buf.append(ord(ch))
            self.bus.write_i2c_block_data(self.address, self.ASR_ADD_WORDS_ADDR, buf)
            time.sleep(0.05)
            return True
        except Exception:
            return False


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        self.declare_parameter('i2c_bus', 5)
        self.declare_parameter('i2c_addr', 0x79)
        self.declare_parameter('mode', 1)
        self.declare_parameter('init_words', True)
        self.declare_parameter('poll_interval', 0.10)
        self.declare_parameter('cooldown_sec', 1.5)
        self.declare_parameter('debug_log_interval_sec', 1.0)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.i2c_addr = int(self.get_parameter('i2c_addr').value)
        self.mode = int(self.get_parameter('mode').value)
        self.init_words = bool(self.get_parameter('init_words').value)
        self.poll_interval = float(self.get_parameter('poll_interval').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.debug_log_interval_sec = float(self.get_parameter('debug_log_interval_sec').value)

        self.pub = self.create_publisher(String, '/voice/result_json', 10)

        self.id_to_command: Dict[int, Dict[str, str]] = {
            1: {'command': 'stand', 'text': '站立'},
            2: {'command': 'sit', 'text': '坐下'},
            3: {'command': 'stop', 'text': '停下'},
        }

        self.last_publish_time = 0.0
        self.last_result_id = None
        self.last_debug_time = 0.0

        self.asr = ASR(self.i2c_addr, self.i2c_bus)

        self.get_logger().info(
            f'voice_control_node start: bus={self.i2c_bus}, addr=0x{self.i2c_addr:02X}, mode={self.mode}'
        )

        if self.init_words:
            self.init_asr_words()

        self.timer = self.create_timer(self.poll_interval, self.poll_asr)

    def init_asr_words(self):
        self.get_logger().info('Initializing ASR words...')

        ok1 = self.asr.erase_words()
        ok2 = self.asr.set_mode(self.mode)
        ok3 = self.asr.add_words(1, 'zhan li')
        ok4 = self.asr.add_words(2, 'zuo xia')
        ok5 = self.asr.add_words(3, 'ting xia')

        self.get_logger().info(
            f'ASR init result: erase={ok1}, set_mode={ok2}, add1={ok3}, add2={ok4}, add3={ok5}'
        )

        self.get_logger().info('ASR words initialized: 1=zhan li, 2=zuo xia, 3=ting xia')

    def publish_result(self, result_id: int, command: str, text: str):
        now = time.time()
        payload = {
            'source': 'voice',
            'result_id': result_id,
            'command': command,
            'text': text,
            'timestamp': now,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        self.get_logger().info(f'publish voice: {msg.data}')

    def poll_asr(self):
        result = self.asr.get_result()
        now = time.time()

        if (now - self.last_debug_time) >= self.debug_log_interval_sec:
            self.get_logger().info(f'ASR raw result={result}')
            self.last_debug_time = now

        if result is None or result <= 0:
            return

        if self.last_result_id == result and (now - self.last_publish_time) < self.cooldown_sec:
            return

        if result not in self.id_to_command:
            self.get_logger().warn(f'Unknown ASR result id: {result}')
            self.last_result_id = result
            self.last_publish_time = now
            return

        command = self.id_to_command[result]['command']
        text = self.id_to_command[result]['text']

        self.publish_result(result, command, text)
        self.last_result_id = result
        self.last_publish_time = now

    def destroy_node(self):
        try:
            self.asr.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
