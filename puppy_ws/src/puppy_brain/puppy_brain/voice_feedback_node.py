#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音反馈节点：收到控制指令 → 发 TTS 反馈语
=================================================
订阅 /voice/result_json（和 decision_node 并行订阅），
解析 command，映射反馈语，发 /tts_text 让 tts_play_node 播放。

Topic:
  订阅: /voice/result_json (std_msgs/String)  intent_router 发出的控制指令
  发布: /tts_text           (std_msgs/String)  反馈语，tts_play_node 播放

反馈语映射可在此文件 FEEDBACK_MAP 修改。
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============ 指令 → 反馈语映射 ============
FEEDBACK_MAP = {
    'forward':    '好的，前进',
    'backward':   '好的，后退',
    'turn_left':  '好的，左转',
    'turn_right': '好的，右转',
    'stand':      '好的，站起来',
    'sit':        '好的，坐下',
    'crouch':     '好的，趴下',
    'wave':       '好的，摇摆',
    'stop':       '好的，停止',
    'follow_start': '好的，开始跟随',
    'follow_stop':  '好的，停止跟随',
    'gesture_on':   '手势控制已开启',
    'gesture_off':  '手势控制已关闭',
}


class VoiceFeedbackNode(Node):
    def __init__(self):
        super().__init__('voice_feedback_node')

        self.declare_parameter('cooldown_sec', 1.0)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.last_command = None
        self.last_time = 0.0

        self.voice_sub = self.create_subscription(
            String, '/voice/result_json', self.on_voice_command, 10
        )
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)

        self.get_logger().info('voice_feedback_node 启动')
        self.get_logger().info(f'反馈映射: {len(FEEDBACK_MAP)} 条指令')

    def on_voice_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        command = payload.get('command', None)
        if not command:
            return

        # 冷却：同一指令短时间内不重复反馈
        now = time.time()
        if command == self.last_command and \
           (now - self.last_time) < self.cooldown_sec:
            return

        feedback = FEEDBACK_MAP.get(command, None)
        if not feedback:
            # 未知指令不反馈
            return

        self.last_command = command
        self.last_time = now

        out = String()
        out.data = feedback
        self.tts_pub.publish(out)
        self.get_logger().info(f'[反馈] {command} → "{feedback}"')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceFeedbackNode()
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
