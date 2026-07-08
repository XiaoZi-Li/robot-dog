#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChatLlmBridgeNode(Node):
    def __init__(self):
        super().__init__('chat_llm_bridge_node')

        self.declare_parameter('chat_input_topic', '/chat/input_text')
        self.declare_parameter('chat_output_topic', '/chat/response_text')
        self.declare_parameter('llm_input_topic', '/prompt_text')
        self.declare_parameter('llm_output_topic', '/tts_text')

        # 收尾超时：多久没收到新片段，就认为本轮回答结束
        self.declare_parameter('flush_timeout_sec', 0.8)

        self.chat_input_topic = self.get_parameter('chat_input_topic').value
        self.chat_output_topic = self.get_parameter('chat_output_topic').value
        self.llm_input_topic = self.get_parameter('llm_input_topic').value
        self.llm_output_topic = self.get_parameter('llm_output_topic').value
        self.flush_timeout_sec = float(self.get_parameter('flush_timeout_sec').value)

        self.chat_in_sub = self.create_subscription(
            String,
            self.chat_input_topic,
            self.on_chat_input,
            10
        )

        self.llm_out_sub = self.create_subscription(
            String,
            self.llm_output_topic,
            self.on_llm_output,
            10
        )

        self.llm_in_pub = self.create_publisher(
            String,
            self.llm_input_topic,
            10
        )

        self.chat_out_pub = self.create_publisher(
            String,
            self.chat_output_topic,
            10
        )

        self._lock = threading.Lock()
        self._segments = []
        self._last_segment_time = 0.0

        self.flush_timer = self.create_timer(0.2, self.on_flush_timer)

        self.get_logger().info('chat_llm_bridge_node started')
        self.get_logger().info(f'{self.chat_input_topic}  ->  {self.llm_input_topic}')
        self.get_logger().info(f'{self.llm_output_topic}  ->  {self.chat_output_topic}')
        self.get_logger().info(f'flush_timeout_sec = {self.flush_timeout_sec}')

    def on_chat_input(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        # 新问题来了，先清空上一轮可能残留的片段
        with self._lock:
            self._segments = []
            self._last_segment_time = 0.0

        out = String()
        out.data = text
        self.llm_in_pub.publish(out)
        self.get_logger().info(f'forward chat->llm: "{text}"')

    def on_llm_output(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if text == 'READY_IGNORE':
            self.get_logger().info('ignore startup cute_words chunk')
            return
        now = time.time()

        with self._lock:
            self._segments.append(text)
            self._last_segment_time = now

        self.get_logger().info(f'recv llm chunk: "{text}"')

    def on_flush_timer(self):
        now = time.time()

        with self._lock:
            if not self._segments:
                return

            if self._last_segment_time <= 0.0:
                return

            if (now - self._last_segment_time) < self.flush_timeout_sec:
                return

            merged = self.merge_segments(self._segments)
            self._segments = []
            self._last_segment_time = 0.0

        if not merged:
            return

        out = String()
        out.data = merged
        self.chat_out_pub.publish(out)
        self.get_logger().info(f'forward llm->chat merged: "{merged}"')

    @staticmethod
    def merge_segments(segments):
        parts = []
        for seg in segments:
            seg = seg.strip()
            if not seg:
                continue
            parts.append(seg)

        if not parts:
            return ''

        # 这里先直接无分隔拼接，更适合中文
        merged = ''.join(parts)

        # 简单收尾清理
        merged = merged.replace('\n', ' ').strip()
        return merged


def main(args=None):
    rclpy.init(args=args)
    node = ChatLlmBridgeNode()
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