#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentRouterNode(Node):
    def __init__(self):
        super().__init__('intent_router_node')

        self.declare_parameter('control_cooldown_sec', 2.0)

        self.control_cooldown_sec = float(self.get_parameter('control_cooldown_sec').value)

        self.asr_sub = self.create_subscription(
            String,
            '/asr/text',
            self.asr_callback,
            10
        )

        self.voice_pub = self.create_publisher(String, '/voice/result_json', 10)
        self.chat_pub = self.create_publisher(String, '/chat/input_text', 10)

        self.last_control_command = None
        self.last_control_time = 0.0

        self.get_logger().info('intent_router_node started')

    def asr_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warn('invalid /asr/text json')
            return

        text = payload.get('text', '').strip()
        compact = text.replace(' ', '')

        if not compact:
            return

        intent = self.route_intent(compact)

        if intent['type'] == 'control':
            command = intent['command']
            now = time.time()

            if self.last_control_command == command and (now - self.last_control_time) < self.control_cooldown_sec:
                self.get_logger().info(f'ignore repeated control command: {command}')
                return

            out = {
                'source': 'voice',
                'sub_source': 'usb_asr_router',
                'command': command,
                'text': text,
                'timestamp': now,
            }

            out_msg = String()
            out_msg.data = json.dumps(out, ensure_ascii=False)
            self.voice_pub.publish(out_msg)

            self.last_control_command = command
            self.last_control_time = now

            self.get_logger().info(f'route to control: {out_msg.data}')
            return

        if intent['type'] == 'chat':
            out = {
                'source': 'usb_asr_router',
                'text': text,
                'timestamp': time.time(),
            }

            out_msg = String()
            out_msg.data = json.dumps(out, ensure_ascii=False)
            self.chat_pub.publish(out_msg)

            self.get_logger().info(f'route to chat: {out_msg.data}')
            return

    def route_intent(self, text: str):
        # 第一版：单主意图规则分流
        # 控制类优先，其余全部进对话链

        # 先匹配复合指令（避免被单字指令误吞）
        if any(kw in text for kw in ['开始跟随', '跟着我', '跟随我', '跟我走']):
            return {'type': 'control', 'command': 'follow_start'}

        if any(kw in text for kw in ['停止跟随', '不要跟了', '别跟了', '不要跟我']):
            return {'type': 'control', 'command': 'follow_stop'}

        if any(kw in text for kw in ['停下', '停止', '别动', '不要动', '停']):
            return {'type': 'control', 'command': 'stop'}

        if any(kw in text for kw in ['坐下', '坐下来', '请坐下', '坐下吧']):
            return {'type': 'control', 'command': 'sit'}

        if any(kw in text for kw in ['站立', '站起来', '起来', '请站起来', '站立起来']):
            return {'type': 'control', 'command': 'stand'}

        # 移动指令（持续 voice_move_sec 秒后自动停）
        if any(kw in text for kw in ['前进', '向前走', '往前走', '走', '向前', '直走']):
            return {'type': 'control', 'command': 'forward'}

        if any(kw in text for kw in ['后退', '向后走', '往后走', '倒车', '向后']):
            return {'type': 'control', 'command': 'backward'}

        if any(kw in text for kw in ['左转', '向左转', '往左转', '左边走', '左拐']):
            return {'type': 'control', 'command': 'turn_left'}

        if any(kw in text for kw in ['右转', '向右转', '往右转', '右边走', '右拐']):
            return {'type': 'control', 'command': 'turn_right'}

        return {'type': 'chat'}


def main(args=None):
    rclpy.init(args=args)
    node = IntentRouterNode()
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
