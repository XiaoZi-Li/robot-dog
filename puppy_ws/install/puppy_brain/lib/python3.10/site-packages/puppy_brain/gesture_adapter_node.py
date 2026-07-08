#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_adapter_node.py - hobot 手势结果 → 统一 JSON 适配节点

订阅 hobot hand_gesture_detection 输出的 PerceptionTargets，
转成统一 JSON 发到 /gesture/result_json，供 decision_node 消费。

增强点：
1. 手势值→可读名称映射 (gesture_name 字段)
2. QoS 改 BEST_EFFORT，兼容 hobot AI 节点输出 (BEST_EFFORT 订阅者能同时兼容 RELIABLE/BEST_EFFORT 发布者)
3. 提取置信度 confidence (如 hobot 提供)
4. 节流日志，方便调试
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets


# ============ hobot hand_gesture 模型手势值→名称映射 ============
# 来源: docs/puppy_ws使用指南.md + hobot gestureDet_8x21.hbm 标准
# 1-5 已确认; 6+ 预留扩展 (hobot 32类模型可能输出更多值)
GESTURE_NAME_MAP = {
    1.0: 'palm',           # 手掌张开
    2.0: 'fist',           # 握拳
    3.0: 'okay',           # OK 手势
    4.0: 'thumb_up',       # 点赞
    5.0: 'index_finger',   # 竖食指
    # --- 以下为预留, hobot 标准模型未确认输出, 供 mediapipe/扩展用 ---
    6.0: 'thumb_down',
    7.0: 'iloveyou',
    8.0: 'rock',
    9.0: 'vulcan',
    10.0: 'pinch',
    # --- mediapipe 扩展手势值 (操作手册约定) ---
    11.0: 'okay_mp',       # mediapipe Okay → crouch
    12.0: 'thumb_left',    # mediapipe ThumbLeft → turn_left
    13.0: 'thumb_right',   # mediapipe ThumbRight → turn_right
    14.0: 'awesome',       # mediapipe Awesome → follow_on
}


def lookup_gesture_name(value) -> str:
    """手势值 → 可读名称, 未知值返回 'unknown'"""
    try:
        return GESTURE_NAME_MAP.get(float(value), 'unknown')
    except Exception:
        return 'unknown'


# ============ 订阅 hobot AI 输出用的兼容 QoS ============
# SKILL.md 约定: BEST_EFFORT 订阅者能同时兼容 RELIABLE 和 BEST_EFFORT 发布者
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class GestureAdapterNode(Node):
    def __init__(self):
        super().__init__('gesture_adapter_node')

        self.declare_parameter('input_topic', '/hobot_hand_gesture_detection')
        self.declare_parameter('output_topic', '/gesture/result_json')
        self.declare_parameter('log_interval_sec', 0.5)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        self.pub = self.create_publisher(String, self.output_topic, 10)
        # 用 BEST_EFFORT QoS 订阅, 兼容 hobot AI 节点输出
        self.sub = self.create_subscription(
            PerceptionTargets,
            self.input_topic,
            self.callback,
            SENSOR_QOS,
        )

        self.last_log_time = 0.0

        self.get_logger().info(
            f'gesture_adapter_node started. input={self.input_topic}, '
            f'output={self.output_topic}, qos=BEST_EFFORT'
        )

    def callback(self, msg: PerceptionTargets):
        gesture_value = None
        track_id = None
        confidence = None

        for target in msg.targets:
            for attr in target.attributes:
                if attr.type == 'gesture':
                    gesture_value = attr.value
                    track_id = target.track_id
                    # hobot 可能提供置信度, 尝试提取
                    try:
                        confidence = float(attr.confidence)
                    except Exception:
                        confidence = None
                    break
            if gesture_value is not None:
                break

        if gesture_value is None:
            return

        gesture_name = lookup_gesture_name(gesture_value)

        out = {
            'gesture': gesture_name if gesture_name != 'unknown' else str(gesture_value),
            'gesture_value': gesture_value,
            'gesture_name': gesture_name,
            'track_id': track_id,
            'source_topic': self.input_topic,
            'timestamp': time.time(),
        }
        if confidence is not None:
            out['confidence'] = confidence

        out_msg = String()
        out_msg.data = json.dumps(out, ensure_ascii=False)
        self.pub.publish(out_msg)

        # 节流日志
        now = time.time()
        if now - self.last_log_time > self.log_interval_sec:
            self.get_logger().info(
                f'gesture: name={gesture_name} value={gesture_value} '
                f'track={track_id} conf={confidence}'
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GestureAdapterNode()
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
