#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_controller_node.py

手势控制节点——根据 21 关键点 + hobot 手势模型输出，判定 6 种手势，
映射到 puppy_action（sit/crouch/walk/backward/turn_left/turn_right）。

输入:
  - /hobot_hand_lmk_detection (ai_msgs/msg/PerceptionTargets, 21 关键点)
  - /gesture/result_json        (String+JSON, gesture_adapter_node 输出,
                                  含 thumb_up=4 等模型类)

输出:
  - /puppy_action  (String+JSON) —— 仅当 debug_mode=false 时发布
  - 日志              —— debug_mode=true 时只打印

6 种手势规则:
  thumb_up         : 模型值 4 (👍 点赞)           → sit
  two_fingers_cross: index_tip↔middle_tip 距离 < 阈值, 其他指尖伸出 → crouch
  thumb_left       : 握拳 + thumb_tip 在 wrist 左边                → turn_left
  thumb_right      : 握拳 + thumb_tip 在 wrist 右边                → turn_right
  palm_forward     : 张开手掌 + wrist 在画面下半                   → walk
  palm_back        : 张开手掌 + wrist 在画面上半                   → backward

GS130W 物理倒装: 图像上下颠倒, 因此节点对 21 关键点的 y 坐标翻一次 (1-y)。

约束:
  - 不修改 /app/gs130w_stereo/ 任何代码
  - 不修改 decision_node.py / gesture_adapter_node.py
  - 不启动就生效, 默认 debug_mode=true (安全)
"""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets


# 21 个关键点索引 (标准 mediapipe 风格, hobot 沿用)
WRIST = 0
THUMB_TIP, THUMB_IP, THUMB_MCP = 4, 3, 2
INDEX_TIP, INDEX_PIP, INDEX_MCP = 8, 7, 6
MIDDLE_TIP, MIDDLE_PIP, MIDDLE_MCP = 12, 11, 10
RING_TIP, RING_PIP, RING_MCP = 16, 15, 14
PINKY_TIP, PINKY_PIP, PINKY_MCP = 20, 19, 18

# 默认手势→动作映射 (gesture_map.json 优先; 失败回退)
DEFAULT_GESTURE_MAP = {
    "thumb_up": "sit",
    "two_fingers_cross": "crouch",
    "thumb_left": "turn_left",
    "thumb_right": "turn_right",
    "palm_forward": "walk",
    "palm_back": "backward",
}


class GestureControllerNode(Node):
    def __init__(self):
        super().__init__('gesture_controller_node')

        # ====== 声明参数 (launch 可覆盖) ======
        self.declare_parameter('gesture_topic', '/gesture/result_json')
        self.declare_parameter('hand_lmk_topic', '/hobot_hand_lmk_detection')
        self.declare_parameter('action_topic', '/puppy_action')
        self.declare_parameter('gesture_map_path',
                               '/app/puppy_ws/src/puppy_brain/config/gesture_map.json')
        self.declare_parameter('debug_mode', True)        # true: 只 print, 不发 puppy_action
        self.declare_parameter('image_flip_y', True)       # GS130W 物理倒装修正
        self.declare_parameter('hold_sec', 0.8)            # 手势保持时长 (去抖)
        self.declare_parameter('action_throttle_sec', 1.5)  # 同一动作最小发送间隔

        # ====== 几何阈值 (调试时常调) ======
        self.declare_parameter('cross_distance_th', 0.04)        # 两指交叉指尖距离
        self.declare_parameter('thumb_side_offset_th', 0.10)      # 拇指左右偏移
        self.declare_parameter('hand_open_th', 0.15)              # 张开时四指平均距离
        self.declare_parameter('hand_closed_th', 0.08)            # 握拳时四指最大距离
        self.declare_parameter('forward_backward_y_split', 0.50)  # wrist y 上下分割线
        self.declare_parameter('log_first_n_frames', 3)           # 启动后打印前 N 帧关键点

        # ====== 读参数 ======
        self.gesture_topic = self.get_parameter('gesture_topic').value
        self.hand_lmk_topic = self.get_parameter('hand_lmk_topic').value
        self.action_topic = self.get_parameter('action_topic').value
        self.gesture_map_path = self.get_parameter('gesture_map_path').value
        self.debug_mode = bool(self.get_parameter('debug_mode').value)
        self.image_flip_y = bool(self.get_parameter('image_flip_y').value)
        self.hold_sec = float(self.get_parameter('hold_sec').value)
        self.action_throttle_sec = float(self.get_parameter('action_throttle_sec').value)

        self.cross_distance_th = float(self.get_parameter('cross_distance_th').value)
        self.thumb_side_offset_th = float(self.get_parameter('thumb_side_offset_th').value)
        self.hand_open_th = float(self.get_parameter('hand_open_th').value)
        self.hand_closed_th = float(self.get_parameter('hand_closed_th').value)
        self.fb_y_split = float(self.get_parameter('forward_backward_y_split').value)
        self.log_first_n_frames = int(self.get_parameter('log_first_n_frames').value)

        # ====== 状态 ======
        self.gesture_map = self._load_gesture_map(self.gesture_map_path)
        self.current_gesture_name = None
        self.gesture_first_seen = 0.0
        self.last_action_sent = None
        self.last_action_time = 0.0
        self.frames_seen = 0

        # ====== Publisher ======
        self.action_pub = self.create_publisher(String, self.action_topic, 10)

        # ====== Subscribers (BEST_EFFORT 兼容 hobot 输出) ======
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.gesture_sub = self.create_subscription(
            String, self.gesture_topic, self.gesture_callback, sensor_qos
        )
        self.hand_lmk_sub = self.create_subscription(
            PerceptionTargets, self.hand_lmk_topic, self.hand_lmk_callback, sensor_qos
        )

        self.get_logger().info(
            f'gesture_controller_node started. debug_mode={self.debug_mode}, '
            f'flip_y={self.image_flip_y}, map={self.gesture_map_path}, '
            f'loaded={len(self.gesture_map)} gestures'
        )

    # ------------------------------------------------------------------
    # 加载手势映射
    # ------------------------------------------------------------------
    def _load_gesture_map(self, path: str) -> dict:
        try:
            p = Path(path)
            if not p.exists():
                self.get_logger().warn(
                    f'gesture_map not found at {path}, using DEFAULT_GESTURE_MAP'
                )
                return DEFAULT_GESTURE_MAP
            with p.open('r', encoding='utf-8') as f:
                data = json.load(f)
            clean = {k: v for k, v in data.items() if not k.startswith('_')
                     and isinstance(k, str) and isinstance(v, str)}
            if not clean:
                raise ValueError('no valid gesture→action entries')
            self.get_logger().info(
                f'gesture_map loaded: {clean} from {path}'
            )
            return clean
        except Exception as e:
            self.get_logger().error(f'load_gesture_map error: {e}, using default')
            return DEFAULT_GESTURE_MAP

    # ------------------------------------------------------------------
    # 模型类手势回调 (gesture_adapter_node 输出 JSON)
    # ------------------------------------------------------------------
    def gesture_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        gesture_value = payload.get('gesture_value', None)
        if gesture_value is None:
            return

        try:
            v = float(gesture_value)
        except Exception:
            return

        # 模型值 4 = thumb_up (点赞) — 我们映射表里直接认
        if v == 4.0:
            self._set_current_gesture('thumb_up')
        # 其他模型值 (1-3, 5-14) 我们用关键点规则覆盖,
        # 这里不主动处理, 避免和关键点规则抢优先级

    # ------------------------------------------------------------------
    # 21 关键点回调
    # ------------------------------------------------------------------
    def hand_lmk_callback(self, msg: PerceptionTargets):
        pts = self._extract_landmarks(msg)
        if pts is None:
            return

        # 启动后打印前 N 帧关键点位置, 方便用户对照摄像头方向验证 flip_y 是否正确
        if self.frames_seen < self.log_first_n_frames:
            self._dump_landmarks(pts, frame_idx=self.frames_seen)
        self.frames_seen += 1

        # 应用 y 翻转 (GS130W 物理倒装)
        if self.image_flip_y:
            for p in pts:
                p.y = 1.0 - p.y

        gesture_name = self._detect_gesture_by_rules(pts)
        if gesture_name is None:
            return

        self._set_current_gesture(gesture_name)

    def _extract_landmarks(self, msg: PerceptionTargets):
        """从 PerceptionTargets 提取第一个 hand 的 21 个关键点."""
        for target in msg.targets:
            for pt_msg in target.points:
                if pt_msg.type == 'hand_kps' and len(pt_msg.point) == 21:
                    return list(pt_msg.point)
        return None

    def _dump_landmarks(self, pts, frame_idx: int):
        """打印前 N 帧关键点位置, 用于调试."""
        labels = {
            WRIST: 'wrist', THUMB_TIP: 'thumb_tip', INDEX_TIP: 'index_tip',
            MIDDLE_TIP: 'mid_tip', RING_TIP: 'ring_tip', PINKY_TIP: 'pinky_tip',
        }
        parts = []
        for i, name in labels.items():
            parts.append(f'{name}=({pts[i].x:.2f},{pts[i].y:.2f})')
        self.get_logger().info(
            f'[调试 frame {frame_idx}] ' + ' '.join(parts)
            + (' (flip_y=True, 0=图像顶)' if self.image_flip_y else ' (flip_y=False, 0=图像底)')
        )

    def _detect_gesture_by_rules(self, pts):
        """6 个手势规则判定 (相对 wrist 的几何关系)."""
        if pts is None or len(pts) != 21:
            return None

        wx, wy = pts[WRIST].x, pts[WRIST].y
        if wx < 0 or wy < 0 or wx > 1.0 or wy > 1.0:
            return None

        d_thumb = math.hypot(pts[THUMB_TIP].x - wx, pts[THUMB_TIP].y - wy)
        d_index = math.hypot(pts[INDEX_TIP].x - wx, pts[INDEX_TIP].y - wy)
        d_middle = math.hypot(pts[MIDDLE_TIP].x - wx, pts[MIDDLE_TIP].y - wy)
        d_ring = math.hypot(pts[RING_TIP].x - wx, pts[RING_TIP].y - wy)
        d_pinky = math.hypot(pts[PINKY_TIP].x - wx, pts[PINKY_TIP].y - wy)
        d_fingers_avg = (d_index + d_middle + d_ring + d_pinky) / 4.0

        thumb_dx = pts[THUMB_TIP].x - wx

        # ===== 规则 1: 两指交叉 (index_tip ↔ middle_tip 距离小) =====
        tip_dist_im = math.hypot(
            pts[INDEX_TIP].x - pts[MIDDLE_TIP].x,
            pts[INDEX_TIP].y - pts[MIDDLE_TIP].y,
        )
        if tip_dist_im < self.cross_distance_th \
                and d_index > self.hand_closed_th * 1.5 \
                and d_middle > self.hand_closed_th * 1.5 \
                and d_ring > self.hand_closed_th \
                and d_pinky > self.hand_closed_th:
            return 'two_fingers_cross'

        # ===== 规则 2: 握拳 + 拇指伸向左/右 =====
        fingers_closed = (
            d_index < self.hand_closed_th
            and d_middle < self.hand_closed_th
            and d_ring < self.hand_closed_th
            and d_pinky < self.hand_closed_th
        )
        if fingers_closed and d_thumb > self.hand_closed_th:
            if thumb_dx < -self.thumb_side_offset_th:
                return 'thumb_left'
            if thumb_dx > self.thumb_side_offset_th:
                return 'thumb_right'

        # ===== 规则 3: 手掌张开 → 前进/后退 (wrist 在画面 y 分割) =====
        if d_fingers_avg > self.hand_open_th and d_thumb > 0.08:
            if wy > self.fb_y_split:
                return 'palm_forward'
            return 'palm_back'

        return None

    # ------------------------------------------------------------------
    # 设置当前手势 + 去抖 + 限流
    # ------------------------------------------------------------------
    def _set_current_gesture(self, gesture_name: str):
        if gesture_name not in self.gesture_map:
            return

        now = time.time()

        if gesture_name != self.current_gesture_name:
            self.current_gesture_name = gesture_name
            self.gesture_first_seen = now
            return

        if now - self.gesture_first_seen < self.hold_sec:
            return

        action = self.gesture_map[gesture_name]

        if action == self.last_action_sent \
                and now - self.last_action_time < self.action_throttle_sec:
            return

        self._send_action(action, gesture_name)

    def _send_action(self, action: str, gesture_name: str):
        now = time.time()
        self.last_action_sent = action
        self.last_action_time = now

        if self.debug_mode:
            self.get_logger().info(
                f'[DEBUG] ✓ 识别到手势 [{gesture_name}] → 应该执行 action [{action}] '
                f'(hold={now - self.gesture_first_seen:.2f}s)'
            )
            return

        payload = {
            'action': action,
            'source': 'gesture_controller',
            'timestamp': now,
            'gesture': gesture_name,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.action_pub.publish(msg)
        self.get_logger().info(
            f'→ sent action [{action}] from gesture [{gesture_name}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GestureControllerNode()
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
