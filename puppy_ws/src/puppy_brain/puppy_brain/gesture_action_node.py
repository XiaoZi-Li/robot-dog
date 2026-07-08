#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_action_node.py - 手势 → 机器狗动作节点 (用户自定义映射)

订阅 hobot hand_gesture_detection 原始输出 (ai_msgs/PerceptionTargets),
按用户需求映射为动作, 发布到 /puppy_action, 由 ros_udp_bridge 转给 sit.py。

手势 → 动作映射 (用户需求):
  单 palm (手掌张开)  → forward   (前进)
  双 palm (双手)      → turn_right (右转)
  fist (握拳)         → crouch    (趴下)
  thumb_up (点赞)     → sit       (坐下)
  okay (OK 手势)      → backward  (后退)
  index_finger (食指) → turn_left  (左转)

执行策略:
  - 移动类 (forward/backward/turn_left/turn_right): 手势持续存在时持续发 follow_control,
    手势消失超 hold_sec 或超过 max_move_sec 自动刹车
  - 离散类 (sit/crouch): 单次发送, 加锁 action_lock_sec 避免重复触发

注意: 此节点与 decision_node 的手势映射不同, launch 时不要同时启动 decision_node,
否则两个节点都发 /puppy_action 会冲突。
"""
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets


# ============ 手势值 → 名称映射 (X5 官方 gestureDet_8x21.hbm) ============
# 来源: https://developer.d-robotics.cc/tros_doc/boxs/body/hand_gesture_detection
GESTURE_NAME_MAP = {
    2.0: 'thumb_up',       # 竖大拇指 (点赞)
    3.0: 'victory',        # V 手势
    4.0: 'mute',           # 嘘手势
    5.0: 'palm',           # 手掌张开
    11.0: 'okay',          # OK 手势
    12.0: 'thumb_left',    # 大拇指向左
    13.0: 'thumb_right',   # 大拇指向右
    14.0: 'awesome',       # 666 手势
}

# ============ hobot AI 节点输出用 BEST_EFFORT QoS ============
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class GestureActionNode(Node):
    def __init__(self):
        super().__init__('gesture_action_node')

        # ========= 参数 =========
        self.declare_parameter('input_topic', '/hobot_hand_gesture_detection')
        self.declare_parameter('output_topic', '/puppy_action')
        self.declare_parameter('gesture_hold_sec', 0.4)         # 手势消失后多久刹车
        self.declare_parameter('max_move_sec', 3.0)             # 单次移动最长持续
        self.declare_parameter('action_lock_sec', 2.5)          # 离散动作防重复锁
        self.declare_parameter('control_rate_hz', 10.0)         # follow_control 发送频率
        self.declare_parameter('forward_speed', 0.55)
        self.declare_parameter('backward_speed', 0.35)
        self.declare_parameter('turn_speed', 0.75)
        self.declare_parameter('log_interval_sec', 0.5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.gesture_hold_sec = float(self.get_parameter('gesture_hold_sec').value)
        self.max_move_sec = float(self.get_parameter('max_move_sec').value)
        self.action_lock_sec = float(self.get_parameter('action_lock_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.backward_speed = float(self.get_parameter('backward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # ========= 发布者 =========
        self.action_pub = self.create_publisher(String, self.output_topic, 10)

        # ========= 订阅 =========
        self.sub = self.create_subscription(
            PerceptionTargets,
            self.input_topic,
            self.gesture_callback,
            SENSOR_QOS,
        )

        # ========= 状态 =========
        self._lock = threading.Lock()
        self.current_gestures = {}        # {track_id: gesture_name}, 当前帧检测到的所有手
        self.last_gesture_time = 0.0      # 最后一次收到手势的时间
        self.move_action = None           # 当前移动动作: forward/backward/turn_left/turn_right/None
        self.move_until = 0.0             # 移动持续到何时
        self.last_action = None           # 上一次离散动作 (sit/crouch)
        self.action_lock_until = 0.0      # 离散动作防重复锁到期
        self.last_log_time = 0.0
        self.last_control_send = 0.0

        # ========= 控制定时器 =========
        period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'gesture_action_node 启动. input={self.input_topic} '
            f'output={self.output_topic} '
            f'映射(X5): palm(5)→前进 双palm→右转 victory(3)→趴下 '
            f'thumb_up(2)→坐下 okay(11)→后退 thumb_left(12)→左转'
        )

    # -----------------------------------------------------
    # 手势回调: 解析所有 target, 统计每种手势
    # -----------------------------------------------------
    def gesture_callback(self, msg: PerceptionTargets):
        now = time.time()
        gestures = {}

        for target in msg.targets:
            for attr in target.attributes:
                if attr.type != 'gesture':
                    continue
                try:
                    value = float(attr.value)
                except Exception:
                    continue
                name = GESTURE_NAME_MAP.get(value)
                if name is None:
                    continue
                gestures[target.track_id] = name
                break  # 每个 target 只取第一个 gesture 属性

        with self._lock:
            self.current_gestures = gestures
            if gestures:
                self.last_gesture_time = now

        # 决策并发送动作
        self.decide_and_act(now)

    # -----------------------------------------------------
    # 决策: 根据当前手势集合决定动作
    # -----------------------------------------------------
    def decide_and_act(self, now: float):
        with self._lock:
            gestures = dict(self.current_gestures)

        # 统计每种手势数量
        gesture_counts = {}
        for g in gestures.values():
            gesture_counts[g] = gesture_counts.get(g, 0) + 1

        palm_count = gesture_counts.get('palm', 0)
        victory_count = gesture_counts.get('victory', 0)
        thumb_up_count = gesture_counts.get('thumb_up', 0)
        okay_count = gesture_counts.get('okay', 0)
        thumb_left_count = gesture_counts.get('thumb_left', 0)

        # 优先级: 离散动作 (sit/crouch) > 双手 > 单手移动
        # 离散动作: thumb_up → sit, victory → crouch (X5 无 fist, 用 V 代替)
        if thumb_up_count > 0 and now >= self.action_lock_until:
            self.trigger_discrete('sit', 'thumb_up', now)
            return
        if victory_count > 0 and now >= self.action_lock_until:
            self.trigger_discrete('crouch', 'victory', now)
            return

        # 双手 palm → turn_right
        if palm_count >= 2:
            self.start_move('turn_right', now)
            return

        # 单手移动类
        if palm_count == 1:
            self.start_move('forward', now)
            return
        if okay_count > 0:
            self.start_move('backward', now)
            return
        if thumb_left_count > 0:
            self.start_move('turn_left', now)
            return

        # 无手势: 不立即刹车, 由 control_loop 根据超时处理
        # (避免手势检测短暂漏帧导致频繁刹车)

    # -----------------------------------------------------
    # 触发离散动作 (sit/crouch)
    # -----------------------------------------------------
    def trigger_discrete(self, action: str, gesture_name: str, now: float):
        with self._lock:
            self.action_lock_until = now + self.action_lock_sec
            self.move_action = None
            self.move_until = 0.0
            self.last_action = action

        payload = {
            'action': action,
            'source': 'gesture',
            'gesture': gesture_name,
            'timestamp': now,
        }
        self.publish_action(payload)

        if now - self.last_log_time > self.log_interval_sec:
            self.get_logger().info(f'[gesture→{action}] gesture={gesture_name}')
            self.last_log_time = now

    # -----------------------------------------------------
    # 启动/续期移动动作
    # -----------------------------------------------------
    def start_move(self, action: str, now: float):
        # 离散动作锁定期内不响应移动
        if now < self.action_lock_until:
            return

        with self._lock:
            # 切换动作时先刹车
            if self.move_action is not None and self.move_action != action:
                self.publish_brake(now, reason='switch_action')
            self.move_action = action
            self.move_until = now + self.max_move_sec

        move_map = {
            'forward':    ( self.forward_speed,  0.0),
            'backward':   (-self.backward_speed, 0.0),
            'turn_left':  (0.0,  self.turn_speed),
            'turn_right': (0.0, -self.turn_speed),
        }
        fwd, turn = move_map.get(action, (0.0, 0.0))

        # 用 follow_control 模式发送 (ros_udp_bridge 直接转发, sit.py 持续执行)
        payload = {
            'mode': 'follow_control',
            'forward': fwd,
            'turn': turn,
            'source': 'gesture',
            'gesture_action': action,
            'timestamp': now,
        }
        self.publish_action(payload)

        if now - self.last_log_time > self.log_interval_sec:
            self.get_logger().info(
                f'[gesture→{action}] forward={fwd:.2f} turn={turn:.2f}'
            )
            self.last_log_time = now

    # -----------------------------------------------------
    # 控制循环: 持续发 follow_control + 超时刹车
    # -----------------------------------------------------
    def control_loop(self):
        now = time.time()

        with self._lock:
            move_action = self.move_action
            move_until = self.move_until
            last_gesture = self.last_gesture_time

        # 无移动动作, 不做事
        if move_action is None:
            return

        # 手势消失超 gesture_hold_sec → 立即刹车
        # 只看 last_gesture_time, 不依赖 current_gestures 是否为空
        # (hobot 节点在手势消失时可能不发消息, current_gestures 不会被清空)
        if now - last_gesture > self.gesture_hold_sec:
            self.publish_brake(now, reason='gesture_lost')
            with self._lock:
                self.move_action = None
                self.move_until = 0.0
            return

        # 移动超时 (max_move_sec) → 刹车 (安全上限, 防止手势持续误识别)
        if now >= move_until:
            self.publish_brake(now, reason='max_move_timeout')
            with self._lock:
                self.move_action = None
                self.move_until = 0.0
            return

        # 持续发 follow_control (维持 sit.py 的运动)
        # ros_udp_bridge 收到 mode=follow_control 直接转发, sit.py 持续运动
        # 控制量与 start_move 一致, 这里复用
        move_map = {
            'forward':    ( self.forward_speed,  0.0),
            'backward':   (-self.backward_speed, 0.0),
            'turn_left':  (0.0,  self.turn_speed),
            'turn_right': (0.0, -self.turn_speed),
        }
        fwd, turn = move_map.get(move_action, (0.0, 0.0))

        # 控制发送频率, 避免 ros_udp_bridge 过载 (10Hz 已够)
        if now - self.last_control_send < 1.0 / max(self.control_rate_hz, 1.0) - 0.005:
            return

        payload = {
            'mode': 'follow_control',
            'forward': fwd,
            'turn': turn,
            'source': 'gesture',
            'gesture_action': move_action,
            'timestamp': now,
        }
        self.publish_action(payload)
        self.last_control_send = now

    # -----------------------------------------------------
    # 发送刹车 (zero follow_control)
    # -----------------------------------------------------
    def publish_brake(self, now: float, reason: str = ''):
        payload = {
            'mode': 'follow_control',
            'forward': 0.0,
            'turn': 0.0,
            'source': 'gesture',
            'brake_reason': reason,
            'timestamp': now,
        }
        self.publish_action(payload)
        self.get_logger().info(f'[gesture→刹车] reason={reason}')

    # -----------------------------------------------------
    # 发布动作
    # -----------------------------------------------------
    def publish_action(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.action_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前刹车
        try:
            node.publish_brake(time.time(), reason='node_exit')
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
