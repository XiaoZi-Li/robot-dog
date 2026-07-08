import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('image_width', 960.0)
        self.declare_parameter('image_height', 544.0)

        self.declare_parameter('follow_area_near_stop', 0.55)
        self.declare_parameter('follow_area_far_walk', 0.10)
        self.declare_parameter('min_valid_area_ratio', 0.015)

        self.declare_parameter('center_ratio', 0.50)
        self.declare_parameter('turn_deadband_ratio', 0.07)
        self.declare_parameter('max_turn_error_ratio', 0.22)
        self.declare_parameter('turn_gain', 1.00)

        self.declare_parameter('small_error_ratio', 0.10)
        self.declare_parameter('large_error_ratio', 0.24)

        self.declare_parameter('small_error_forward_keep', 0.96)
        self.declare_parameter('mid_error_forward_keep', 0.78)
        self.declare_parameter('large_error_forward_keep', 0.18)

        self.declare_parameter('min_cruise_forward_small', 0.22)
        self.declare_parameter('min_cruise_forward_mid', 0.16)

        self.declare_parameter('forward_min', 0.0)
        self.declare_parameter('forward_max', 0.95)

        self.declare_parameter('ghost_memory_time', 0.30)
        self.declare_parameter('publish_repeat_sec', 0.15)

        self.declare_parameter('gesture_hold_sec', 0.8)
        self.declare_parameter('follow_default_enabled', True)
        self.declare_parameter('gesture_action_lock_sec', 2.5)
        self.declare_parameter('gesture_stop_lock_sec', 1.0)

        self.declare_parameter('voice_action_lock_sec', 2.5)
        self.declare_parameter('voice_priority_enabled', True)

        # 语音移动指令参数
        self.declare_parameter('voice_move_sec', 2.5)
        self.declare_parameter('voice_forward_speed', 0.55)
        self.declare_parameter('voice_backward_speed', 0.35)
        self.declare_parameter('voice_turn_speed', 0.75)

        self.declare_parameter('control_smooth_alpha', 0.28)
        self.declare_parameter('turn_zero_threshold', 0.04)
        self.declare_parameter('forward_zero_threshold', 0.03)
        self.declare_parameter('debug_print_sec', 0.5)

        self.image_width = float(self.get_parameter('image_width').value)
        self.image_height = float(self.get_parameter('image_height').value)

        self.follow_area_near_stop = float(self.get_parameter('follow_area_near_stop').value)
        self.follow_area_far_walk = float(self.get_parameter('follow_area_far_walk').value)
        self.min_valid_area_ratio = float(self.get_parameter('min_valid_area_ratio').value)

        self.center_ratio = float(self.get_parameter('center_ratio').value)
        self.turn_deadband_ratio = float(self.get_parameter('turn_deadband_ratio').value)
        self.max_turn_error_ratio = float(self.get_parameter('max_turn_error_ratio').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)

        self.small_error_ratio = float(self.get_parameter('small_error_ratio').value)
        self.large_error_ratio = float(self.get_parameter('large_error_ratio').value)

        self.small_error_forward_keep = float(self.get_parameter('small_error_forward_keep').value)
        self.mid_error_forward_keep = float(self.get_parameter('mid_error_forward_keep').value)
        self.large_error_forward_keep = float(self.get_parameter('large_error_forward_keep').value)

        self.min_cruise_forward_small = float(self.get_parameter('min_cruise_forward_small').value)
        self.min_cruise_forward_mid = float(self.get_parameter('min_cruise_forward_mid').value)

        self.forward_min = float(self.get_parameter('forward_min').value)
        self.forward_max = float(self.get_parameter('forward_max').value)

        self.ghost_memory_time = float(self.get_parameter('ghost_memory_time').value)
        self.publish_repeat_sec = float(self.get_parameter('publish_repeat_sec').value)

        self.gesture_hold_sec = float(self.get_parameter('gesture_hold_sec').value)
        self.follow_enabled = bool(self.get_parameter('follow_default_enabled').value)

        self.gesture_action_lock_sec = float(self.get_parameter('gesture_action_lock_sec').value)
        self.gesture_stop_lock_sec = float(self.get_parameter('gesture_stop_lock_sec').value)

        self.voice_action_lock_sec = float(self.get_parameter('voice_action_lock_sec').value)
        self.voice_priority_enabled = bool(self.get_parameter('voice_priority_enabled').value)

        self.voice_move_sec = float(self.get_parameter('voice_move_sec').value)
        self.voice_forward_speed = float(self.get_parameter('voice_forward_speed').value)
        self.voice_backward_speed = float(self.get_parameter('voice_backward_speed').value)
        self.voice_turn_speed = float(self.get_parameter('voice_turn_speed').value)

        self.control_smooth_alpha = float(self.get_parameter('control_smooth_alpha').value)
        self.turn_zero_threshold = float(self.get_parameter('turn_zero_threshold').value)
        self.forward_zero_threshold = float(self.get_parameter('forward_zero_threshold').value)
        self.debug_print_sec = float(self.get_parameter('debug_print_sec').value)

        self.action_pub = self.create_publisher(String, '/puppy_action', 10)

        self.perception_sub = self.create_subscription(
            String, '/perception/result_json', self.perception_callback, 10
        )
        self.gesture_sub = self.create_subscription(
            String, '/gesture/result_json', self.gesture_callback, 10
        )
        self.voice_sub = self.create_subscription(
            String, '/voice/result_json', self.voice_callback, 10
        )

        self.last_send_time = 0.0
        self.last_payload_str = ''
        self.last_person_time = 0.0

        self.current_gesture = None
        self.current_gesture_value = None
        self.gesture_expire_time = 0.0
        self.gesture_lock_until = 0.0

        self.voice_lock_until = 0.0
        self.last_voice_command = None

        # 语音移动指令状态：在 voice_move_until 之前持续发送 voice_move_forward/turn
        self.voice_move_until = 0.0
        self.voice_move_forward = 0.0
        self.voice_move_turn = 0.0

        self.last_forward_cmd = 0.0
        self.last_turn_cmd = 0.0
        self.last_debug_print_time = 0.0

        self.get_logger().info(
            f'decision_node started. follow_enabled={self.follow_enabled}, voice_priority_enabled={self.voice_priority_enabled}'
        )

    def perception_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        now = time.time()

        # 语音移动指令优先级最高：在 voice_move_until 之前持续发固定控制量
        if now < self.voice_move_until:
            forward_cmd, turn_cmd = self.smooth_control(
                self.voice_move_forward, self.voice_move_turn
            )
            voice_move_payload = {
                'mode': 'follow_control',
                'forward': forward_cmd,
                'turn': turn_cmd,
                'source': 'voice_move',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
                'voice_move': True,
            }
            self.publish_payload(voice_move_payload)
            return

        # 语音移动指令刚到期：补发一次零速刹车，避免残留惯性
        if self.voice_move_until > 0.0 and now >= self.voice_move_until:
            self.voice_move_until = 0.0
            self.voice_move_forward = 0.0
            self.voice_move_turn = 0.0
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            stop_payload = {
                'mode': 'follow_control',
                'forward': 0.0,
                'turn': 0.0,
                'source': 'voice_move',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
                'voice_move_end': True,
            }
            self.publish_payload(stop_payload, force=True)
            return

        if now < self.voice_lock_until:
            return
        if now < self.gesture_lock_until:
            return
        if not self.follow_enabled:
            return

        detections = payload.get('detections', [])
        follow_payload = self.decide_follow_control(detections)
        self.publish_payload(follow_payload)
        self.maybe_print_follow_debug(follow_payload)

    def gesture_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        now = time.time()

        if self.voice_priority_enabled and now < self.voice_lock_until:
            self.get_logger().info('gesture ignored because voice lock is active')
            return

        self.current_gesture = payload.get('gesture', None)
        self.current_gesture_value = payload.get('gesture_value', None)
        self.gesture_expire_time = time.time() + self.gesture_hold_sec

        gesture_action = self.map_gesture_to_action(self.current_gesture_value)
        if gesture_action is None:
            return

        if gesture_action == 'follow_on':
            self.follow_enabled = True
            self.gesture_lock_until = now + 0.5

        elif gesture_action == 'follow_off':
            self.follow_enabled = False
            self.gesture_lock_until = now + 0.5
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0

        elif gesture_action == 'sit':
            self.follow_enabled = False
            self.gesture_lock_until = now + self.gesture_action_lock_sec
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0

        elif gesture_action == 'stand':
            self.follow_enabled = True
            self.gesture_lock_until = now + self.gesture_action_lock_sec
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0

        elif gesture_action == 'stop':
            self.follow_enabled = True
            self.gesture_lock_until = now + self.gesture_stop_lock_sec
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0

        elif gesture_action == 'crouch':
            # 趴下: 关闭跟随, 清运动, 加锁
            self.follow_enabled = False
            self.gesture_lock_until = now + self.gesture_action_lock_sec
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            self.voice_move_until = 0.0
            self.voice_move_forward = 0.0
            self.voice_move_turn = 0.0

        elif gesture_action in ('wave', 'bow', 'nod'):
            # 表演动作: 不改 follow 状态, 但加锁避免跟随干扰动作组执行
            self.gesture_lock_until = now + self.gesture_action_lock_sec
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0

        elif gesture_action in ('forward', 'backward', 'turn_left', 'turn_right'):
            # 移动类手势: 复用 voice_move 机制, 持续 voice_move_sec 后自动刹车
            move_map = {
                'forward':    (self.voice_forward_speed, 0.0),
                'backward':   (-self.voice_backward_speed, 0.0),
                'turn_left':  (0.0, self.voice_turn_speed),
                'turn_right': (0.0, -self.voice_turn_speed),
            }
            fwd, turn = move_map[gesture_action]
            self.voice_move_forward = fwd
            self.voice_move_turn = turn
            self.voice_move_until = now + self.voice_move_sec
            self.gesture_lock_until = now + self.voice_move_sec + 0.3
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            self.follow_enabled = True

        gesture_payload = {
            'action': gesture_action,
            'source': 'gesture',
            'timestamp': now,
            'follow_enabled': self.follow_enabled,
            'gesture': self.current_gesture,
            'gesture_value': self.current_gesture_value,
        }
        self.publish_payload(gesture_payload, force=True)

    def voice_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        now = time.time()
        command = payload.get('command', None)

        # 处理离散动作指令（原有）
        if command in ('stand', 'sit', 'stop'):
            self.voice_lock_until = now + self.voice_action_lock_sec
            self.last_voice_command = command
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            self.voice_move_until = 0.0   # 取消任何正在进行的移动指令
            self.voice_move_forward = 0.0
            self.voice_move_turn = 0.0

            if command == 'sit':
                self.follow_enabled = False
            elif command == 'stand':
                self.follow_enabled = True
            elif command == 'stop':
                self.follow_enabled = True

            voice_payload = {
                'action': command,
                'source': 'voice',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
                'voice_lock_until': self.voice_lock_until,
                'raw_voice': payload,
            }
            self.get_logger().info(
                f'[voice] command={command} lock={self.voice_action_lock_sec:.2f}s'
            )
            self.publish_payload(voice_payload, force=True)
            return

        # 处理跟随开关指令
        if command == 'follow_start':
            self.follow_enabled = True
            self.voice_lock_until = now + 0.5
            self.last_voice_command = command
            voice_payload = {
                'action': 'follow_on',
                'source': 'voice',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
            }
            self.get_logger().info('[voice] follow_start -> follow_enabled=True')
            self.publish_payload(voice_payload, force=True)
            return

        if command == 'follow_stop':
            self.follow_enabled = False
            self.voice_lock_until = now + 0.5
            self.last_voice_command = command
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            self.voice_move_until = 0.0
            voice_payload = {
                'action': 'follow_off',
                'source': 'voice',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
            }
            self.get_logger().info('[voice] follow_stop -> follow_enabled=False')
            self.publish_payload(voice_payload, force=True)
            return

        # 处理移动指令：设置持续 voice_move_sec 的固定控制量
        move_map = {
            'forward':    ( self.voice_forward_speed,  0.0),
            'backward':   (-self.voice_backward_speed, 0.0),
            'turn_left':  (0.0,  self.voice_turn_speed),
            'turn_right': (0.0, -self.voice_turn_speed),
        }

        if command in move_map:
            fwd, turn = move_map[command]
            self.voice_move_forward = fwd
            self.voice_move_turn = turn
            self.voice_move_until = now + self.voice_move_sec
            self.voice_lock_until = now + self.voice_move_sec + 0.3
            self.last_voice_command = command
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            self.follow_enabled = True   # 移动期间允许 follow_control 模式输出

            voice_payload = {
                'action': command,
                'source': 'voice',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
                'voice_move_sec': self.voice_move_sec,
                'voice_move_forward': fwd,
                'voice_move_turn': turn,
            }
            self.get_logger().info(
                f'[voice] move {command} fwd={fwd:.2f} turn={turn:.2f} for {self.voice_move_sec:.1f}s'
            )
            self.publish_payload(voice_payload, force=True)
            return

    def map_gesture_to_action(self, gesture_value):
        """手势值 → 动作映射

        hobot gestureDet_8x21 标准输出 1-5;
        6-10 为 32 类模型预留 (未确认, 供模型升级);
        11-14 为 mediapipe 扩展 (操作手册约定).
        """
        if gesture_value is None:
            return None
        try:
            value = float(gesture_value)
        except Exception:
            return None

        # === hobot 标准手势 (1-5, 已验证) ===
        if value == 1.0:   # palm 手掌张开
            return 'follow_on'
        if value == 2.0:   # fist 握拳
            return 'follow_off'
        if value == 3.0:   # okay
            return 'stop'
        if value == 4.0:   # thumb_up 点赞
            return 'sit'
        if value == 5.0:   # index_finger 竖食指
            return 'stand'

        # === hobot 32类预留扩展 (6-10, 模型升级后可能输出) ===
        if value == 6.0:   # thumb_down
            return 'crouch'
        if value == 7.0:   # iloveyou
            return 'wave'
        if value == 8.0:   # rock
            return 'bow'
        if value == 9.0:   # vulcan
            return 'nod'
        if value == 10.0:  # pinch
            return 'stop'

        # === mediapipe 扩展手势 (11-14, 操作手册约定) ===
        if value == 11.0:  # okay_mp
            return 'crouch'
        if value == 12.0:  # thumb_left
            return 'turn_left'
        if value == 13.0:  # thumb_right
            return 'turn_right'
        if value == 14.0:  # awesome
            return 'follow_on'

        return None

    def clamp(self, v, vmin, vmax):
        return max(vmin, min(vmax, v))

    def smooth_control(self, target_forward, target_turn):
        alpha = self.control_smooth_alpha

        smoothed_forward = (1.0 - alpha) * self.last_forward_cmd + alpha * target_forward
        smoothed_turn = (1.0 - alpha) * self.last_turn_cmd + alpha * target_turn

        if abs(smoothed_forward) < self.forward_zero_threshold:
            smoothed_forward = 0.0
        if abs(smoothed_turn) < self.turn_zero_threshold:
            smoothed_turn = 0.0

        self.last_forward_cmd = smoothed_forward
        self.last_turn_cmd = smoothed_turn
        return smoothed_forward, smoothed_turn

    def select_best_person(self, detections):
        best_person = None
        best_area = 0.0

        for det in detections:
            if det.get('name') != 'person':
                continue

            bbox = det.get('bbox', None)
            if not bbox or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = bbox
            box_w = max(0.0, x2 - x1)
            box_h = max(0.0, y2 - y1)
            if box_w <= 0.0 or box_h <= 0.0:
                continue

            area_ratio = (box_w * box_h) / (self.image_width * self.image_height)
            if area_ratio < self.min_valid_area_ratio:
                continue

            if area_ratio > best_area:
                best_area = area_ratio
                best_person = (x1, y1, x2, y2, area_ratio)

        return best_person

    def decide_follow_control(self, detections):
        now = time.time()
        best_person = self.select_best_person(detections)

        if best_person is None:
            time_since_last_seen = now - self.last_person_time

            if time_since_last_seen < self.ghost_memory_time:
                forward_cmd, turn_cmd = self.smooth_control(0.0, 0.0)
            else:
                self.last_forward_cmd = 0.0
                self.last_turn_cmd = 0.0
                forward_cmd, turn_cmd = 0.0, 0.0

            return {
                'mode': 'follow_control',
                'forward': forward_cmd,
                'turn': turn_cmd,
                'source': 'follow',
                'timestamp': now,
                'follow_enabled': self.follow_enabled,
                'lost_target': True,
            }

        x1, y1, x2, y2, area_ratio = best_person
        x_center = (x1 + x2) / 2.0
        cx_ratio = x_center / self.image_width

        self.last_person_time = now

        error = cx_ratio - self.center_ratio
        abs_error = abs(error)

        if abs_error < self.turn_deadband_ratio:
            raw_turn = 0.0
        else:
            effective_error = abs_error - self.turn_deadband_ratio
            norm_error = effective_error / self.max_turn_error_ratio
            norm_error = self.clamp(norm_error, 0.0, 1.0)
            turn_mag = self.turn_gain * norm_error
            raw_turn = turn_mag if error < 0 else -turn_mag

        raw_turn = self.clamp(raw_turn, -1.0, 1.0)

        if area_ratio >= self.follow_area_near_stop:
            raw_forward = 0.0
        elif area_ratio <= self.follow_area_far_walk:
            raw_forward = self.forward_max
        else:
            ratio = (self.follow_area_near_stop - area_ratio) / (
                self.follow_area_near_stop - self.follow_area_far_walk
            )
            ratio = self.clamp(ratio, 0.0, 1.0)
            ratio = ratio * ratio
            raw_forward = self.forward_min + (self.forward_max - self.forward_min) * ratio

        if abs_error < self.small_error_ratio:
            raw_forward = raw_forward * self.small_error_forward_keep
            if raw_turn != 0.0 and raw_forward < self.min_cruise_forward_small:
                raw_forward = self.min_cruise_forward_small
        elif abs_error < self.large_error_ratio:
            raw_forward = raw_forward * self.mid_error_forward_keep
            if raw_turn != 0.0 and raw_forward < self.min_cruise_forward_mid:
                raw_forward = self.min_cruise_forward_mid
        else:
            raw_forward = raw_forward * self.large_error_forward_keep

        raw_forward = self.clamp(raw_forward, 0.0, 1.0)
        forward_cmd, turn_cmd = self.smooth_control(raw_forward, raw_turn)

        return {
            'mode': 'follow_control',
            'forward': forward_cmd,
            'turn': turn_cmd,
            'source': 'follow',
            'timestamp': now,
            'follow_enabled': self.follow_enabled,
            'cx_ratio': cx_ratio,
            'area_ratio': area_ratio,
            'lost_target': False,
        }

    def maybe_print_follow_debug(self, payload: dict):
        now = time.time()
        if now - self.last_debug_print_time < self.debug_print_sec:
            return
        if payload.get('mode') != 'follow_control':
            return

        lost_target = payload.get('lost_target', False)
        forward = float(payload.get('forward', 0.0))
        turn = float(payload.get('turn', 0.0))

        if lost_target and abs(forward) < 0.01 and abs(turn) < 0.01:
            return

        self.last_debug_print_time = now

        if lost_target:
            self.get_logger().info(
                f'[follow] lost_target forward={forward:.2f} turn={turn:.2f}'
            )
        else:
            cx_ratio = float(payload.get('cx_ratio', 0.0))
            area_ratio = float(payload.get('area_ratio', 0.0))
            self.get_logger().info(
                f'[follow] target=1 cx={cx_ratio:.3f} area={area_ratio:.3f} '
                f'forward={forward:.2f} turn={turn:.2f}'
            )

    def publish_payload(self, payload: dict, force=False):
        now = time.time()
        payload_str = json.dumps(payload, ensure_ascii=False)

        should_publish = force or (
            payload_str != self.last_payload_str
            or (now - self.last_send_time) > self.publish_repeat_sec
        )

        if not should_publish:
            return

        msg = String()
        msg.data = payload_str
        self.action_pub.publish(msg)

        self.last_payload_str = payload_str
        self.last_send_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
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
