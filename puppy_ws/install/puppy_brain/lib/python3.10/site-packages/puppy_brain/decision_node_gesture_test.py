import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNodeGestureTest(Node):
    def __init__(self):
        super().__init__('decision_node_gesture_test')

        self.declare_parameter('publish_repeat_sec', 1.0)
        self.declare_parameter('log_interval_sec', 0.2)
        self.declare_parameter('gesture_hold_sec', 0.8)

        self.publish_repeat_sec = float(self.get_parameter('publish_repeat_sec').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.gesture_hold_sec = float(self.get_parameter('gesture_hold_sec').value)

        self.action_pub = self.create_publisher(String, '/puppy_action', 10)

        self.gesture_sub = self.create_subscription(
            String,
            '/gesture/result_json',
            self.gesture_callback,
            10
        )

        self.last_action = 'none'
        self.last_source = 'none'
        self.last_send_time = 0.0
        self.last_log_time = 0.0

        self.current_gesture = None
        self.current_gesture_value = None
        self.gesture_expire_time = 0.0

        self.get_logger().info('decision_node_gesture_test started.')

    def gesture_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Invalid gesture JSON: {e}')
            return

        self.current_gesture = payload.get('gesture', None)
        self.current_gesture_value = payload.get('gesture_value', None)
        self.gesture_expire_time = time.time() + self.gesture_hold_sec

        gesture_action = self.map_gesture_to_action(self.current_gesture_value)
        if gesture_action is None:
            return

        now = time.time()
        if now - self.last_log_time > self.log_interval_sec:
            self.get_logger().info(
                f'收到手势: gesture={self.current_gesture}, '
                f'value={self.current_gesture_value}, mapped={gesture_action}'
            )
            self.last_log_time = now

        self.publish_action(
            action=gesture_action,
            source='gesture',
            extra={
                'gesture': self.current_gesture,
                'gesture_value': self.current_gesture_value,
            }
        )

    def map_gesture_to_action(self, gesture_value):
        if gesture_value is None:
            return None

        try:
            value = float(gesture_value)
        except Exception:
            return None

        if value == 3.0:
            return 'stop'
        if value == 4.0:
            return 'sit'
        if value == 5.0:
            return 'stand'

        return None

    def publish_action(self, action: str, source: str, extra=None):
        now = time.time()
        should_publish = (
            action != self.last_action
            or source != self.last_source
            or (now - self.last_send_time) > self.publish_repeat_sec
        )

        if not should_publish:
            return

        payload = {
            'action': action,
            'source': source,
            'timestamp': now,
        }
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.action_pub.publish(msg)

        self.get_logger().info(f'发布动作: {msg.data}')

        self.last_action = action
        self.last_source = source
        self.last_send_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNodeGestureTest()
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