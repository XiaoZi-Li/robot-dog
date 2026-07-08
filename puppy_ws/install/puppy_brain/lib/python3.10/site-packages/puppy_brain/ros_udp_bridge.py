import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu


class RosUdpBridge(Node):
    def __init__(self):
        super().__init__('ros_udp_bridge')

        self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('action_udp_port', 5005)
        self.declare_parameter('imu_udp_port', 5006)

        self.udp_ip = self.get_parameter('udp_ip').value
        self.action_udp_port = int(self.get_parameter('action_udp_port').value)
        self.imu_udp_port = int(self.get_parameter('imu_udp_port').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.action_sub = self.create_subscription(
            String,
            '/puppy_action',
            self.action_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/ros_robot_controller/imu_raw',
            self.imu_callback,
            10
        )

        self.last_imu_log_time = 0.0

        self.get_logger().info(
            f'ros_udp_bridge started. '
            f'action: /puppy_action -> udp={self.udp_ip}:{self.action_udp_port}, '
            f'imu: /ros_robot_controller/imu_raw -> udp={self.udp_ip}:{self.imu_udp_port}'
        )

    def action_callback(self, msg: String):
        raw = msg.data

        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                if payload.get('mode') == 'follow_control':
                    # 连续控制只转发，不打印
                    self.sock.sendto(
                        raw.encode('utf-8'),
                        (self.udp_ip, self.action_udp_port)
                    )
                    return

                action_cmd = payload.get('action', None)
                source = payload.get('source', 'unknown')
                if action_cmd:
                    self.sock.sendto(
                        action_cmd.encode('utf-8'),
                        (self.udp_ip, self.action_udp_port)
                    )
                    self.get_logger().info(
                        f'UDP action: action=[{action_cmd}] source=[{source}]'
                    )
                    return
        except Exception:
            pass

        if raw:
            self.sock.sendto(
                raw.encode('utf-8'),
                (self.udp_ip, self.action_udp_port)
            )

    def imu_callback(self, msg: Imu):
        payload = {
            'type': 'imu',
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            }
        }

        raw = json.dumps(payload, ensure_ascii=False)
        self.sock.sendto(
            raw.encode('utf-8'),
            (self.udp_ip, self.imu_udp_port)
        )

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_imu_log_time > 1.0:
            self.get_logger().info(
                'UDP IMU: '
                f'acc=({msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, '
                f'{msg.linear_acceleration.z:.3f}) '
                f'gyro=({msg.angular_velocity.x:.3f}, '
                f'{msg.angular_velocity.y:.3f}, '
                f'{msg.angular_velocity.z:.3f})'
            )
            self.last_imu_log_time = now


def main(args=None):
    rclpy.init(args=args)
    bridge = RosUdpBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()