import os
import sys
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# 把幻尔 SDK 路径加入 Python 搜索路径
SDK_DIR = '/app/pydev_demo/puppypi_control'
if SDK_DIR not in sys.path:
    sys.path.append(SDK_DIR)

from ros_robot_controller_sdk import Board


class ImuNodeRos2(Node):
    def __init__(self):
        super().__init__('imu_node_ros2')

        self.declare_parameter('topic_name', '/ros_robot_controller/imu_raw')
        self.declare_parameter('publish_hz', 50.0)

        self.topic_name = str(self.get_parameter('topic_name').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.imu_pub = self.create_publisher(Imu, self.topic_name, 10)

        self.board = Board()
        self.board.enable_reception()

        self.data_lock = Lock()
        self.last_log_time = 0.0

        timer_period = 1.0 / self.publish_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'imu_node_ros2 started. publish topic={self.topic_name}, hz={self.publish_hz}'
        )

    def timer_callback(self):
        """
        按照官方 ROS1 节点的思路：
        data[0]: ax
        data[1]: ay
        data[2]: az
        data[3]: gx
        data[4]: gy
        data[5]: gz
        """
        try:
            data = self.board.get_imu()
        except Exception as e:
            now = time.time()
            if now - self.last_log_time > 1.0:
                self.get_logger().warn(f'get_imu() exception: {e}')
                self.last_log_time = now
            return

        if data is None:
            now = time.time()
            if now - self.last_log_time > 1.0:
                self.get_logger().warn('get_imu() returned None')
                self.last_log_time = now
            return

        if not isinstance(data, (list, tuple)) or len(data) < 6:
            now = time.time()
            if now - self.last_log_time > 1.0:
                self.get_logger().warn(f'get_imu() invalid data: {data}')
                self.last_log_time = now
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # 线加速度
        imu_msg.linear_acceleration.x = float(data[0])
        imu_msg.linear_acceleration.y = float(data[1])
        imu_msg.linear_acceleration.z = float(data[2])

        # 角速度
        imu_msg.angular_velocity.x = float(data[3])
        imu_msg.angular_velocity.y = float(data[4])
        imu_msg.angular_velocity.z = float(data[5])

        # 当前没有可靠四元数，就先置零
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        self.imu_pub.publish(imu_msg)

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                'publish imu: '
                f'acc=({imu_msg.linear_acceleration.x:.3f}, '
                f'{imu_msg.linear_acceleration.y:.3f}, '
                f'{imu_msg.linear_acceleration.z:.3f}) '
                f'gyro=({imu_msg.angular_velocity.x:.3f}, '
                f'{imu_msg.angular_velocity.y:.3f}, '
                f'{imu_msg.angular_velocity.z:.3f})'
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ImuNodeRos2()
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