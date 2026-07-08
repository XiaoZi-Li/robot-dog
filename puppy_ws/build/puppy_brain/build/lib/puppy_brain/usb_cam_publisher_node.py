#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""usb_cam_publisher_node.py - USB 摄像头 ROS2 发布节点

后台线程从 USB 摄像头采集帧（YUYV 640x480），发布为:
  - /image_raw/compressed  (sensor_msgs/CompressedImage, JPEG)  供 hobot_codec 消费
  - /image_raw             (sensor_msgs/Image, BGR8)            供 YOLOv5 viewer 消费

发布频率受限于摄像头实际帧率，默认请求 30fps。

用法:
  ros2 run puppy_brain usb_cam_publisher_node
  ros2 run puppy_brain usb_cam_publisher_node --ros-args -p device:=/dev/video0 -p width:=640 -p height:=480 -p fps:=30
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class UsbCamPublisherNode(Node):
    def __init__(self):
        super().__init__('usb_cam_publisher_node')

        # ========= 参数 =========
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 60)        # hobot_codec 输入 jpeg 质量
        self.declare_parameter('out_compressed_topic', '/image_raw/compressed')
        self.declare_parameter('out_raw_topic', '/image_raw')
        self.declare_parameter('log_interval_sec', 5.0)

        self.device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.compressed_topic = self.get_parameter('out_compressed_topic').value
        self.raw_topic = self.get_parameter('out_raw_topic').value
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # ========= 发布者 =========
        self.compressed_pub = self.create_publisher(CompressedImage, self.compressed_topic, 10)
        self.raw_pub = self.create_publisher(Image, self.raw_topic, 10)
        self.bridge = CvBridge()

        # ========= 状态 =========
        self._running = True
        self._frames = 0
        self._t0 = time.time()
        self._last_log = 0.0
        self._cap = None

        # ========= 启动采集线程 =========
        if not self._open_camera():
            self.get_logger().error(f'无法打开摄像头 {self.device},节点将持续重试...')
            self._retry_timer = self.create_timer(2.0, self._retry_open)
            return

        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info(
            f'usb_cam_publisher 启动成功 device={self.device} '
            f'实际={self.width}x{self.height}@{self.fps}fps '
            f'compressed_topic={self.compressed_topic} raw_topic={self.raw_topic}'
        )

    def _open_camera(self):
        self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            return False
        # 显式 YUYV (这颗 icspring 摄像头只支持 YUYV 4:2:2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 读回实际值
        self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = int(self._cap.get(cv2.CAP_PROP_FPS))
        return True

    def _retry_open(self):
        if self._cap is not None and self._cap.isOpened():
            return
        self.get_logger().info(f'重试打开摄像头 {self.device} ...')
        if self._open_camera():
            self._retry_timer.cancel()
            threading.Thread(target=self._capture_loop, daemon=True).start()
            self.get_logger().info('摄像头重连成功,采集线程已启动')

    def _capture_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while self._running and self._cap is not None and self._cap.isOpened():
            ret, frame = self._cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            try:
                # 发布 CompressedImage (JPEG) - 供 hobot_codec 消费
                ok, buf = cv2.imencode('.jpg', frame, encode_param)
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = self.get_clock().now().to_msg()
                    cmsg.format = 'jpeg'
                    cmsg.data = buf.tobytes()
                    self.compressed_pub.publish(cmsg)

                # 发布 raw Image (BGR8) - 供 YOLOv5 viewer 消费
                rmsg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                rmsg.header.stamp = cmsg.header.stamp
                self.raw_pub.publish(rmsg)
            except Exception as e:
                self.get_logger().error(f'发布图像失败: {e}')
                time.sleep(0.05)
                continue

            # 统计 fps
            self._frames += 1
            now = time.time()
            if now - self._last_log > self.log_interval_sec:
                fps = self._frames / (now - self._last_log)
                self.get_logger().info(
                    f'采集 fps={fps:.1f} 实际={self.width}x{self.height}'
                )
                self._frames = 0
                self._last_log = now

    def destroy_node(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCamPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
