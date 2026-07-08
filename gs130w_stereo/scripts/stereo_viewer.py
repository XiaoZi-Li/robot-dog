#!/usr/bin/env python3
"""双拼合成 viewer（修正：双 source 都是双拼图）.

订阅:
  /image_combine_jpeg                  (1280x2176: 下半=左眼, 上半=右眼)
  /StereoNetNode/stereonet_visual_jpeg (640x704: 双眼深度叠加, 每半 640x352)

输出 1280x1088 = 左半 640x1088（左眼原图） + 右半 640x1088（深度图 resize）
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import threading


class StereoViewer(Node):
    TARGET_H = 1088
    HALF_W = 640

    def __init__(self):
        super().__init__('stereo_viewer')
        self.lock = threading.Lock()
        self.left_bgr = None    # 640x1088 BGR
        self.depth_bgr = None   # 640x1088 BGR

        self.create_subscription(CompressedImage, '/image_combine_jpeg',
                                 self.cb_left, 1)
        self.create_subscription(CompressedImage, '/StereoNetNode/stereonet_visual_jpeg',
                                 self.cb_depth, 1)
        self.pub = self.create_publisher(CompressedImage,
                                         '/stereo_view/composite_jpeg', 1)
        self.get_logger().info(
            'stereo_viewer subscribed → composite 1280x1088')

    @staticmethod
    def _decode(msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    @staticmethod
    def _take_bottom_half(img):
        h, w = img.shape[:2]
        if h >= w:
            return img[h // 2:, :, :].copy()
        return img[:, :w // 2, :].copy()

    def cb_left(self, msg):
        img = self._decode(msg)
        if img is None:
            return
        eye = self._take_bottom_half(img)
        eye = cv2.resize(eye, (self.HALF_W, self.TARGET_H))
        with self.lock:
            self.left_bgr = eye
            depth = self.depth_bgr
        self._compose_and_pub(eye, depth)

    def cb_depth(self, msg):
        img = self._decode(msg)
        if img is None:
            return
        depth = self._take_bottom_half(img)
        depth_big = cv2.resize(depth, (self.HALF_W, self.TARGET_H),
                                interpolation=cv2.INTER_LINEAR)
        with self.lock:
            self.depth_bgr = depth_big
            left = self.left_bgr
        self._compose_and_pub(left, depth_big)

    def _compose_and_pub(self, left, depth):
        if left is None or depth is None:
            return
        try:
            comp = np.hstack([left, depth])
            cv2.line(comp, (self.HALF_W, 0), (self.HALF_W, self.TARGET_H),
                     (0, 255, 255), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(comp, 'LEFT EYE (raw)', (20, 50), font, 1.2, (0, 255, 0), 3)
            cv2.putText(comp, 'DEPTH (stereonet BPU)',
                        (self.HALF_W + 20, 50), font, 1.2, (0, 255, 0), 3)
            cv2.putText(comp, 'red=near  blue=far',
                        (self.HALF_W + 20, 95), font, 0.7, (0, 255, 255), 2)
            ok, buf = cv2.imencode('.jpg', comp,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not ok:
                return
            out = CompressedImage()
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'compose: {e}')


def main():
    rclpy.init()
    n = StereoViewer()
    rclpy.spin(n)


if __name__ == '__main__':
    main()