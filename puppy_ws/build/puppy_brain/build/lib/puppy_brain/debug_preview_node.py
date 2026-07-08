#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import queue
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage


class DebugPreviewNode(Node):
    def __init__(self):
        super().__init__('debug_preview_node')

        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('perception_topic', '/perception/result_json')
        self.declare_parameter('gesture_topic', '/gesture/result_json')
        self.declare_parameter('action_topic', '/puppy_action')
        self.declare_parameter('window_name', 'Puppy Debug Preview')
        self.declare_parameter('display_scale', 1.0)
        self.declare_parameter('max_fps', 15.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.perception_topic = self.get_parameter('perception_topic').value
        self.gesture_topic = self.get_parameter('gesture_topic').value
        self.action_topic = self.get_parameter('action_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.display_scale = float(self.get_parameter('display_scale').value)
        self.max_fps = float(self.get_parameter('max_fps').value)

        self.frame_queue = queue.Queue(maxsize=2)

        self.subscription_created = False
        self.image_sub = None
        self.last_sub_log_time = 0.0

        self.last_action = 'none'
        self.last_action_source = 'none'
        self.last_follow_enabled = None
        self.last_action_time = 0.0

        self.last_gesture = 'none'
        self.last_gesture_value = None
        self.last_gesture_time = 0.0

        self.last_detections = []
        self.last_perception_time = 0.0

        self.detect_timer = self.create_timer(1.0, self.try_create_image_subscription)

        self.perception_sub = self.create_subscription(
            String,
            self.perception_topic,
            self.perception_callback,
            10
        )

        self.gesture_sub = self.create_subscription(
            String,
            self.gesture_topic,
            self.gesture_callback,
            10
        )

        self.action_sub = self.create_subscription(
            String,
            self.action_topic,
            self.action_callback,
            10
        )

        self._running = True
        self.render_thread_obj = threading.Thread(target=self.render_thread, daemon=True)
        self.render_thread_obj.start()

        self.get_logger().info(
            f'debug_preview_node started. image={self.image_topic}, '
            f'perception={self.perception_topic}, gesture={self.gesture_topic}, action={self.action_topic}'
        )

    # -----------------------------------------------------
    # 动态识别 /image 类型
    # -----------------------------------------------------
    def try_create_image_subscription(self):
        if self.subscription_created:
            return

        topics = dict(self.get_topic_names_and_types())

        if self.image_topic not in topics:
            now = time.time()
            if now - self.last_sub_log_time > 2.0:
                self.get_logger().info(f'waiting topic {self.image_topic} ...')
                self.last_sub_log_time = now
            return

        topic_types = topics[self.image_topic]

        if 'sensor_msgs/msg/CompressedImage' in topic_types:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_callback,
                10
            )
            self.subscription_created = True
            self.get_logger().info(
                f'subscribed {self.image_topic} as sensor_msgs/msg/CompressedImage'
            )
            return

        if 'sensor_msgs/msg/Image' in topic_types:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                10
            )
            self.subscription_created = True
            self.get_logger().info(
                f'subscribed {self.image_topic} as sensor_msgs/msg/Image'
            )
            return

        self.get_logger().warn(
            f'topic {self.image_topic} exists but unsupported types: {topic_types}'
        )

    # -----------------------------------------------------
    # 图像订阅
    # -----------------------------------------------------
    def compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            self.push_frame(bgr)
        except Exception as e:
            self.get_logger().error(f'compressed_image_callback error: {e}')

    def raw_image_callback(self, msg: Image):
        try:
            h = msg.height
            w = msg.width
            enc = msg.encoding.lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if enc == 'bgr8':
                bgr = data.reshape((h, w, 3)).copy()
            elif enc == 'rgb8':
                rgb = data.reshape((h, w, 3)).copy()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif enc in ['mono8', '8uc1']:
                gray = data.reshape((h, w)).copy()
                bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                self.get_logger().warn(f'unsupported raw image encoding: {msg.encoding}')
                return

            self.push_frame(bgr)
        except Exception as e:
            self.get_logger().error(f'raw_image_callback error: {e}')

    def push_frame(self, bgr: np.ndarray):
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
        self.frame_queue.put(bgr)

    # -----------------------------------------------------
    # 业务 topic
    # -----------------------------------------------------
    def perception_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.last_detections = payload.get('detections', [])
            self.last_perception_time = time.time()
        except Exception as e:
            self.get_logger().error(f'perception_callback error: {e}')

    def gesture_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.last_gesture = str(payload.get('gesture', 'none'))
            self.last_gesture_value = payload.get('gesture_value', None)
            self.last_gesture_time = time.time()
        except Exception as e:
            self.get_logger().error(f'gesture_callback error: {e}')

    def action_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.last_action = str(payload.get('action', 'none'))
            self.last_action_source = str(payload.get('source', 'unknown'))
            self.last_follow_enabled = payload.get('follow_enabled', None)
            self.last_action_time = time.time()
        except Exception:
            # 兼容旧版纯字符串 action
            self.last_action = msg.data
            self.last_action_source = 'legacy'
            self.last_action_time = time.time()

    # -----------------------------------------------------
    # 绘制
    # -----------------------------------------------------
    def draw_overlay(self, frame: np.ndarray) -> np.ndarray:
        vis = frame.copy()

        h, w = vis.shape[:2]

        panel_h = 130
        cv2.rectangle(vis, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.rectangle(vis, (0, 0), (w, panel_h), (80, 80, 80), 1)

        follow_text = 'unknown'
        if self.last_follow_enabled is True:
            follow_text = 'ON'
        elif self.last_follow_enabled is False:
            follow_text = 'OFF'

        gesture_text = f'{self.last_gesture}'
        if self.last_gesture_value is not None:
            gesture_text += f' ({self.last_gesture_value})'

        lines = [
            f'ACTION: {self.last_action}',
            f'SOURCE: {self.last_action_source}',
            f'GESTURE: {gesture_text}',
            f'FOLLOW: {follow_text}',
            f'DETECTIONS: {len(self.last_detections)}',
        ]

        y = 28
        for line in lines:
            cv2.putText(
                vis, line, (12, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
            )
            y += 24

        for det in self.last_detections:
            try:
                name = det.get('name', 'obj')
                bbox = det.get('bbox', None)
                score = det.get('score', None)

                if not bbox or len(bbox) != 4:
                    continue

                x1, y1, x2, y2 = [int(v) for v in bbox]

                color = (0, 255, 255) if name == 'person' else (255, 255, 0)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

                label = name
                if score is not None:
                    try:
                        label += f' {float(score):.2f}'
                    except Exception:
                        pass

                cv2.putText(
                    vis, label, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA
                )
            except Exception:
                continue

        return vis

    # -----------------------------------------------------
    # 渲染线程
    # -----------------------------------------------------
    def render_thread(self):
        last_show_time = 0.0

        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        except Exception as e:
            self.get_logger().error(f'cv2.namedWindow failed: {e}')
            return

        while self._running:
            try:
                frame = self.frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            now = time.time()
            if self.max_fps > 0:
                min_interval = 1.0 / self.max_fps
                if now - last_show_time < min_interval:
                    continue
                last_show_time = now

            try:
                vis = self.draw_overlay(frame)

                if self.display_scale != 1.0:
                    vis = cv2.resize(
                        vis,
                        None,
                        fx=self.display_scale,
                        fy=self.display_scale,
                        interpolation=cv2.INTER_LINEAR
                    )

                cv2.imshow(self.window_name, vis)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'render_thread error: {e}')
                time.sleep(0.05)

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DebugPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
