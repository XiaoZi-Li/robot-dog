#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yolov5_mjpeg_server.py - YOLOv5 检测 + 手势叠加 + MJPEG HTTP 推流节点

订阅:
  - /image_raw/compressed   (sensor_msgs/CompressedImage)  显示底图
  - /perception/result_json (std_msgs/String, JSON)         叠加 YOLO 检测框
  - /hobot_hand_gesture_detection (ai_msgs/PerceptionTargets) 叠加手势标签

HTTP 路由 (端口 8093):
  /          查看页 (内嵌 <img>)
  /stream    MJPEG 流 (叠加 YOLO 框 + 手势 + 当前动作)
  /snapshot  当前帧 JPEG 截图
  /health    状态文本

后台线程订阅图像, 维护最新帧; HTTP handler 取最新帧叠加检测结果后编码 JPEG 推送。
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from http.server import HTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge


# ============ hobot AI 节点输出用 BEST_EFFORT QoS ============
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

GESTURE_NAME_MAP = {
    1.0: 'palm',
    2.0: 'fist',
    3.0: 'okay',
    4.0: 'thumb_up',
    5.0: 'index_finger',
}

# COCO 80 类前 10 个常用 (完整列表从文件加载, 这里仅 fallback)
COCO_NAMES_FALLBACK = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light'
]


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    pass


class Yolov5MjpegServerNode(Node):
    def __init__(self):
        super().__init__('yolov5_mjpeg_server')

        # ========= 参数 =========
        self.declare_parameter('port', 8093)
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('perception_topic', '/perception/result_json')
        self.declare_parameter('gesture_topic', '/hobot_hand_gesture_detection')
        self.declare_parameter('coco_names_file', '/app/pydev_demo/07_usb_camera_sample/coco_classes.names')
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_frame_age_sec', 0.5)    # 显示帧最大延迟
        self.declare_parameter('log_interval_sec', 5.0)

        self.port = int(self.get_parameter('port').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.perception_topic = self.get_parameter('perception_topic').value
        self.gesture_topic = self.get_parameter('gesture_topic').value
        self.coco_names_file = self.get_parameter('coco_names_file').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # ========= COCO 类名 =========
        self.coco_names = self._load_coco_names()

        # ========= 状态 (帧 + 检测 + 手势) =========
        self._lock = threading.Lock()
        self.latest_frame = None           # BGR ndarray
        self.latest_frame_time = 0.0
        self.latest_detections = []        # [{name, score, bbox:[x1,y1,x2,y2]}, ...]
        self.latest_detections_time = 0.0
        self.latest_gestures = []          # [{track_id, name, value}, ...]
        self.latest_gestures_time = 0.0

        self.bridge = CvBridge()
        self._frames_rendered = 0
        self._t0 = time.time()
        self._last_log = 0.0

        # ========= 订阅 =========
        # 图像: 优先订阅 compressed, 兼容 raw
        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, 10
        )
        # 也订阅 raw 以防万一
        raw_topic = self.image_topic.replace('/compressed', '')
        if raw_topic != self.image_topic:
            self.raw_sub = self.create_subscription(
                Image, raw_topic, self.raw_image_callback, 10
            )

        self.perception_sub = self.create_subscription(
            String, self.perception_topic, self.perception_callback, 10
        )
        self.gesture_sub = self.create_subscription(
            PerceptionTargets, self.gesture_topic, self.gesture_callback, SENSOR_QOS
        )

        self.get_logger().info(
            f'yolov5_mjpeg_server 启动. port={self.port} '
            f'image={self.image_topic} perception={self.perception_topic} '
            f'gesture={self.gesture_topic} coco_names={len(self.coco_names)}类'
        )

        # ========= 启动 HTTP server (后台线程) =========
        self._http_server = ThreadingHTTPServer(('0.0.0.0', self.port), self._make_handler())
        threading.Thread(target=self._http_server.serve_forever, daemon=True).start()
        self.get_logger().info(f'HTTP MJPEG server 已就绪: http://0.0.0.0:{self.port}/')

    def _load_coco_names(self):
        try:
            with open(self.coco_names_file, 'r', encoding='utf-8') as f:
                names = [line.strip() for line in f if line.strip()]
            if names:
                return names
        except Exception as e:
            self.get_logger().warn(f'加载 COCO 类名失败 {self.coco_names_file}: {e}')
        return COCO_NAMES_FALLBACK

    # -----------------------------------------------------
    # 图像回调
    # -----------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            with self._lock:
                self.latest_frame = bgr
                self.latest_frame_time = time.time()
        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    def raw_image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                # 只在没有 compressed 帧或 raw 更新时用
                if self.latest_frame is None:
                    self.latest_frame = bgr
                    self.latest_frame_time = time.time()
        except Exception as e:
            self.get_logger().error(f'raw_image_callback error: {e}')

    # -----------------------------------------------------
    # 感知回调 (YOLOv5 检测结果)
    # -----------------------------------------------------
    def perception_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            detections = payload.get('detections', [])
            with self._lock:
                self.latest_detections = detections
                self.latest_detections_time = time.time()
        except Exception as e:
            self.get_logger().error(f'perception_callback error: {e}')

    # -----------------------------------------------------
    # 手势回调 (解析所有 target)
    # -----------------------------------------------------
    def gesture_callback(self, msg: PerceptionTargets):
        gestures = []
        for target in msg.targets:
            for attr in target.attributes:
                if attr.type != 'gesture':
                    continue
                try:
                    value = float(attr.value)
                except Exception:
                    continue
                name = GESTURE_NAME_MAP.get(value, f'unknown_{value}')
                gestures.append({
                    'track_id': target.track_id,
                    'name': name,
                    'value': value,
                })
                break
        with self._lock:
            self.latest_gestures = gestures
            self.latest_gestures_time = time.time()

    # -----------------------------------------------------
    # 渲染一帧 (叠加 YOLO 框 + 手势标签)
    # -----------------------------------------------------
    def render_frame(self) -> bytes:
        with self._lock:
            frame = None
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
            detections = list(self.latest_detections)
            gestures = list(self.latest_gestures)
            det_time = self.latest_detections_time
            ges_time = self.latest_gestures_time

        if frame is None:
            # 无帧时返回占位图
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(frame, 'Waiting for camera...', (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 1)

        now = time.time()

        # 叠加 YOLO 检测框 (检测结果 1 秒内有效)
        if det_time > 0 and now - det_time < 1.0:
            for det in detections:
                name = det.get('name', '?')
                score = det.get('score', 0.0)
                bbox = det.get('bbox', None)
                if not bbox or len(bbox) != 4:
                    continue
                x1, y1, x2, y2 = [int(v) for v in bbox]
                # 不同类别不同颜色 (用类别 id hash)
                cls_id = self._get_class_id(name)
                color = self._color_for_class(cls_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f'{name} {score:.2f}'
                self._draw_label(frame, label, (x1, y1 - 6), color)

        # 叠加手势标签 (手势结果 1 秒内有效)
        if ges_time > 0 and now - ges_time < 1.0 and gestures:
            y_offset = 30
            for g in gestures:
                name = g.get('name', '?')
                tid = g.get('track_id', '?')
                text = f'Gesture: {name} (id={tid})'
                cv2.putText(frame, text, (10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                y_offset += 30

        # 顶部状态栏
        status = (
            f'YOLOv5: {len(detections) if now - det_time < 1.0 else "stale"} det | '
            f'Gesture: {len(gestures) if now - ges_time < 1.0 else 0}'
        )
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 24), (0, 0, 0), -1)
        cv2.putText(frame, status, (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)

        # 编码 JPEG
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return b''
        return buf.tobytes()

    def _get_class_id(self, name: str) -> int:
        try:
            return self.coco_names.index(name)
        except ValueError:
            return 0

    def _color_for_class(self, cls_id: int):
        # 简单 hash 出颜色
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (0, 255, 255), (255, 0, 255), (128, 0, 0), (0, 128, 0),
        ]
        return colors[cls_id % len(colors)]

    def _draw_label(self, img, text, org, color):
        x, y = org
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y = max(y, th + 2)
        cv2.rectangle(img, (x, y - th - 2), (x + tw + 2, y + 2), color, -1)
        cv2.putText(img, text, (x + 1, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # -----------------------------------------------------
    # HTTP handler 工厂
    # -----------------------------------------------------
    def _make_handler(self):
        node = self

        INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>YOLOv5 + 手势 实时画面</title>
<style>
body { margin:0; background:#111; color:#eee; font-family:sans-serif;
       display:flex; flex-direction:column; align-items:center; }
h2 { margin:14px 0 6px; }
.bar { font-size:13px; color:#9bd; margin-bottom:10px; }
img { border:2px solid #333; background:#000; max-width:95vw; max-height:82vh; }
.btns { margin:10px 0 24px; }
button { padding:6px 14px; margin:0 6px; font-size:14px; cursor:pointer; }
</style></head><body>
<h2>YOLOv5 检测 + 手势识别 实时画面</h2>
<div class="bar">MJPEG: /stream | 截图: /snapshot | 健康: /health</div>
<img id="cam" src="/stream" alt="等待画面...">
<div class="btns">
<button onclick="document.getElementById('cam').src='/stream?t='+Date.now()">重连</button>
<button onclick="location.href='/snapshot'">截图</button>
</div></body></html>
"""

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/' or self.path.startswith('/?'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(INDEX_HTML.encode('utf-8'))
                    return

                if self.path.startswith('/health'):
                    with node._lock:
                        det_cnt = len(node.latest_detections)
                        ges_cnt = len(node.latest_gestures)
                        frame_age = time.time() - node.latest_frame_time if node.latest_frame_time else -1
                    msg = (f'OK port={node.port} '
                           f'frame_age={frame_age:.2f}s '
                           f'det={det_cnt} gesture={ges_cnt} '
                           f'coco={len(node.coco_names)}\n')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/plain; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(msg.encode('utf-8'))
                    return

                if self.path.startswith('/snapshot'):
                    jpg = node.render_frame()
                    if not jpg:
                        self.send_response(503)
                        self.end_headers()
                        self.wfile.write(b'no frame')
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    return

                if self.path.startswith('/stream'):
                    self.send_response(200)
                    self.send_header('Content-Type',
                                     'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Pragma', 'no-cache')
                    self.end_headers()
                    try:
                        while True:
                            jpg = node.render_frame()
                            if not jpg:
                                time.sleep(0.05)
                                continue
                            self.wfile.write(b'--frame\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                            # 约 15-20 fps 渲染, 避免空转
                            time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    except Exception:
                        return
                    return

                self.send_response(404)
                self.end_headers()

            def log_message(self, fmt, *args):
                pass

        return Handler

    def destroy_node(self):
        try:
            self._http_server.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Yolov5MjpegServerNode()
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
