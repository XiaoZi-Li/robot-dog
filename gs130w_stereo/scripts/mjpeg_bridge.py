#!/usr/bin/env python3
"""标准 MJPEG HTTP 桥: 把 ROS 压缩图像 topic 转成浏览器原生可看的 multipart/x-mixed-replace 流.

支持 --region {top,bottom,full}: mipi_cam dual_combine=2 把左右眼上下叠成 1280×2176，
  --region bottom = 下半（左眼，1280×1088）
  --region top    = 上半（右眼，1280×1088）
  --region full   = 整张不切
"""
import sys, json, socket, threading, time, struct, os
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """每个 HTTP 客户端一个线程，并发"""
from io import BytesIO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage


class MjpegBridge(Node):
    """订阅单个 jpeg topic，把最新一帧（可选 split + flip）缓存起来以备 HTTP server 取"""
    def __init__(self, topic, region='full', vflip=False, hflip=False):
        super().__init__(f'mjpeg_bridge_{topic.strip("/").replace("/", "_")}')
        self.topic = topic
        self.region = region
        self.vflip = vflip
        self.hflip = hflip
        self.frame = None
        self.frame_lock = threading.Lock()
        self.subscribers = set()
        self.subs_lock = threading.Lock()
        self.fps = 0.0
        self.frames = 0
        self.t0 = time.time()
        # 用 BEST_EFFORT + KEEP_LAST(10) 订阅
        # hobot_codec 编码出的 jpeg topic 默认是 sensor_data QoS (BEST_EFFORT)
        # RELIABLE 订阅者收不到 BEST_EFFORT 发布者的数据（DDS QoS 不兼容）
        # BEST_EFFORT 订阅者能同时兼容 RELIABLE 和 BEST_EFFORT 发布者
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(CompressedImage, topic, self.cb, sensor_qos)
        self.get_logger().info(f'MJPEG bridge subscribed to {topic} region={region} vflip={vflip} hflip={hflip}')

    def _decode_split_encode(self, raw):
        """JPEG bytes -> region crop + flip -> JPEG bytes. 用 OpenCV（板端可用 + 速度快）"""
        try:
            import cv2
            import numpy as np
        except ImportError:
            # fallback: Pillow
            from PIL import Image
            img = Image.open(BytesIO(raw))
            w, h = img.size
            if self.region == 'top':
                img = img.crop((0, 0, w, h // 2))
            elif self.region == 'bottom':
                img = img.crop((0, h // 2, w, h))
            if self.hflip:
                img = img.transpose(Image.FLIP_LEFT_RIGHT)
            if self.vflip:
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
            buf = BytesIO()
            img.save(buf, format='JPEG', quality=85)
            return buf.getvalue()
        arr = np.frombuffer(raw, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return raw
        h, w = img.shape[:2]
        if self.region == 'top':
            img = img[0:h // 2, :, :]
        elif self.region == 'bottom':
            img = img[h // 2:h, :, :]
        # flip（颠倒图像修复）
        if self.hflip:
            img = cv2.flip(img, 1)   # 水平翻转
        if self.vflip:
            img = cv2.flip(img, 0)   # 垂直翻转
        # 降低 jpeg quality 提升帧率（85 -> 75 视觉差异小，编码快）
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return raw
        return buf.tobytes()

    def cb(self, msg):
        raw = bytes(msg.data)
        # region 切割 或 flip 都需要重编码
        if self.region != 'full' or self.vflip or self.hflip:
            raw = self._decode_split_encode(raw)
        with self.frame_lock:
            self.frame = raw
        self.frames += 1
        now = time.time()
        if now - self.t0 > 5:
            fps = self.frames / (now - self.t0)
            self.get_logger().info(f'fps={fps:.1f}')
            self.fps = fps
            self.frames = 0
            self.t0 = now
        with self.subs_lock:
            dead = set()
            for c in self.subscribers:
                try:
                    c.sendall(b'x')
                except Exception:
                    dead.add(c)
            self.subscribers -= dead

    def wait_frame(self, timeout=2.0):
        with self.frame_lock:
            cached = self.frame
        if cached is not None:
            return cached
        client_side, server_side = socket.socketpair()
        client_side.setblocking(False)
        with self.subs_lock:
            self.subscribers.add(server_side)
        try:
            deadline = time.time() + timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                client_side.settimeout(min(0.2, remaining))
                try:
                    client_side.recv(4096)
                    break
                except (socket.timeout, Exception):
                    pass
                with self.frame_lock:
                    if self.frame is not None:
                        break
            with self.frame_lock:
                return self.frame
        finally:
            with self.subs_lock:
                self.subscribers.discard(server_side)
            server_side.close()
            client_side.close()


def make_handler(bridge: MjpegBridge):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/health':
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(f'OK fps={bridge.fps:.1f} region={bridge.region}\n'.encode())
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()
            try:
                while True:
                    frame = bridge.wait_frame(timeout=3.0)
                    if not frame:
                        continue
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return
            except Exception:
                return
        def log_message(self, format, *args):
            pass
    return Handler


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, required=True)
    p.add_argument('--topic', required=True)
    p.add_argument('--region', choices=['top', 'bottom', 'full'], default='full',
                   help='top=右眼上半 / bottom=左眼下半 / full=整张')
    p.add_argument('--vflip', action='store_true', help='垂直翻转（上下颠倒修复）')
    p.add_argument('--hflip', action='store_true', help='水平翻转（左右镜像修复）')
    args = p.parse_args()

    rclpy.init()
    bridge = MjpegBridge(args.topic, region=args.region, vflip=args.vflip, hflip=args.hflip)
    spin_t = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_t.start()

    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(bridge))
    print(f'MJPEG bridge ready: port={args.port} <- topic={args.topic} region={args.region}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()


if __name__ == '__main__':
    main()
