#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_control.py - USB 摄像头 + MediaPipe 手势识别 + UDP 直发 sit.py (5005)

完全脱离 ROS, 用 Google MediaPipe Hands 做手部 21 关键点检测 + 自定义手势分类,
识别结果直接通过 UDP JSON 发给 sit.py 控制机器狗。

依赖:
  pip install mediapipe opencv-python numpy

手势 → 动作映射 (满足用户需求):
  手掌张开 (5指伸直)        → walk    (前进)
  握拳 (5指弯曲)            → crouch  (趴下)
  点赞 (只竖拇指)           → sit     (坐下)
  OK (拇指+食指捏圈)        → backward(后退)
  V/食指中指 (2指伸直)      → turn_left  (左转)
  双手掌 (双手都是张开)     → turn_right (右转)
  无手势 / 其他             → stop    (停止)

  注: 停止会在手势消失 0.5s 后自动发送, 避免机器狗失控。

sit.py UDP 协议 (端口 5005):
  {"action": "walk", "source": "gesture"}
  支持的 action: walk/forward, backward, turn_left, turn_right, stop, sit, stand, crouch

运行:
  # 先启动 sit.py (终端1)
  python gesture_control.py
  python gesture_control.py --device /dev/video0 --show

  --show 会开本地窗口显示 (SSH 需要 X11 转发, 否则不要加)
"""
import os
import sys
import time
import json
import socket
import argparse
import threading
import http.server
import socketserver
from typing import Optional, List, Tuple

import cv2
import numpy as np

try:
    import mediapipe as mp
except ImportError:
    print("[ERROR] 未安装 mediapipe, 请执行: pip install mediapipe")
    sys.exit(1)


# ============================================================
# sit.py UDP 通信
# ============================================================
class SitUdpClient:
    """向 sit.py 发送 UDP JSON 指令"""

    def __init__(self, ip: str = '127.0.0.1', port: int = 5005):
        self.server = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_action(self, action: str, source: str = 'gesture', **extra):
        """发送离散动作: walk/backward/turn_left/turn_right/stop/sit/stand/crouch"""
        payload = {'action': action, 'source': source}
        payload.update(extra)
        self.sock.sendto(json.dumps(payload, ensure_ascii=False).encode('utf-8'),
                         self.server)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


# ============================================================
# 手势分类器 (基于 MediaPipe 21 关键点)
# ============================================================
# MediaPipe Hand 21 关键点索引:
#  0:手腕  1-4:拇指  5-8:食指  9-12:中指  13-16:无名指  17-20:小指
#  指尖: 4(拇指),8(食指),12(中指),16(无名指),20(小指)
FINGER_TIPS = [4, 8, 12, 16, 20]
FINGER_PIPS = [3, 6, 10, 14, 18]   # 第二关节 (用于判断伸直/弯曲)


class GestureDetector:
    """基于关键点几何关系分类手势"""

    @staticmethod
    def _dist(a, b) -> float:
        """a, b 是 NormalizedLandmark (有 .x .y .z 属性)"""
        return float(np.hypot(a.x - b.x, a.y - b.y))

    @staticmethod
    def _is_finger_extended(landmarks, tip_idx: int, pip_idx: int,
                            handedness_is_right: bool) -> bool:
        """判断手指是否伸直 (指尖比PIP关节更远离手腕)。
        拇指方向特殊, 用横向距离判断。"""
        wrist = landmarks[0]
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]

        if tip_idx == 4:  # 拇指: 用 x 方向 (镜像: 右手在画面左侧)
            # MediaPipe 已根据 handedness 调整, 右手拇指伸直时 tip.x < pip.x
            if handedness_is_right:
                return tip.x < pip.x - 0.02
            else:
                return tip.x > pip.x + 0.02
        else:
            # 其他四指: tip 比 pip 更远离手腕
            tip_dist = GestureDetector._dist(tip, wrist)
            pip_dist = GestureDetector._dist(pip, wrist)
            return tip_dist > pip_dist * 1.1

    @staticmethod
    def classify(landmarks, handedness_label: str) -> str:
        """返回手势名称: palm/fist/thumb_up/okay/victory/unknown
        landmarks: 21 个 (x,y,z) 归一化坐标
        handedness_label: 'Left' 或 'Right'
        """
        is_right = (handedness_label == 'Right')

        # 5 指伸直状态
        extended = [
            GestureDetector._is_finger_extended(landmarks, tip, pip, is_right)
            for tip, pip in zip(FINGER_TIPS, FINGER_PIPS)
        ]
        # extended = [拇指, 食指, 中指, 无名指, 小指]
        thumb, index, middle, ring, pinky = extended

        # ---- OK 手势: 拇指尖与食指尖距离很近, 中指/无名指/小指伸直 ----
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        ok_dist = GestureDetector._dist(thumb_tip, index_tip)
        # 归一化: 用手掌大小 (手腕到中指根部) 作为尺度
        palm_size = GestureDetector._dist(landmarks[0], landmarks[9])
        if palm_size > 1e-3 and ok_dist / palm_size < 0.5 and middle and ring and pinky:
            return 'okay'

        # ---- 手掌张开: 5 指都伸直 ----
        if thumb and index and middle and ring and pinky:
            return 'palm'

        # ---- 握拳: 5 指都弯曲 ----
        if not thumb and not index and not middle and not ring and not pinky:
            return 'fist'

        # ---- 点赞: 只拇指伸直 ----
        if thumb and not index and not middle and not ring and not pinky:
            return 'thumb_up'

        # ---- V 手势: 食指+中指伸直, 其他弯曲 ----
        if index and middle and not thumb and not ring and not pinky:
            return 'victory'

        return 'unknown'


# ============================================================
# 手势 → 动作决策 (含双手识别)
# ============================================================
# 用户需求映射:
#   palm(手掌)   → walk (前进)
#   fist(握拳)   → crouch (趴下)
#   thumb_up(点赞) → sit (坐下)
#   okay        → backward (后退)
#   victory(V)  → turn_left (左转)
#   双 palm     → turn_right (右转)
#   无/unknown  → stop
GESTURE_TO_ACTION = {
    'palm':      'walk',
    'fist':      'crouch',
    'thumb_up':  'sit',
    'okay':      'backward',
    'victory':   'turn_left',
}

# 离散动作: 单次发送, 加锁防重复
DISCRETE_ACTIONS = {'sit', 'crouch', 'stand'}
# 移动动作: 持续发送, 手势消失自动 stop
MOVE_ACTIONS = {'walk', 'backward', 'turn_left', 'turn_right'}


class ActionScheduler:
    """管理动作发送节奏, 避免重复触发和失控"""

    def __init__(self, udp: SitUdpClient,
                 gesture_hold_sec: float = 0.5,
                 action_lock_sec: float = 2.5,
                 log_interval: float = 0.5):
        self.udp = udp
        self.gesture_hold_sec = gesture_hold_sec
        self.action_lock_sec = action_lock_sec
        self.log_interval = log_interval

        self._lock = threading.Lock()
        self.last_gesture_time = 0.0
        self.current_move_action: Optional[str] = None  # 当前正在执行的移动动作
        self.last_discrete_action: Optional[str] = None
        self.action_lock_until = 0.0
        self.last_log_time = 0.0
        self.last_send_time = 0.0

    def update(self, gestures: List[str], now: float):
        """gestures: 当前帧检测到的所有手势名称列表"""
        with self._lock:
            self.last_gesture_time = now

        # 统计双手
        palm_count = sum(1 for g in gestures if g == 'palm')

        # 决策优先级: 离散动作 > 双手 > 单手移动
        action = None
        gesture_name = 'none'

        # 1. 离散动作 (fist/thumb_up/okay)
        if now >= self.action_lock_until:
            for g in gestures:
                if g in ('fist', 'thumb_up'):
                    action = GESTURE_TO_ACTION[g]
                    gesture_name = g
                    break

        # 2. 双手 palm → turn_right
        if action is None and palm_count >= 2:
            action = 'turn_right'
            gesture_name = 'double_palm'

        # 3. 单手移动类
        if action is None:
            for g in gestures:
                if g in GESTURE_TO_ACTION and GESTURE_TO_ACTION[g] in MOVE_ACTIONS:
                    # okay 单独处理 (移动类)
                    if g == 'okay':
                        action = 'backward'
                        gesture_name = 'okay'
                        break
                    elif g == 'victory':
                        action = 'turn_left'
                        gesture_name = 'victory'
                        break
                    elif g == 'palm':
                        action = 'walk'
                        gesture_name = 'palm'
                        break

        # 4. 执行
        if action is None:
            # 无有效手势, 不立即停, 由 _check_timeout 处理
            return

        # 离散动作: 触发一次 + 加锁
        if action in DISCRETE_ACTIONS:
            self._send_discrete(action, gesture_name, now)
        # 移动动作: 持续发送 (切换时先 stop)
        elif action in MOVE_ACTIONS:
            self._send_move(action, gesture_name, now)

    def _send_discrete(self, action: str, gesture: str, now: float):
        with self._lock:
            if action == self.last_discrete_action and now < self.action_lock_until:
                return  # 锁定期内同动作, 跳过
            self.last_discrete_action = action
            self.action_lock_until = now + self.action_lock_sec
            self.current_move_action = None

        self.udp.send_action(action)
        if now - self.last_log_time > self.log_interval:
            print(f"[gesture] {gesture} → {action}")
            self.last_log_time = now

    def _send_move(self, action: str, gesture: str, now: float):
        with self._lock:
            # 切换动作时先停车
            if (self.current_move_action is not None
                    and self.current_move_action != action):
                self.udp.send_action('stop')
                print(f"[gesture] 切换 {self.current_move_action}→{action}, 先停车")
            self.current_move_action = action

        # 限频发送 (sit.py 收到 walk 会持续走, 不需要太高频率)
        if now - self.last_send_time < 0.2:
            return
        self.last_send_time = now
        self.udp.send_action(action)
        if now - self.last_log_time > self.log_interval:
            print(f"[gesture] {gesture} → {action} (持续)")
            self.last_log_time = now

    def check_timeout(self, now: float):
        """手势消失超时自动停车 (必须周期调用)"""
        with self._lock:
            if self.current_move_action is None:
                return
            if now - self.last_gesture_time <= self.gesture_hold_sec:
                return
            action = self.current_move_action
            self.current_move_action = None
        # 锁外发 stop (避免死锁)
        self.udp.send_action('stop')
        print(f"[gesture] 手势消失, 停车 (was {action})")

    def brake(self):
        """退出前停车"""
        self.udp.send_action('stop')
        print("[gesture] 退出, 发送 stop")


# ============================================================
# 主程序: USB 摄像头 + MediaPipe Hands + 动作调度
# ============================================================
# ============================================================
# HTTP MJPEG 推流 (8093端口) - 供浏览器实时查看手势画面
# ============================================================
class FrameBuffer:
    """线程安全的最新帧缓冲, 供推流线程读取"""
    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._jpeg: Optional[bytes] = None
        self._frame_id = 0

    def update(self, jpeg: bytes):
        with self._cond:
            self._jpeg = jpeg
            self._frame_id += 1
            self._cond.notify_all()

    def get_frame(self, timeout: float = 1.0) -> Optional[bytes]:
        with self._cond:
            cur = self._frame_id
            if self._jpeg is None or self._frame_id == cur:
                self._cond.wait(timeout)
            return self._jpeg

    def get_snapshot(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg


class MjpegHandler(http.server.BaseHTTPRequestHandler):
    buf: FrameBuffer = None  # 由 main 注入

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == '/' or self.path.startswith('/index'):
            self._send_index()
        elif self.path.startswith('/stream'):
            self._send_stream()
        elif self.path.startswith('/snapshot'):
            self._send_snapshot()
        elif self.path == '/health':
            self._send_health()
        else:
            self.send_error(404)

    def _send_index(self):
        html = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Gesture Control</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0}
img{max-width:100%;border:2px solid #444}
a{color:#4af}</style></head>
<body>
<h2>Gesture Control (MediaPipe Hands)</h2>
<img src="/stream" alt="stream">
<p><a href="/snapshot" target="_blank">snapshot</a> | <a href="/health">health</a></p>
</body></html>""".encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _send_stream(self):
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                jpeg = self.buf.get_frame(timeout=2.0)
                if jpeg is None:
                    continue
                self.wfile.write(b'--FRAME\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_snapshot(self):
        jpeg = self.buf.get_snapshot()
        if jpeg is None:
            self.send_error(503, 'No frame')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_health(self):
        txt = "gesture_control running".encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(txt)))
        self.end_headers()
        self.wfile.write(txt)


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def find_first_usb_camera() -> Optional[str]:
    if not os.path.isdir('/dev'):
        return None
    for dev in sorted(os.listdir('/dev')):
        if not dev.startswith('video'):
            continue
        path = os.path.join('/dev', dev)
        cap = cv2.VideoCapture(path)
        if cap.isOpened():
            cap.release()
            return path
    return None


def main():
    parser = argparse.ArgumentParser(description='USB 摄像头手势识别 → sit.py 机器狗控制')
    parser.add_argument('--device', type=str, default='', help='USB 摄像头设备, 留空自动检测')
    parser.add_argument('--udp-ip', type=str, default='127.0.0.1', help='sit.py 的 IP')
    parser.add_argument('--udp-port', type=int, default=5005, help='sit.py 的 UDP 端口')
    parser.add_argument('--max-hands', type=int, default=2, help='最大检测手数 (1或2)')
    parser.add_argument('--show', action='store_true', help='显示本地窗口 (需X11)')
    parser.add_argument('--port', type=int, default=8093, help='HTTP MJPEG 推流端口')
    parser.add_argument('--hold-sec', type=float, default=0.5,
                        help='手势消失多少秒后停车')
    parser.add_argument('--lock-sec', type=float, default=2.5,
                        help='离散动作(sit/crouch)防重复锁时长')
    args = parser.parse_args()

    # 1. 打开摄像头
    device = args.device or find_first_usb_camera()
    if not device:
        print("[ERROR] 未找到 USB 摄像头")
        sys.exit(1)
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"[ERROR] 无法打开 {device}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    print(f"[camera] 已打开 {device}")

    # 2. 初始化 MediaPipe Hands
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=args.max_hands,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5,
    )
    print("[mediapipe] Hands 初始化完成")

    # 3. 初始化 UDP + 动作调度
    udp = SitUdpClient(args.udp_ip, args.udp_port)
    scheduler = ActionScheduler(udp,
                                gesture_hold_sec=args.hold_sec,
                                action_lock_sec=args.lock_sec)
    print(f"[udp] 目标 {args.udp_ip}:{args.udp_port}")
    print("[main] 手势映射:")
    print("  手掌张开 → walk(前进)")
    print("  握拳    → crouch(趴下)")
    print("  点赞    → sit(坐下)")
    print("  OK     → backward(后退)")
    print("  V/剪刀 → turn_left(左转)")
    print("  双手掌  → turn_right(右转)")
    print("  无手势  → stop(停止)")
    print("[main] Ctrl+C 退出")

    # 4. 启动 HTTP MJPEG 推流 (后台线程)
    frame_buf = FrameBuffer()
    MjpegHandler.buf = frame_buf
    http_server = ThreadingHTTPServer(('0.0.0.0', args.port), MjpegHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()
    print(f"[server] MJPEG 推流已就绪: http://0.0.0.0:{args.port}/")

    # 5. 主循环
    fps_t0 = time.time()
    fps_cnt = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # MediaPipe 需要 RGB
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            # 解析手势
            gestures_this_frame: List[str] = []
            if results.multi_hand_landmarks and results.multi_handedness:
                for idx, (lms, hd) in enumerate(
                        zip(results.multi_hand_landmarks, results.multi_handedness)):
                    label = hd.classification[0].label  # 'Left'/'Right'
                    gesture = GestureDetector.classify(lms.landmark, label)
                    gestures_this_frame.append(gesture)

                    # 画关键点 + 标签 (始终渲染, 用于推流)
                    mp_drawing.draw_landmarks(frame, lms, mp_hands.HAND_CONNECTIONS)
                    h, w = frame.shape[:2]
                    cx = int(lms.landmark[0].x * w)
                    cy = int(lms.landmark[0].y * h)
                    cv2.putText(frame, f"{label}:{gesture}",
                                (cx - 40, cy + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 动作决策
            now = time.time()
            scheduler.update(gestures_this_frame, now)
            scheduler.check_timeout(now)

            # 状态栏 (始终画, 用于推流)
            cv2.putText(frame, f"gestures: {gestures_this_frame}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 推流: 编码 JPEG → FrameBuffer
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                frame_buf.update(buf.tobytes())

            # 本地显示 (可选)
            if args.show:
                cv2.imshow("Gesture Control", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # FPS
            fps_cnt += 1
            if now - fps_t0 >= 5.0:
                fps = fps_cnt / (now - fps_t0)
                print(f"[main] fps={fps:.1f} gestures={gestures_this_frame}")
                fps_cnt = 0
                fps_t0 = now

    except KeyboardInterrupt:
        print("\n[main] 退出中...")
    finally:
        scheduler.brake()
        cap.release()
        hands.close()
        udp.close()
        http_server.shutdown()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
