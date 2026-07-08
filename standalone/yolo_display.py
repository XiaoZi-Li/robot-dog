#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yolo_display.py - USB 摄像头 + YOLOv5 BPU 推理 + HTTP MJPEG 推流 (8093端口)

完全脱离 ROS, 直接用 hbm_runtime 调 BPU 推理, OpenCV 采集, HTTP 推流到浏览器。
复用项目 /app/pydev_demo/07_usb_camera_sample/usb_camera_yolov5x.py 的推理逻辑。

数据流:
  USB 摄像头 (YUYV 640x480)
    → OpenCV 采集 BGR 帧
    → hbm_runtime YOLOv5s 推理 (BPU)
    → 画框 + 标签
    → JPEG 编码
    → HTTP multipart/x-mixed-replace MJPEG 推流 (8093端口)

运行:
  python yolo_display.py
  python yolo_display.py --device /dev/video0 --port 8093 --model /app/model/basic/yolov5s_672x672_nv12.bin

浏览器访问: http://<板端IP>:8093/
"""
import os
import sys
import time
import threading
import argparse
import http.server
import socketserver
from typing import Optional, Dict, Tuple

import cv2
import numpy as np

# 复用项目里的 hbm_runtime 推理逻辑 (utils 在 07_usb_camera_sample 的上一级)
sys.path.append('/app/pydev_demo')
import hbm_runtime
import utils.preprocess_utils as pre_utils
import utils.postprocess_utils as post_utils
import utils.common_utils as common
import utils.draw_utils as draw

STRIDES = np.array([8, 16, 32], dtype=np.int32)
ANCHORS = np.array([
    [10, 13], [16, 30], [33, 23],
    [30, 61], [62, 45], [59, 119],
    [116, 90], [156, 198], [373, 326]
], dtype=np.float32).reshape(3, 3, 2)


# ============================================================
# YOLOv5 推理封装
# ============================================================
class YoloV5:
    def __init__(self, model_path: str, score_thres: float = 0.25, nms_thres: float = 0.45):
        self.model = hbm_runtime.HB_HBMRuntime(model_path)
        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]
        self.output_quants = self.model.output_quants[self.model_name]
        self.input_H = self.input_shapes[self.input_names[0]][2]
        self.input_W = self.input_shapes[self.input_names[0]][3]
        self.score_thres = score_thres
        self.nms_thres = nms_thres
        self.resize_type = 1
        self.classes_num = 80

    def infer(self, img: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """输入 BGR 图像, 返回 (boxes, cls_ids, scores)"""
        h, w = img.shape[:2]
        # 预处理: resize + BGR→NV12
        resize_img = pre_utils.resized_image(img, self.input_W, self.input_H, self.resize_type)
        y, uv = pre_utils.bgr_to_nv12_planes(resize_img)
        nv12 = np.concatenate((y.reshape(-1), uv.reshape(-1)), axis=0)
        nv12 = nv12.reshape((1, self.input_H * 3 // 2, self.input_W, 1))
        input_tensor = {self.model_name: {self.input_names[0]: nv12}}

        # 推理
        outputs = self.model.run(input_tensor)[self.model_name]

        # 后处理
        fp32_outputs = post_utils.dequantize_outputs(outputs, self.output_quants)
        pred = post_utils.decode_outputs(self.output_names, fp32_outputs,
                                         STRIDES, ANCHORS, self.classes_num)
        xyxy_boxes, score, cls = post_utils.filter_predictions(pred, self.score_thres)
        keep = post_utils.NMS(xyxy_boxes, score, cls, self.nms_thres)
        xyxy = post_utils.scale_coords_back(xyxy_boxes[keep], w, h,
                                            self.input_W, self.input_H, self.resize_type)
        return xyxy, cls[keep], score[keep]


# ============================================================
# USB 摄像头采集 + 推理线程
# ============================================================
class CameraYoloThread(threading.Thread):
    def __init__(self, device: str, model_path: str, label_file: str,
                 score_thres: float, nms_thres: float):
        super().__init__(daemon=True)
        self.device = device
        self.model_path = model_path
        self.label_file = label_file
        self.score_thres = score_thres
        self.nms_thres = nms_thres

        self.cap = None
        self.yolo = None
        self.coco_names = None

        # 帧缓冲: 用 Condition + 帧序号通知推流线程取新帧
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest_jpeg = None       # 最新的 JPEG 字节流
        self._latest_frame_id = 0
        self._fps = 0.0
        self._running = True

    def run(self):
        # 1. 打开摄像头 (YUYV)
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            print(f"[ERROR] 无法打开摄像头 {self.device}")
            self._running = False
            return
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[camera] 已打开 {self.device} 实际={w}x{h}")

        # 2. 加载 YOLO 模型
        print(f"[yolo] 加载模型 {self.model_path} ...")
        self.yolo = YoloV5(self.model_path, self.score_thres, self.nms_thres)
        self.coco_names = common.load_class_names(self.label_file)
        print(f"[yolo] 模型加载完成, 类别数={len(self.coco_names)}")

        # 3. 循环采集 + 推理
        fps_t0 = time.time()
        fps_cnt = 0
        while self._running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue

            # BPU 推理
            try:
                boxes, cls_ids, scores = self.yolo.infer(frame)
            except Exception as e:
                print(f"[yolo] 推理失败: {e}")
                continue

            # 画框
            vis = draw.draw_boxes(frame.copy(), boxes, cls_ids, scores,
                                  self.coco_names, common.rdk_colors)

            # 状态栏
            cv2.putText(vis, f"YOLOv5 BPU  fps={self._fps:.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 编码 JPEG
            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                continue

            # 更新帧缓冲
            with self._cond:
                self._latest_jpeg = buf.tobytes()
                self._latest_frame_id += 1
                self._cond.notify_all()

            # FPS 统计
            fps_cnt += 1
            now = time.time()
            if now - fps_t0 >= 5.0:
                self._fps = fps_cnt / (now - fps_t0)
                print(f"[yolo] fps={self._fps:.1f} 检测到 {len(boxes)} 个目标")
                fps_cnt = 0
                fps_t0 = now

    def get_frame(self, timeout: float = 1.0) -> Optional[bytes]:
        """阻塞等待下一帧, 返回 JPEG 字节流"""
        with self._cond:
            cur = self._latest_frame_id
            if self._latest_jpeg is None:
                self._cond.wait(timeout)
            else:
                if self._latest_frame_id == cur:
                    self._cond.wait(timeout)
            return self._latest_jpeg

    def get_snapshot(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg

    def stop(self):
        self._running = False
        if self.cap:
            self.cap.release()


# ============================================================
# HTTP MJPEG 推流服务器
# ============================================================
class MjpegHandler(http.server.BaseHTTPRequestHandler):
    cam_thread: CameraYoloThread = None  # 类变量, 由 main 注入

    def log_message(self, fmt, *args):
        pass  # 静默日志

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
<title>YOLOv5 USB Camera</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0}
img{max-width:100%;border:2px solid #444}
a{color:#4af}</style></head>
<body>
<h2>YOLOv5 BPU realtime detection (USB Camera)</h2>
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
        last_id = 0
        try:
            while True:
                jpeg = self.cam_thread.get_frame(timeout=2.0)
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
        jpeg = self.cam_thread.get_snapshot()
        if jpeg is None:
            self.send_error(503, 'No frame')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_health(self):
        txt = (f"device={self.cam_thread.device} fps={self.cam_thread._fps:.1f} "
               f"running={self.cam_thread._running}").encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(txt)))
        self.end_headers()
        self.wfile.write(txt)


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ============================================================
# 主函数
# ============================================================
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
    parser = argparse.ArgumentParser(description='YOLOv5 USB 摄像头实时检测 + HTTP 推流')
    parser.add_argument('--device', type=str, default='',
                        help='USB 摄像头设备路径, 留空自动检测')
    parser.add_argument('--port', type=int, default=8093, help='HTTP 推流端口')
    parser.add_argument('--model', type=str,
                        default='/app/model/basic/yolov5s_672x672_nv12.bin',
                        help='YOLOv5 BPU 模型路径')
    parser.add_argument('--label-file', type=str,
                        default='/app/pydev_demo/07_usb_camera_sample/coco_classes.names',
                        help='COCO 类别文件')
    parser.add_argument('--score-thres', type=float, default=0.25)
    parser.add_argument('--nms-thres', type=float, default=0.45)
    opt = parser.parse_args()

    # 自动检测摄像头
    device = opt.device or find_first_usb_camera()
    if not device:
        print("[ERROR] 未找到 USB 摄像头")
        sys.exit(1)
    print(f"[main] 使用摄像头: {device}")

    # 启动采集+推理线程
    cam_thread = CameraYoloThread(device, opt.model, opt.label_file,
                                  opt.score_thres, opt.nms_thres)
    cam_thread.start()
    time.sleep(2)  # 等模型加载
    if not cam_thread._running:
        print("[ERROR] 采集线程启动失败")
        sys.exit(1)

    # 启动 HTTP 服务器
    MjpegHandler.cam_thread = cam_thread
    server = ThreadingHTTPServer(('0.0.0.0', opt.port), MjpegHandler)
    print(f"[server] YOLOv5 推流已就绪:")
    print(f"[server]   查看页: http://0.0.0.0:{opt.port}/")
    print(f"[server]   直拉流: http://0.0.0.0:{opt.port}/stream")
    print(f"[server]   截图:   http://0.0.0.0:{opt.port}/snapshot")
    print(f"[server] 按 Ctrl+C 退出")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[main] 退出中...")
    finally:
        cam_thread.stop()
        server.shutdown()


if __name__ == '__main__':
    main()
