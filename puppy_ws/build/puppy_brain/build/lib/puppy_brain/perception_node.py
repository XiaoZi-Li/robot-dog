#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import queue
import threading
import ctypes

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

# hobot_codec / mipi_cam 输出的 jpeg topic 默认是 sensor_data QoS (BEST_EFFORT)
# 用默认 RELIABLE 订阅会因 DDS QoS 不兼容而收不到任何数据
# BEST_EFFORT 订阅者能同时兼容 RELIABLE 和 BEST_EFFORT 发布者
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn


# =========================================================
# 和你现有工程兼容的结构体定义
# =========================================================
class hbSysMem_t(ctypes.Structure):
    _fields_ = [
        ("phyAddr", ctypes.c_double),
        ("virAddr", ctypes.c_void_p),
        ("memSize", ctypes.c_int)
    ]


class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [
        ("shiftLen", ctypes.c_int),
        ("shiftData", ctypes.c_char_p)
    ]


class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [
        ("scaleLen", ctypes.c_int),
        ("scaleData", ctypes.POINTER(ctypes.c_float)),
        ("zeroPointLen", ctypes.c_int),
        ("zeroPointData", ctypes.c_char_p)
    ]


class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [
        ("dimensionSize", ctypes.c_int * 8),
        ("numDimensions", ctypes.c_int)
    ]


class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ("validShape", hbDNNTensorShape_t),
        ("alignedShape", hbDNNTensorShape_t),
        ("tensorLayout", ctypes.c_int),
        ("tensorType", ctypes.c_int),
        ("shift", hbDNNQuantiShift_yt),
        ("scale", hbDNNQuantiScale_t),
        ("quantiType", ctypes.c_int),
        ("quantizeAxis", ctypes.c_int),
        ("alignedByteSize", ctypes.c_int),
        ("stride", ctypes.c_int * 8)
    ]


class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [
        ("sysMem", hbSysMem_t * 4),
        ("properties", hbDNNTensorProperties_t)
    ]


class Yolov5PostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ("height", ctypes.c_int),
        ("width", ctypes.c_int),
        ("ori_height", ctypes.c_int),
        ("ori_width", ctypes.c_int),
        ("score_threshold", ctypes.c_float),
        ("nms_threshold", ctypes.c_float),
        ("nms_top_k", ctypes.c_int),
        ("is_pad_resize", ctypes.c_int)
    ]


libpostprocess = ctypes.CDLL('/usr/lib/libpostprocess.so')
get_postprocess_result = libpostprocess.Yolov5PostProcess
get_postprocess_result.argtypes = [ctypes.POINTER(Yolov5PostProcessInfo_t)]
get_postprocess_result.restype = ctypes.c_char_p


def get_tensor_layout(layout_name: str) -> int:
    return 2 if layout_name == "NCHW" else 0


# =========================================================
# 感知节点：改为订阅统一图像输入，不再自己开摄像头
# =========================================================
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ========= 参数 =========
        self.declare_parameter('model_path', '/app/model/basic/yolov5s_672x672_nv12.bin')
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('nms_top_k', 20)

        self.declare_parameter('input_width', 672)
        self.declare_parameter('input_height', 672)

        # 这里的 orig_width / orig_height 指“统一输入图像”的尺寸
        # 你当前 gesture 链路用 mipi_cam + codec，实际是 960x544
        self.declare_parameter('orig_width', 960)
        self.declare_parameter('orig_height', 544)

        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('log_interval_sec', 1.0)

        self.model_path = self.get_parameter('model_path').value
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.nms_top_k = int(self.get_parameter('nms_top_k').value)

        self.input_width = int(self.get_parameter('input_width').value)
        self.input_height = int(self.get_parameter('input_height').value)

        self.orig_width = int(self.get_parameter('orig_width').value)
        self.orig_height = int(self.get_parameter('orig_height').value)

        self.image_topic = self.get_parameter('image_topic').value
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # ========= 发布 =========
        self.result_pub = self.create_publisher(String, '/perception/result_json', 10)

        # ========= 队列 =========
        self.frame_queue = queue.Queue(maxsize=3)
        self.result_queue = queue.Queue(maxsize=3)

        # ========= 状态 =========
        self.frame_id = 0
        self.last_log_time = 0.0
        self.last_sub_log_time = 0.0
        self.subscription_created = False
        self.image_sub = None

        # ========= 模型 =========
        self.get_logger().info(f'Loading model: {self.model_path}')
        self.models = dnn.load(self.model_path)

        # ========= 后处理配置 =========
        self.post_info = Yolov5PostProcessInfo_t()
        self.post_info.height = self.input_height
        self.post_info.width = self.input_width
        self.post_info.ori_height = self.orig_height
        self.post_info.ori_width = self.orig_width
        self.post_info.score_threshold = float(self.score_threshold)
        self.post_info.nms_threshold = float(self.nms_threshold)
        self.post_info.nms_top_k = int(self.nms_top_k)
        self.post_info.is_pad_resize = 1

        # ========= 动态检测 /image 类型 =========
        self.detect_timer = self.create_timer(1.0, self.try_create_image_subscription)

        # ========= 线程 =========
        self._running = True
        threading.Thread(target=self.ai_thread, daemon=True).start()
        threading.Thread(target=self.publisher_thread, daemon=True).start()

        self.get_logger().info(
            f'perception_node started. waiting image topic: {self.image_topic}'
        )

    # -----------------------------------------------------
    # 自动识别 /image 的 ROS2 消息类型
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
                SENSOR_QOS
            )
            self.subscription_created = True
            self.get_logger().info(
                f'subscribed {self.image_topic} as sensor_msgs/msg/CompressedImage (BEST_EFFORT)'
            )
            return

        if 'sensor_msgs/msg/Image' in topic_types:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                SENSOR_QOS
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
    # 图像回调
    # -----------------------------------------------------
    def compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr is None:
                self.get_logger().warn('compressed image decode failed')
                return
            self.push_bgr_frame(bgr)
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

            self.push_bgr_frame(bgr)
        except Exception as e:
            self.get_logger().error(f'raw_image_callback error: {e}')

    # -----------------------------------------------------
    # 统一预处理：BGR -> resize -> NV12
    # -----------------------------------------------------
    def push_bgr_frame(self, bgr: np.ndarray):
        try:
            self.orig_height, self.orig_width = bgr.shape[:2]
            self.post_info.ori_height = self.orig_height
            self.post_info.ori_width = self.orig_width

            resized = cv2.resize(
                bgr,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_LINEAR
            )

            nv12 = self.bgr_to_nv12(resized)

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

            self.frame_queue.put({
                'nv12': nv12,
                'orig_width': self.orig_width,
                'orig_height': self.orig_height,
            })
        except Exception as e:
            self.get_logger().error(f'push_bgr_frame error: {e}')

    def bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        yuv_i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)

        y_size = h * w
        uv_size = y_size // 4

        y = yuv_i420[:y_size]
        u = yuv_i420[y_size:y_size + uv_size].reshape((h // 2, w // 2))
        v = yuv_i420[y_size + uv_size:].reshape((h // 2, w // 2))

        uv = np.empty((h // 2, w), dtype=np.uint8)
        uv[:, 0::2] = u
        uv[:, 1::2] = v

        nv12 = np.concatenate([y, uv.reshape(-1)])
        return nv12

    # -----------------------------------------------------
    # AI 推理线程
    # -----------------------------------------------------
    def ai_thread(self):
        while self._running:
            try:
                item = self.frame_queue.get()
                nv12 = item['nv12']

                self.post_info.ori_width = int(item['orig_width'])
                self.post_info.ori_height = int(item['orig_height'])

                outputs = self.models[0].forward(nv12)

                output_tensors = (hbDNNTensor_t * len(outputs))()

                for i in range(len(outputs)):
                    output_tensors[i].properties.tensorLayout = get_tensor_layout(
                        outputs[i].properties.layout
                    )

                    if len(outputs[i].properties.scale_data) == 0:
                        output_tensors[i].properties.quantiType = 0
                        output_tensors[i].sysMem[0].virAddr = ctypes.cast(
                            outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                            ctypes.c_void_p
                        )
                    else:
                        output_tensors[i].properties.quantiType = 2
                        output_tensors[i].properties.scale.scaleData = \
                            outputs[i].properties.scale_data.ctypes.data_as(
                                ctypes.POINTER(ctypes.c_float)
                            )
                        output_tensors[i].sysMem[0].virAddr = ctypes.cast(
                            outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                            ctypes.c_void_p
                        )

                    for j in range(len(outputs[i].properties.shape)):
                        output_tensors[i].properties.validShape.dimensionSize[j] = \
                            outputs[i].properties.shape[j]

                    libpostprocess.Yolov5doProcess(
                        output_tensors[i],
                        ctypes.pointer(self.post_info),
                        i
                    )

                result_str = get_postprocess_result(
                    ctypes.pointer(self.post_info)
                ).decode('utf-8')

                detections = self.safe_parse_postprocess_result(result_str)

                payload = {
                    "timestamp": time.time(),
                    "frame_id": self.frame_id,
                    "image_width": int(item['orig_width']),
                    "image_height": int(item['orig_height']),
                    "detections": detections
                }
                self.frame_id += 1

                if self.result_queue.full():
                    try:
                        self.result_queue.get_nowait()
                    except queue.Empty:
                        pass

                self.result_queue.put(payload)

            except Exception as e:
                self.get_logger().error(f'ai_thread error: {e}')
                time.sleep(0.05)

    # -----------------------------------------------------
    # 发布线程
    # -----------------------------------------------------
    def publisher_thread(self):
        while self._running:
            try:
                payload = self.result_queue.get()

                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                self.result_pub.publish(msg)

                now = time.time()
                if now - self.last_log_time > self.log_interval_sec:
                    self.get_logger().info(
                        f'perception publish | frame_id={payload["frame_id"]} | '
                        f'image={payload["image_width"]}x{payload["image_height"]} | '
                        f'detections={len(payload["detections"])}'
                    )
                    self.last_log_time = now

            except Exception as e:
                self.get_logger().error(f'publisher_thread error: {e}')
                time.sleep(0.05)

    # -----------------------------------------------------
    # 后处理结果解析
    # -----------------------------------------------------
    def safe_parse_postprocess_result(self, result_str: str):
        try:
            idx = result_str.find('[')
            if idx != -1:
                return json.loads(result_str[idx:])
            return []
        except Exception:
            try:
                return json.loads(result_str[16:])
            except Exception:
                self.get_logger().warn(
                    f'Failed to parse postprocess result: {result_str[:120]}'
                )
                return []

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
