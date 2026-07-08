import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import queue
import time
import numpy as np
import json
import ctypes

# RDK库
try:
    from hobot_vio import libsrcampy as srcampy
except:
    from hobot_vio_rdkx5 import libsrcampy as srcampy

try:
    from hobot_dnn import pyeasy_dnn as dnn
except:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn


# ==========================================
# 🌟 100% 还原官方原版 C++ 结构体映射
# ==========================================
class hbSysMem_t(ctypes.Structure):
    _fields_ = [("phyAddr",ctypes.c_double), ("virAddr",ctypes.c_void_p), ("memSize",ctypes.c_int)]

class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [("shiftLen",ctypes.c_int), ("shiftData",ctypes.c_char_p)]

class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [("scaleLen",ctypes.c_int), ("scaleData",ctypes.POINTER(ctypes.c_float)), ("zeroPointLen",ctypes.c_int), ("zeroPointData",ctypes.c_char_p)]    

class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [("dimensionSize",ctypes.c_int * 8), ("numDimensions",ctypes.c_int)]

class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ("validShape",hbDNNTensorShape_t), ("alignedShape",hbDNNTensorShape_t),
        ("tensorLayout",ctypes.c_int), ("tensorType",ctypes.c_int),
        ("shift",hbDNNQuantiShift_yt), ("scale",hbDNNQuantiScale_t),
        ("quantiType",ctypes.c_int), ("quantizeAxis", ctypes.c_int),
        ("alignedByteSize",ctypes.c_int), ("stride",ctypes.c_int * 8)
    ]

class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [("sysMem",hbSysMem_t * 4), ("properties",hbDNNTensorProperties_t)]

class Yolov5PostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ("height",ctypes.c_int), ("width",ctypes.c_int),
        ("ori_height",ctypes.c_int), ("ori_width",ctypes.c_int),
        ("score_threshold",ctypes.c_float), ("nms_threshold",ctypes.c_float),
        ("nms_top_k",ctypes.c_int), ("is_pad_resize",ctypes.c_int)
    ]

libpostprocess = ctypes.CDLL('/usr/lib/libpostprocess.so') 
get_Postprocess_result = libpostprocess.Yolov5PostProcess
get_Postprocess_result.argtypes = [ctypes.POINTER(Yolov5PostProcessInfo_t)]  
get_Postprocess_result.restype = ctypes.c_char_p  

def get_TensorLayout(Layout):
    return int(2) if Layout == "NCHW" else int(0)


# ==========================================
# 视觉节点
# ==========================================

class VisionNode(Node):

    def __init__(self):
        super().__init__('ai_vision_node')
        self.publisher = self.create_publisher(String, '/puppy_action', 10)
        self.get_logger().info("AI Vision Node Started")

        self.frame_queue = queue.Queue(maxsize=3)
        self.result_queue = queue.Queue(maxsize=3)

        self.last_action = "none"
        self.last_send_time = 0
        self.last_log_time = 0
        
        # 幽灵记忆机制状态变量
        self.last_person_time = 0.0
        self.last_person_area = 0.0
        self.ghost_memory_time = 3.0

        self.get_logger().info("Loading YOLO model")
        self.models = dnn.load('/app/model/basic/yolov5s_672x672_nv12.bin')

        self.cam = srcampy.Camera()
        self.cam.open_cam(0, -1, -1, [672,1920], [672,1080],1080,1920)

        # 初始化解析配置
        self.post_info = Yolov5PostProcessInfo_t()
        self.post_info.height = 672
        self.post_info.width = 672
        self.post_info.ori_height = 1080
        self.post_info.ori_width = 1920
        self.post_info.score_threshold = 0.25 # 同步了你原版的0.25
        self.post_info.nms_threshold = 0.45
        self.post_info.nms_top_k = 20
        self.post_info.is_pad_resize = 1

        threading.Thread(target=self.camera_thread,daemon=True).start()
        threading.Thread(target=self.ai_thread,daemon=True).start()
        threading.Thread(target=self.decision_thread,daemon=True).start()


    def camera_thread(self):
        while True:
            img = self.cam.get_img(2,672,672)
            if img is None:
                continue
            
            # 🌟 防止底层内存撕裂，保留 .copy()
            frame = np.frombuffer(img, dtype=np.uint8).copy()
            if not self.frame_queue.full():
                self.frame_queue.put(frame)


    def ai_thread(self):
        while True:
            frame = self.frame_queue.get()
            outputs = self.models[0].forward(frame)
            
            # 🌟 100% 原汁原味的 C++ 加速对接逻辑
            output_tensors = (hbDNNTensor_t * len(outputs))()
            for i in range(len(outputs)):
                output_tensors[i].properties.tensorLayout = get_TensorLayout(outputs[i].properties.layout)
                
                if (len(outputs[i].properties.scale_data) == 0):
                    output_tensors[i].properties.quantiType = 0
                    output_tensors[i].sysMem[0].virAddr = ctypes.cast(outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ctypes.c_void_p)
                else:
                    output_tensors[i].properties.quantiType = 2       
                    output_tensors[i].properties.scale.scaleData = outputs[i].properties.scale_data.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
                    output_tensors[i].sysMem[0].virAddr = ctypes.cast(outputs[i].buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), ctypes.c_void_p)
                
                for j in range(len(outputs[i].properties.shape)):
                    output_tensors[i].properties.validShape.dimensionSize[j] = outputs[i].properties.shape[j]
                
                libpostprocess.Yolov5doProcess(output_tensors[i], ctypes.pointer(self.post_info), i)

            result_str = get_Postprocess_result(ctypes.pointer(self.post_info)).decode('utf-8')  
            data = json.loads(result_str[16:])  

            if not self.result_queue.full():
                self.result_queue.put(data)


    def decision_thread(self):
        while True:
            data = self.result_queue.get()
            
            action = "stop"
            person_detected_this_frame = False

            for result in data:
                name = result['name']
                if name != "person":
                    continue
                
                person_detected_this_frame = True
                bbox = result['bbox']    
                
                x1 = max(0, int(bbox[0]))
                y1 = max(2, int(bbox[1]))
                x2 = min(1920, int(bbox[2]))
                y2 = min(1080, int(bbox[3]))
                
                x_center = (x1 + x2) / 2
                box_area = (x2 - x1) * (y2 - y1)
                area_ratio = box_area / (1920 * 1080)
                
                # 更新幽灵记忆状态机
                self.last_person_time = time.time()
                self.last_person_area = area_ratio
                
                # 距离与追踪逻辑
                if area_ratio > 0.35: 
                    action = "stop" 
                elif area_ratio < 0.15: 
                    if x_center < 700:
                        action = "turn_left"
                    elif x_center > 1220:
                        action = "turn_right"
                    else:
                        action = "walk"
                else:
                    if x_center < 700:
                        action = "turn_left"
                    elif x_center > 1220:
                        action = "turn_right"
                    else:
                        action = "stop"
                
                # 控制打印频率
                current_time = time.time()
                if current_time - self.last_log_time > 0.2:
                    self.get_logger().info(f"🎯 [原装驱动锁定] X={x_center:.0f} | 面积比={area_ratio:.2f} | 动作: {action}")
                    self.last_log_time = current_time
                    
                break # 抓到一个主要的人就退出遍历

            # ==========================================
            # 👻 幽灵记忆防撞逻辑
            # ==========================================
            if not person_detected_this_frame:
                time_since_last_seen = time.time() - self.last_person_time
                if time_since_last_seen < self.ghost_memory_time and self.last_person_area > 0.35:
                    current_time = time.time()
                    if current_time - self.last_log_time > 0.3:
                        self.get_logger().info(f"👻 [幽灵触发] 目标贴脸消失！强制刹车保命！(剩余记忆 {self.ghost_memory_time - time_since_last_seen:.1f}s)")
                        self.last_log_time = current_time
                    action = "stop"
                else:
                    action = "stop"

            # ==========================================
            # 防抖发布
            # ==========================================
            if action != self.last_action and time.time()-self.last_send_time > 0.3:
                msg = String()
                msg.data = action
                self.publisher.publish(msg)
                self.get_logger().info(f"🚀 >> UDP 网桥动作下发: 【{action}】")
                self.last_action = action
                self.last_send_time = time.time()

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.cam.close_cam()
    rclpy.shutdown()

if __name__=="__main__":
    main()