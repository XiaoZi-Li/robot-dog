# 🤖 PuppyPi 机器人系统 — 从零上手极详细指南

> 本文档基于代码原文逐行解读，目标是让完全没用过 ROS2 的人也能从零上手。
>
> 最后更新：2026-04-18

---

## 目录

- [第一章：系统整体认知](#第一章系统整体认知)
  - [1.1 这套系统是什么？](#11-这套系统是什么)
  - [1.2 系统数据流一图看懂](#12-系统数据流一图看懂)
  - [1.3 话题（Topic）完整清单](#13-话题topic完整清单)
- [第二章：从零部署——第一次跑起来](#第二章从零部署第一次跑起来)
  - [2.1 前置条件检查](#21-前置条件检查)
  - [2.2 编译工程](#22-编译工程)
  - [2.3 启动完整系统](#23-启动完整系统)
  - [2.4 验证系统是否正常运行](#24-验证系统是否正常运行)
- [第三章：核心节点深度剖析](#第三章核心节点深度剖析)
  - [3.1 perception_node.py — 视觉感知节点](#31-perception_nodepy--视觉感知节点)
  - [3.2 decision_node.py — 决策仲裁节点（最核心）](#32-decision_nodepy--决策仲裁节点最核心)
  - [3.3 ros_udp_bridge.py — ROS→UDP转发桥](#33-ros_udp_bridgepy--rosudp转发桥)
  - [3.4 voice_control_node.py — I2C语音控制节点](#34-voice_control_nodepy--i2c语音控制节点)
  - [3.5 usb_asr_text_node.py — USB语音识别节点](#35-usb_asr_text_nodepy--usb语音识别节点)
  - [3.6 intent_router_node.py — 意图路由节点](#36-intent_router_nodepy--意图路由节点)
  - [3.7 chat_llm_bridge_node.py — LLM对话桥](#37-chat_llm_bridge_nodepy--llm对话桥)
  - [3.8 debug_preview_node.py — 调试可视化节点](#38-debug_preview_nodepy--调试可视化节点)
  - [3.9 gesture_adapter_node.py — 手势协议适配器](#39-gesture_adapter_nodepy--手势协议适配器)
  - [3.10 imu_node_ros2.py — IMU发布节点](#310-imu_node_ros2py--imu发布节点)
  - [3.11 ai_vision_node.py — 旧版一体化视觉节点（已弃用）](#311-ai_vision_nodepy--旧版一体化视觉节点已弃用)
- [第四章：三种启动模式与场景选择](#第四章三种启动模式与场景选择)
- [第五章：最常见的调参场景与解决方案](#第五章最常见的调参场景与解决方案)
- [第六章：旧版节点与新版节点的对比](#第六章旧版节点与新版节点的对比)
- [第七章：工具脚本详解](#第七章工具脚本详解)
- [第八章：40pin GPIO 接口示例详解](#第八章40pin-gpio-接口示例详解)
- [第九章：pydev_demo BPU推理示例详解](#第九章pydev_demo-bpu推理示例详解)
- [第十章：常用调试命令速查](#第十章常用调试命令速查)
- [第十一章：扩展开发指南](#第十一章扩展开发指南)

---

## 第一章：系统整体认知

### 1.1 这套系统是什么？

这是一个跑在 **地平线 RDK X5 开发板**上的四足机器人控制系统。整个系统用 **ROS2（Humble）** 框架把各个功能模块串联起来，你可以理解为：

- **ROS2** 是"消息总线"，所有模块（节点）通过它互相通信
- 每个 `.py` 文件就是一个"节点"，节点之间通过"话题（topic）"传递消息
- 最终的控制命令通过 UDP 发给机器狗底层执行

### 1.2 系统数据流一图看懂

```
【硬件输入层】
  摄像头(MIPI F37, 960x544)
      │  NV12格式共享内存
      ▼ /hbmem_img
  hobot_codec  ──→  JPEG压缩  ──→  /image (CompressedImage)
      │
      │────────────────────────────────────────────┐
      ▼                                            ▼
【感知链】                                   【手势链（TROS官方）】
  perception_node                            mono2d_body_detection
  (YOLOv5s BPU推理)                         hand_lmk_detection
      │                                      hand_gesture_detection
      ▼ /perception/result_json                   │
  {"detections": [{"name":"person",          gesture_adapter_node
    "bbox":[x1,y1,x2,y2],"score":0.87}]}         │
                                                  ▼ /gesture/result_json
【语音链①：I2C硬件模块】                    {"gesture_value": 4.0}
  voice_control_node
  (轮询I2C 0x79@bus5)                【语音链②：USB麦克风】
      │                               usb_asr_text_node
      ▼ /voice/result_json            (arecord + Vosk识别)
  {"command":"stand","text":"站立"}         │
                                           ▼ /asr/text
                              intent_router_node
                                │            │
                                ▼            ▼
                         /voice/result   /chat/input_text
                         (控制词)         (对话词)

【决策层（最核心）】
  decision_node  ←─────  /perception/result_json
                ←─────  /gesture/result_json
                ←─────  /voice/result_json
      │
      ▼ /puppy_action
  {"mode":"follow_control","forward":0.5,"turn":-0.3}
  或 {"action":"sit","source":"gesture"}

【执行层】
  ros_udp_bridge
      │ UDP:5005
      ▼
  幻尔SDK执行层（机器狗底层进程）

【LLM对话链】
  /chat/input_text → chat_llm_bridge_node → /prompt_text
  hobot_llamacpp(Qwen2.5-0.5B) → /tts_text
  chat_llm_bridge_node → /chat/response_text

【IMU链】
  imu_node_ros2 (Board.get_imu(), 50Hz)
      │ /ros_robot_controller/imu_raw
      ▼
  ros_udp_bridge → UDP:5006
```

### 1.3 话题（Topic）完整清单

| 话题名 | 消息类型 | 发布者 | 订阅者 | 内容说明 |
|--------|---------|--------|--------|---------|
| `/hbmem_img` | hbm_img | mipi_cam | hobot_codec | 摄像头NV12原始帧（共享内存） |
| `/image` | CompressedImage | hobot_codec | perception_node, debug_preview | JPEG压缩图像 |
| `/hobot_mono2d_body_detection` | PerceptionTargets | mono2d_body | hand_lmk | 人体检测结果 |
| `/hobot_hand_lmk_detection` | PerceptionTargets | hand_lmk | hand_gesture | 手部21关键点 |
| `/hobot_hand_gesture_detection` | PerceptionTargets | hand_gesture | gesture_adapter | 手势分类结果 |
| `/gesture/result_json` | String(JSON) | gesture_adapter | decision_node | `{"gesture_value":4.0}` |
| `/perception/result_json` | String(JSON) | perception_node | decision_node, debug_preview | 检测框列表 |
| `/voice/result_json` | String(JSON) | voice_control / intent_router | decision_node | `{"command":"sit"}` |
| `/asr/text` | String(JSON) | usb_asr_text | intent_router | `{"text":"小狗你好"}` |
| `/chat/input_text` | String | intent_router / asr_wakeup | chat_llm_bridge | 对话文本 |
| `/prompt_text` | String | chat_llm_bridge | hobot_llamacpp | 转发给LLM的prompt |
| `/tts_text` | String | hobot_llamacpp | chat_llm_bridge | LLM流式输出片段 |
| `/chat/response_text` | String | chat_llm_bridge | （外部/TTS） | 合并后完整回复 |
| `/puppy_action` | String(JSON) | decision_node | ros_udp_bridge, debug_preview | 控制指令 |
| `/ros_robot_controller/imu_raw` | Imu | imu_node_ros2 | ros_udp_bridge | 加速度+角速度 |

---

## 第二章：从零部署——第一次跑起来

### 2.1 前置条件检查

```bash
# 1. 确认在 RDK X5 上（不是 x86 PC！）
uname -m   # 应该输出: aarch64

# 2. 确认 TROS Humble 已安装
ls /opt/tros/humble/
# 应该有 setup.bash, lib/, share/ 等目录

# 3. 确认摄像头
ls /dev/video*   # F37 摄像头接MIPI接口，通常不是video设备，由mipi_cam驱动直接访问

# 4. 确认I2C总线（语音模块）
ls /dev/i2c*
# 应该有 /dev/i2c-5（对应参数 i2c_bus=5）
i2cdetect -y 5   # 扫描总线5，应该看到 0x79 这个地址

# 5. 确认USB麦克风
arecord -l
# 应该列出 USB 音频设备，记住 card 号和 device 号
# 比如: card 0: USB Audio ..., device 0
# 对应的 device 就是 plughw:0,0

# 6. 确认模型文件存在
ls /app/model/basic/yolov5s_672x672_nv12.bin   # BPU推理模型
ls /app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf  # LLM
ls /app/puppy_ws/models/vosk-model-small-cn-0.22/  # 语音识别模型
```

### 2.2 编译工程

> **重要提示**：每次修改了 `.py` 文件后，都要重新执行 `colcon build` + `source install/setup.bash`，否则改动不生效！

```bash
# ① 进入工程目录
cd /app/puppy_ws

# ② source TROS环境（每次新开终端都要做！）
source /opt/tros/humble/setup.bash

# ③ 编译 puppy_brain 包（只编译我们写的节点，不编译整个TROS）
colcon build --packages-select puppy_brain

# 正常输出大概是：
# Starting >>> puppy_brain
# Finished <<< puppy_brain [5.2s]
# Summary: 1 package(s) finished

# ④ source 编译产物（让ROS2能找到刚编译的节点）
source /app/puppy_ws/install/setup.bash
```

### 2.3 启动完整系统

```bash
# 在 RDK X5 上执行：
cd /app/puppy_ws
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash
ros2 launch puppy_brain full_system.launch.py
```

### 2.4 验证系统是否正常运行

**启动后观察日志，正常情况应该看到：**
```
[mipi_cam] opened camera F37 960x544
[hobot_codec_encoder] start encode channel 1
[perception_node] Loading model: /app/model/basic/yolov5s_672x672_nv12.bin
[perception_node] perception_node started. waiting image topic: /image
[perception_node] subscribed /image as sensor_msgs/msg/CompressedImage   ← 关键！看到这个说明图像订阅成功
[decision_node] decision_node started. follow_enabled=True
[ros_udp_bridge] ros_udp_bridge started. action: /puppy_action -> udp=127.0.0.1:5005
[voice_control_node] ASR words initialized: 1=zhan li, 2=zuo xia, 3=ting xia
[imu_node_ros2] imu_node_ros2 started. publish topic=/ros_robot_controller/imu_raw, hz=50.0
[chat_llm_bridge_node] chat_llm_bridge_node started
```

**随后每1~5秒会看到推理日志：**
```
[perception_node] perception publish | frame_id=42 | image=960x544 | detections=1
[decision_node] [follow] target=1 cx=0.483 area=0.156 forward=0.45 turn=0.03
[ros_udp_bridge] UDP action: action=[follow_control] source=[follow]
```

**如果出错，常见问题排查：**

| 错误现象 | 可能原因 | 排查方法 |
|---------|---------|---------|
| `mipi_cam` 启动失败 | 摄像头没接好或型号不对 | 检查 MIPI 排线，确认 F37 型号 |
| `perception_node` 一直等 topic | codec 没启动成功 | 检查 hobot_codec 是否在运行：`ros2 node list` |
| `voice_control_node` I2C 读写失败 | 语音模块没接对总线 | `i2cdetect -y 5` 看是否有 0x79 |
| `usb_asr_text_node` 找不到设备 | USB麦克风没插或被占用 | `arecord -l` 检查设备列表 |
| `decision_node` 没输出 | follow 可能被关了 | 检查日志中 `follow_enabled` 的值 |

---

## 第三章：核心节点深度剖析

### 3.1 perception_node.py — 视觉感知节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/perception_node.py`

**这个节点干什么？**

简单说：**把图片变成"画面里有没有人、人在哪里"的JSON数据**。

具体流程：
1. 等待 `/image` 话题出现（会每秒检查一次，支持自动识别JPEG/RAW两种格式）
2. 收到图像帧 → 解码 → resize到672×672 → 转换为NV12格式
3. 把NV12数据送进BPU（地平线神经网络加速器）推理
4. 调用 `libpostprocess.so` 的C++后处理函数做NMS去重
5. 把检测结果打包成JSON发布到 `/perception/result_json`

#### NV12是什么？

NV12是地平线BPU原生支持的图像格式（YUV 4:2:0），相比BGR/RGB节省内存带宽，BPU推理必须用这个格式。节点里的 `bgr_to_nv12()` 函数就是做这个转换的：

```python
# perception_node.py 第298-314行
def bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
    h, w = bgr.shape[:2]
    yuv_i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)

    y_size = h * w
    uv_size = y_size // 4

    y = yuv_i420[:y_size]                              # Y分量（亮度）
    u = yuv_i420[y_size:y_size + uv_size].reshape(...)  # U分量（色度）
    v = yuv_i420[y_size + uv_size:].reshape(...)        # V分量（色度）

    # NV12 格式：Y平面 + 交错UV平面（U和V交替排列）
    uv = np.empty((h // 2, w), dtype=np.uint8)
    uv[:, 0::2] = u   # 偶数位置放U
    uv[:, 1::2] = v   # 奇数位置放V

    nv12 = np.concatenate([y, uv.reshape(-1)])
    return nv12
```

#### 三个线程设计（重要！）

| 线程 | 作用 | 队列 |
|------|------|------|
| 主线程（ROS回调） | 接收图像，解码+转NV12，放入 `frame_queue` | → frame_queue |
| `ai_thread` | 从队列取帧，BPU推理，结果放入 `result_queue` | frame_queue → result_queue |
| `publisher_thread` | 从结果队列取数据，发布ROS消息 | result_queue → |

三个线程之间用队列解耦，队列满了会自动丢弃最旧的帧（`maxsize=3`）。这个设计保证了：**BPU推理的速度（约15fps）不会阻塞图像接收，也不会积压太多队列占用内存**。

#### 自动识别图像格式（第184-227行）

节点启动时不会立刻订阅 `/image`，而是每秒检查一次这个话题是否存在，以及它的消息类型：

```python
def try_create_image_subscription(self):
    # 检查话题是否存在
    topics = dict(self.get_topic_names_and_types())
    if self.image_topic not in topics:
        return  # 还没有，下次再检查

    # 如果是 JPEG 压缩图（CompressedImage）
    if 'sensor_msgs/msg/CompressedImage' in topic_types:
        self.image_sub = self.create_subscription(CompressedImage, ...)
        return

    # 如果是原始图（Image, bgr8/rgb8/mono8）
    if 'sensor_msgs/msg/Image' in topic_types:
        self.image_sub = self.create_subscription(Image, ...)
        return
```

这个设计的巧妙之处：**不管上游发布的是什么格式，perception_node 都能自动适配**。

#### 发布的JSON格式

每次检测结果发布到 `/perception/result_json` 的消息：

```json
{
  "timestamp": 1745123456.789,
  "frame_id": 42,
  "image_width": 960,
  "image_height": 544,
  "detections": [
    {
      "name": "person",
      "bbox": [120.5, 80.3, 450.2, 520.1],
      "score": 0.873
    },
    {
      "name": "cat",
      "bbox": [600.0, 200.0, 750.0, 380.0],
      "score": 0.421
    }
  ]
}
```

字段含义：
- `timestamp`：Unix时间戳（float秒）
- `frame_id`：帧序号，单调递增
- `image_width/height`：原始图像尺寸
- `detections`：检测到的目标列表
  - `name`：COCO类别名（80类，如 person, car, dog 等）
  - `bbox`：`[x1, y1, x2, y2]` 像素坐标，相对原始图像左上角
  - `score`：置信度 0~1

#### 关键参数（在 `full_system.launch.py` 里配置）

| 参数名 | 当前值 | 可调范围 | 调整效果 |
|--------|--------|---------|---------|
| `model_path` | `/app/model/basic/yolov5s_672x672_nv12.bin` | 任意BPU模型路径 | 换检测模型 |
| `score_threshold` | `0.25` | 0.1～0.7 | 越高=越严格，漏检少、误检也少；越低=越宽松 |
| `nms_threshold` | `0.45` | 0.3～0.6 | 越低=重叠框合并更激进；越高=允许更多重叠框 |
| `nms_top_k` | `20` | 5～50 | 最多保留几个检测框 |
| `input_width/height` | `672` / `672` | 取决于模型 | 必须和模型输入尺寸一致 |
| `orig_width/height` | `960` / `544` | 取决于摄像头 | 原始图像尺寸，用于坐标映射 |
| `log_interval_sec` | `5.0` | 0.5～30 | 推理日志打印间隔，不影响推理速度 |

---

### 3.2 decision_node.py — 决策仲裁节点（最核心）

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/decision_node.py`

**这个节点干什么？**

这是整个系统的大脑，做三件事：
1. **接收**三路输入：视觉感知结果、手势指令、语音指令
2. **仲裁优先级**：语音 > 手势 > 视觉跟随
3. **计算控制量**：根据人在画面里的位置和大小，计算前进速度和转向速度

#### 三个回调函数分别做什么

```python
# 视觉感知回调（第128-146行）
def perception_callback(self, msg):
    # 如果语音锁或手势锁激活中 → 直接忽略（语音/手势优先级更高）
    # 如果跟随未开启 → 直接忽略
    # 否则：调用 decide_follow_control() 计算控制量并发布

# 手势回调（第148-204行）
def gesture_callback(self, msg):
    # 如果语音优先模式开启且语音锁激活中 → 忽略手势
    # 解析 gesture_value → 查表映射为动作 → 更新 follow_enabled
    # 发布动作指令

# 语音回调（第206-243行）
def voice_callback(self, msg):
    # 只处理 stand/sit/stop 三种指令（其他忽略）
    # 设置语音锁（防止跟随立刻覆盖语音指令）
    # 发布动作指令（作为最高优先级）
```

#### 视觉跟随算法逐行解读

`decide_follow_control()` 函数（第311-391行）是整个系统的核心算法：

```python
def decide_follow_control(self, detections):
    now = time.time()

    # ============ 第一步：选出画面里最大的那个人 ============
    best_person = self.select_best_person(detections)
    # select_best_person 逻辑（第283-309行）：
    # - 遍历所有 detections
    # - 只看 name=='person' 的
    # - 计算每个 person bbox 的面积占比 = bbox面积 / 画面面积
    # - 过滤掉面积 < min_valid_area_ratio(0.015) 的噪声小框
    # - 返回面积最大的那个人

    if best_person is None:
        # ============ 没有检测到人 ============
        time_since_last_seen = now - self.last_person_time

        if time_since_last_seen < self.ghost_memory_time:
            # ghost_memory_time=0.3秒内还见过人
            # 平滑减速到0（不是急停，防止短暂遮挡导致急停）
            forward_cmd, turn_cmd = self.smooth_control(0.0, 0.0)
        else:
            # 超过0.3秒没见到人 → 彻底刹车
            self.last_forward_cmd = 0.0
            self.last_turn_cmd = 0.0
            forward_cmd, turn_cmd = 0.0, 0.0

        return {'mode': 'follow_control', 'forward': forward_cmd, 'turn': turn_cmd, ...}

    # ============ 第二步：计算目标水平位置 ============
    x1, y1, x2, y2, area_ratio = best_person
    x_center = (x1 + x2) / 2.0               # 检测框中心x坐标（像素）
    cx_ratio = x_center / self.image_width     # 归一化到 0~1（0=最左，1=最右）
    self.last_person_time = now               # 更新最后一次见到人的时间

    # ============ 第三步：计算转向 ============
    error = cx_ratio - self.center_ratio      # 与画面中心的偏差
    # 例如：目标在 cx_ratio=0.3，center=0.5，error=-0.2，目标偏左了
    abs_error = abs(error)

    if abs_error < self.turn_deadband_ratio:  # 0.09范围内不转向（防抖动）
        raw_turn = 0.0
    else:
        # 线性映射偏差到转速
        effective_error = abs_error - self.turn_deadband_ratio  # 去掉死区
        norm_error = effective_error / self.max_turn_error_ratio  # 归一化到 0~1
        norm_error = self.clamp(norm_error, 0.0, 1.0)             # 截断
        turn_mag = self.turn_gain * norm_error                     # 乘增益
        raw_turn = turn_mag if error < 0 else -turn_mag           # error<0→左转, error>0→右转

    raw_turn = self.clamp(raw_turn, -1.0, 1.0)

    # ============ 第四步：计算前进速度 ============
    if area_ratio >= self.follow_area_near_stop:    # 0.42，太近了刹车
        raw_forward = 0.0
    elif area_ratio <= self.follow_area_far_walk:   # 0.10，太远了全速
        raw_forward = self.forward_max               # 0.95
    else:
        # 中间区域：线性映射（平方曲线，靠近时减速更快）
        ratio = (near_stop - area_ratio) / (near_stop - far_walk)
        ratio = self.clamp(ratio, 0.0, 1.0)
        ratio = ratio * ratio                       # 平方！靠近时更灵敏
        raw_forward = forward_min + (forward_max - forward_min) * ratio

    # ============ 第五步：根据转向误差调整前进速度 ============
    # 人在画面中间（误差小）→ 多走少转
    # 人在画面边缘（误差大）→ 多转少走
    if abs_error < self.small_error_ratio:           # < 0.10
        raw_forward *= self.small_error_forward_keep  # ×0.96（几乎不减速）
        if raw_turn != 0.0 and raw_forward < self.min_cruise_forward_small:
            raw_forward = self.min_cruise_forward_small  # 转向时至少保持0.22的速度
    elif abs_error < self.large_error_ratio:         # < 0.24
        raw_forward *= self.mid_error_forward_keep    # ×0.78
        if raw_turn != 0.0 and raw_forward < self.min_cruise_forward_mid:
            raw_forward = self.min_cruise_forward_mid   # 转向时至少保持0.16
    else:                                            # >= 0.24
        raw_forward *= self.large_error_forward_keep  # ×0.18（主要转向，很少前进）

    raw_forward = self.clamp(raw_forward, 0.0, 1.0)

    # ============ 第六步：低通滤波平滑 ============
    forward_cmd, turn_cmd = self.smooth_control(raw_forward, raw_turn)
    # smoothed = (1 - alpha) × last_cmd + alpha × target_cmd
    # alpha=0.28 → 28%新值 + 72%旧值
    # 效果：速度不会突变，机器人运动更平滑
```

#### 发布的控制指令 JSON 格式

**类型①：连续跟随控制（视觉跟随时）**
```json
{
  "mode": "follow_control",
  "forward": 0.45,
  "turn": -0.22,
  "source": "follow",
  "timestamp": 1745123456.789,
  "follow_enabled": true,
  "cx_ratio": 0.483,
  "area_ratio": 0.156,
  "lost_target": false
}
```

**类型②：动作指令（手势或语音触发时）**
```json
{
  "action": "sit",
  "source": "gesture",
  "timestamp": 1745123456.789,
  "follow_enabled": false,
  "gesture": "4.0",
  "gesture_value": 4.0
}
```

#### 全部参数完整解读

| 参数名 | launch中实际值 | 代码默认值 | 详细说明 |
|--------|--------------|-----------|---------|
| `image_width` | `960.0` | `960.0` | 图像宽度，用于计算cx_ratio，**必须和摄像头分辨率一致** |
| `image_height` | `544.0` | `544.0` | 图像高度，必须和摄像头分辨率一致 |
| `follow_area_near_stop` | `0.42` | `0.55` | 人的bbox面积占总画面42%时刹车。约等于人站在1米以内。**增大=允许更近才停** |
| `follow_area_far_walk` | `0.10` | `0.10` | 人的bbox面积低于10%时全速追。约等于人在4~5米外。**减小=更远才全速** |
| `min_valid_area_ratio` | `0.015` | `0.015` | 检测框面积低于1.5%忽略（过滤远处噪声小框） |
| `center_ratio` | `0.50` | `0.50` | 目标期望位于画面水平50%位置（正中）。一般不动 |
| `turn_deadband_ratio` | `0.09` | `0.07` | 目标偏离中心9%以内不转向（防抖）。**越小=越灵敏但易抖，越大=越稳但有延迟** |
| `max_turn_error_ratio` | `0.28` | `0.22` | 偏差达到28%时按最大转速转。**越小=更快达到最大转速** |
| `turn_gain` | `0.85` | `1.00` | 转向增益。**越大=转越猛，越小=转越柔** |
| `small_error_ratio` | — | `0.10` | 误差小于此值时，前进速度保持96% |
| `large_error_ratio` | — | `0.24` | 误差大于此值时，前进速度只剩18% |
| `small_error_forward_keep` | — | `0.96` | 小误差时前进速度保持比例 |
| `mid_error_forward_keep` | — | `0.78` | 中等误差时前进速度保持比例 |
| `large_error_forward_keep` | — | `0.18` | 大误差时前进速度保持比例 |
| `min_cruise_forward_small` | — | `0.22` | 小误差+有转向时，最小前进速度（防止原地转不前进） |
| `min_cruise_forward_mid` | — | `0.16` | 中误差+有转向时，最小前进速度 |
| `forward_min` | `0.0` | `0.0` | 最小前进速度，设0=距离近时可以完全停下 |
| `forward_max` | `0.95` | `0.95` | 最大前进速度（归一化0~1）。对应底层全速的95% |
| `ghost_memory_time` | `0.30` | `0.30` | 目标消失后维持0.3秒惯性（防遮挡急停）。**增大=惯性更久** |
| `publish_repeat_sec` | `0.15` | `0.15` | 即使控制量没变，也至少每0.15秒发一次（保持底层活跃） |
| `gesture_hold_sec` | `0.8` | `0.8` | 手势结果有效期0.8秒（超时忽略） |
| `follow_default_enabled` | `True` | `True` | 启动时默认开启跟随。改False=默认关闭 |
| `gesture_action_lock_sec` | `2.5` | `2.5` | sit/stand手势后锁定2.5秒 |
| `gesture_stop_lock_sec` | `1.0` | `1.0` | stop手势锁定1.0秒 |
| `voice_action_lock_sec` | `2.5` | `2.5` | 语音指令后锁定2.5秒 |
| `voice_priority_enabled` | `True` | `True` | True=语音最高优先级，会屏蔽手势 |
| `control_smooth_alpha` | `0.28` | `0.28` | 低通滤波系数，28%新值+72%旧值。**增大=响应更快但更抖** |
| `turn_zero_threshold` | `0.05` | `0.04` | 转速低于此值截断为0（防止微小漂移） |
| `forward_zero_threshold` | `0.05` | `0.03` | 前进速度低于此值截断为0 |
| `debug_print_sec` | `0.5` | `0.5` | 调试日志打印间隔 |

> **注意**：有些参数在代码里有 `declare_parameter` 默认值，但在 `full_system.launch.py` 里没有显式设置（比如 `small_error_ratio`），这时使用代码里的默认值。

#### 手势映射表

| gesture_value | 对应动作 | 对 follow_enabled 的影响 | 锁定时间 |
|--------------|---------|------------------------|---------|
| `1.0` | `follow_on` | 设为 True | 0.5秒 |
| `2.0` | `follow_off` | 设为 False，清零速度 | 0.5秒 |
| `3.0` | `stop` | 保持 True，清零速度 | 1.0秒 |
| `4.0` | `sit` | 设为 False，清零速度 | 2.5秒 |
| `5.0` | `stand` | 设为 True，清零速度 | 2.5秒 |

---

### 3.3 ros_udp_bridge.py — ROS→UDP转发桥

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/ros_udp_bridge.py`

**这个节点干什么？**

把 ROS2 话题消息"翻译"成 UDP 包发给机器狗底层进程。机器狗底层是幻尔的闭源进程，监听 UDP 端口，我们没法直接调用它的函数，只能发 UDP。

#### 两条转发通道

**通道①：`/puppy_action` → UDP:5005（动作/控制）**

转发规则有两种情况（第46-78行）：

```python
def action_callback(self, msg: String):
    raw = msg.data
    payload = json.loads(raw)

    if payload.get('mode') == 'follow_control':
        # 情况A：连续控制 → 原样转发整个JSON字符串
        # 底层SDK自己解析 forward/turn 字段
        self.sock.sendto(raw.encode('utf-8'), (self.udp_ip, self.action_udp_port))
        return

    action_cmd = payload.get('action', None)
    if action_cmd:
        # 情况B：离散动作 → 只发 action 字段的值字符串
        # 比如 sit → 发送 "sit"，stand → 发送 "stand"
        self.sock.sendto(action_cmd.encode('utf-8'), ...)
```

**通道②：`/ros_robot_controller/imu_raw` → UDP:5006（IMU数据）**

把ROS的 `sensor_msgs/Imu` 消息打包成JSON转发：

```json
{
  "type": "imu",
  "linear_acceleration": {"x": 0.02, "y": -0.01, "z": 9.81},
  "angular_velocity": {"x": 0.001, "y": 0.002, "z": -0.003},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

#### 可调参数

| 参数名 | 当前值 | 说明 |
|--------|--------|------|
| `udp_ip` | `127.0.0.1` | 目标IP。同机器=127.0.0.1；跨机器=那台机器的IP |
| `action_udp_port` | `5005` | 动作指令端口 |
| `imu_udp_port` | `5006` | IMU数据端口 |

---

### 3.4 voice_control_node.py — I2C语音控制节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/voice_control_node.py`

**这个节点干什么？**

通过 Linux I2C 总线与语音识别硬件模块通信，轮询它是否识别到了语音命令。

#### 工作流程

1. 启动时向硬件模块写入词库（先擦除旧词库，再写入3个词）
2. 每100ms通过I2C读一次结果寄存器
3. 读到非零ID（1/2/3）就查字典，发布对应的JSON指令

#### I2C通信协议（ASR 类，第15-73行）

| 寄存器地址 | 操作 | 含义 |
|-----------|------|------|
| `100（0x64）` | 写0x64后读1字节 | 读取识别结果ID（0=无，1/2/3=词） |
| `101（0x65）` | 写字节数据0 | 擦除所有词库 |
| `102（0x66）` | 写字节数据mode | 设置识别模式（1=环境模式） |
| `160（0xA0）` | 写块数据 `[id, 'z','h','a','n',' ','l','i']` | 添加一个词条 |

```python
# 读取识别结果的底层操作（第38-46行）
def get_result(self) -> int:
    self.bus.write_byte(self.address, 0x64)   # 先写寄存器地址
    value = self.bus.read_byte(self.address)   # 再读1字节
    return int(value)  # 0=无识别，1/2/3=对应词条ID
```

#### 如何添加更多语音词

打开 `voice_control_node.py`：

**Step 1** — 在 `init_asr_words()` 函数（第119行）添加词条：
```python
def init_asr_words(self):
    ok1 = self.asr.erase_words()      # 擦除全部
    ok2 = self.asr.set_mode(self.mode)
    ok3 = self.asr.add_words(1, 'zhan li')    # ID=1 → 站立
    ok4 = self.asr.add_words(2, 'zuo xia')    # ID=2 → 坐下
    ok5 = self.asr.add_words(3, 'ting xia')   # ID=3 → 停下
    # ⬇️ 新增
    ok6 = self.asr.add_words(4, 'qian jin')   # ID=4 → 前进
    ok7 = self.asr.add_words(5, 'hou tui')    # ID=5 → 后退
```

**Step 2** — 在 `id_to_command` 字典（第98行）添加映射：
```python
self.id_to_command = {
    1: {'command': 'stand', 'text': '站立'},
    2: {'command': 'sit', 'text': '坐下'},
    3: {'command': 'stop', 'text': '停下'},
    # ⬇️ 新增
    4: {'command': 'walk', 'text': '前进'},
    5: {'command': 'backward', 'text': '后退'},
}
```

**Step 3** — 在 `decision_node.py` 的 `voice_callback()` 里（第215行）添加对新命令的处理。

#### 参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `i2c_bus` | `5` | I2C总线号，对应 `/dev/i2c-5` |
| `i2c_addr` | `0x79` | 语音模块I2C地址，固定的 |
| `mode` | `1` | 1=环境唤醒模式（不需要专门的唤醒词） |
| `init_words` | `True` | 每次启动重新写词库，True=方便调试 |
| `poll_interval` | `0.10` | 每100ms轮询一次 |
| `cooldown_sec` | `1.5` | 同一指令1.5秒内不重复触发 |
| `debug_log_interval_sec` | `30.0` | 每30秒打印一次原始I2C读取值 |

---

### 3.5 usb_asr_text_node.py — USB语音识别节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/usb_asr_text_node.py`

**工作流程**：
```
每0.1秒检查 busy 标志
       ↓ 如果不忙
调用 arecord 录音3秒（生成临时WAV）
       ↓
用 Vosk 离线识别 WAV 文件
       ↓
识别出文字后发布到 /asr/text
       ↓
休眠0.5秒（loop_sleep_sec）
       ↓ 循环
```

#### 为什么用 `arecord` 而不是 Python 录音库？

`arecord` 是 Linux 系统级工具，直接调用 ALSA 驱动，比 Python `sounddevice`/`pyaudio` 更稳定，特别是在嵌入式板子上。

#### Vosk 离线识别原理

Vosk 使用 Kaldi 的 GMM-HMM 语音识别引擎，`vosk-model-small-cn-0.22` 是约44MB的中文小模型，完全离线运行，不需要网络。识别过程（第108-137行）：

```python
def recognize_wav(self, wav_path: str) -> str:
    wf = wave.open(wav_path, 'rb')
    rec = KaldiRecognizer(self.model, wf.getframerate())

    # 每次读 4000 帧送入识别器
    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text_parts.append(result.get('text', ''))

    # 最终结果（可能包含前面没确认的部分）
    final_result = json.loads(rec.FinalResult())
    return ''.join(text_parts)
```

#### 常见问题排查

```bash
# 问题1：找不到麦克风设备
arecord -l
# 如果没看到USB麦克风，检查USB连接

# 问题2：录音失败（device not found）
# 查看实际设备号：
arecord -l
# 如果显示 card 1: USB ..., device 0
# 那么 device 参数要改成 plughw:1,0

# 问题3：识别出来全是空字符串
# 手动测试录音：
arecord -D plughw:0,0 -d 3 -f S16_LE -r 16000 -c 1 /tmp/test.wav
aplay /tmp/test.wav  # 回放检查
```

---

### 3.6 intent_router_node.py — 意图路由节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/intent_router_node.py`

**这个节点干什么？**

USB麦克风识别出来的文字，有些是"坐下"这样的控制命令，有些是"你叫什么名字"这样的对话。这个节点用规则把它们分开路由。

#### 关键词匹配逻辑（第90-109行）

```python
def route_intent(self, text: str):
    # text 是去掉空格的紧凑文本

    # 规则1：停止类
    if any(kw in text for kw in ['停下', '停止', '别动', '不要动']):
        return {'type': 'control', 'command': 'stop'}

    # 规则2：坐下类
    if any(kw in text for kw in ['坐下', '坐下来', '请坐下']):
        return {'type': 'control', 'command': 'sit'}

    # 规则3：站立类
    if any(kw in text for kw in ['站立', '站起来', '起来', '请站起来']):
        return {'type': 'control', 'command': 'stand'}

    # 规则4：开始跟随
    if any(kw in text for kw in ['开始跟随', '跟着我', '跟随我']):
        return {'type': 'control', 'command': 'follow_start'}

    # 规则5：停止跟随
    if any(kw in text for kw in ['停止跟随', '不要跟了', '别跟了']):
        return {'type': 'control', 'command': 'follow_stop'}

    # 其他所有：去对话链
    return {'type': 'chat'}
```

#### 防重复机制

控制类指令有2秒冷却（`control_cooldown_sec`），同一指令2秒内不重复发布。

#### 如何增加新的控制词

直接在对应的列表里添加：
```python
if any(kw in text for kw in ['开始跟随', '跟着我', '跟随我', '靠近我', '过来']):
    return {'type': 'control', 'command': 'follow_start'}
```

---

### 3.7 chat_llm_bridge_node.py — LLM对话桥

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/chat_llm_bridge_node.py`

**为什么需要这个节点？**

`hobot_llamacpp` 是流式LLM，它把回答分段发出（比如："你"→"好"→"我"→"是"→"小"→"狗"），每个片段单独发一条 `/tts_text` 消息。这个节点的任务就是：**把这些碎片收集起来，等LLM说完了，把完整答案打包发出去**。

#### 判断LLM说完了的机制（flush超时）

```python
def on_flush_timer(self):  # 每200ms触发一次
    if not self._segments:
        return  # 没有收集到任何片段，跳过

    if (now - self._last_segment_time) < self.flush_timeout_sec:
        return  # 距上次收到片段不到2秒，还没说完，继续等

    # 超过2秒没收到新片段 → 认为LLM说完了
    merged = ''.join(self._segments)  # 拼接成完整答案
    self.chat_out_pub.publish(merged)  # 发出完整回复
```

#### 参数说明

| 参数 | launch中值 | 代码默认值 | 说明 |
|------|-----------|-----------|------|
| `flush_timeout_sec` | `2.0` | `0.8` | 等待多久没新片段才认为说完。launch覆盖为2.0 |
| `chat_input_topic` | `/chat/input_text` | `/chat/input_text` | 接收对话输入的话题 |
| `chat_output_topic` | `/chat/response_text` | `/chat/response_text` | 发布完整回复的话题 |
| `llm_input_topic` | `/prompt_text` | `/prompt_text` | 发给LLM的话题 |
| `llm_output_topic` | `/tts_text` | `/tts_text` | 接收LLM流式输出的话题 |

---

### 3.8 debug_preview_node.py — 调试可视化节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/debug_preview_node.py`

**功能**：在连接了 HDMI 显示器的情况下，打开一个 OpenCV 窗口，实时显示：
- **顶部黑色状态栏**：ACTION（当前动作）、SOURCE（来源）、GESTURE（手势）、FOLLOW状态、DETECTIONS数量
- **画面上**：人的检测框（黄色）和其他物体（青色），以及置信度数字

#### 如何启动

`full_system.launch.py` 里默认**没有启动这个节点**。需要单独启动：

```bash
# 新开一个终端：
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash
ros2 run puppy_brain debug_preview_node
```

或者加到 launch 文件里：
```python
Node(
    package='puppy_brain',
    executable='debug_preview_node',
    name='debug_preview_node',
    output='screen',
    parameters=[{
        'display_scale': 0.75,
        'max_fps': 10.0,
    }]
),
```

#### 参数说明

| 参数 | 说明 |
|------|------|
| `display_scale` | 显示缩放，1.0=原尺寸(960x544)，0.5=半尺寸 |
| `max_fps` | 最大显示帧率，不影响推理速度 |
| `window_name` | 窗口标题 |

---

### 3.9 gesture_adapter_node.py — 手势协议适配器

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/gesture_adapter_node.py`

**为什么需要这个节点？**

TROS官方的手势识别发布的消息格式是 `ai_msgs/msg/PerceptionTargets`（一个复杂的ROS自定义消息），包含了人体框、手部关键点、手势分类等很多信息。我们的 `decision_node` 不想依赖 `ai_msgs`，所以用这个适配器把有用的信息提取出来，转成简单的JSON字符串。

**核心逻辑**（第31-56行）：
```python
def callback(self, msg: PerceptionTargets):
    gesture_value = None
    for target in msg.targets:           # 遍历所有检测目标
        for attr in target.attributes:   # 遍历每个目标的属性
            if attr.type == 'gesture':   # 找到手势属性
                gesture_value = attr.value  # 取手势分类值（1.0/2.0/3.0/4.0/5.0）
                break
        if gesture_value is not None:
            break

    out = {"gesture_value": gesture_value, "track_id": track_id}
    self.pub.publish(out)  # 发布到 /gesture/result_json
```

---

### 3.10 imu_node_ros2.py — IMU发布节点

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/imu_node_ros2.py`

**功能**：调用幻尔SDK，以50Hz频率读取机器狗内置IMU（陀螺仪+加速度计），转成标准ROS2 `sensor_msgs/Imu` 消息发布。

**SDK调用方式**（第10-15行，30-31行）：
```python
from ros_robot_controller_sdk import Board  # 幻尔官方SDK

self.board = Board()
self.board.enable_reception()  # 启动数据接收

# 每次读取（第54行）：
data = self.board.get_imu()
# 返回: [ax, ay, az, gx, gy, gz]
# ax/ay/az: 三轴加速度（m/s²）
# gx/gy/gz: 三轴角速度（rad/s）
```

**注意**：四元数姿态字段目前全部填了 `(0,0,0,1)`（单位四元数=无旋转）。如果需要姿态估计，需要额外集成 IMU 融合算法（比如 Madgwick 滤波器）。

---

### 3.11 ai_vision_node.py — 旧版一体化视觉节点（已弃用）

**源码路径**：`puppy_ws/src/puppy_brain/puppy_brain/ai_vision_node.py`

这是早期版本，把"摄像头+推理+决策"全写在一个节点里。代码逻辑：

- 自己用 `srcampy.Camera()` 开摄像头（不走ROS话题）
- 图像尺寸写死1920×1080
- 决策是离散的四种：`stop`/`walk`/`turn_left`/`turn_right`
- 无手势支持、无语音支持、无LLM支持、无低通滤波

**运行方式**（注意：它会自己开摄像头，不要和mipi_cam同时运行）：
```bash
ros2 run puppy_brain ai_vision_node
```

---

## 第四章：三种启动模式与场景选择

### 4.1 模式一：完整系统（`full_system.launch.py`）

**包含**：摄像头 + 手势链 + 感知 + 语音(I2C) + 决策 + UDP桥 + IMU + LLM + WebSocket预览

**适合**：正式比赛/演示

```bash
ros2 launch puppy_brain full_system.launch.py
```

**WebSocket预览**：浏览器访问 `http://[RDK-IP]:8080` 可看到摄像头实时画面，叠加了人体检测框。

### 4.2 模式二：纯跟随（`follow_only.launch.py`）

**包含**：感知 + 决策 + UDP桥（无手势、无语音、无LLM）

**适合**：调试跟随算法

```bash
ros2 launch puppy_brain follow_only.launch.py
```

**注意**：这个 launch 配置的是 1920×1080 摄像头（旧版配置），如果用 F37 摄像头960×544，需要修改 launch 里的 `image_width/image_height` 和 `orig_width/orig_height`。

### 4.3 模式三：纯手势测试（`gesture_only.launch.py`）

**包含**：摄像头 + 完整手势链（5级）+ 手势适配器 + 决策节点

**适合**：单独调试手势识别准确率

```bash
ros2 launch puppy_brain gesture_only.launch.py
```

---

## 第五章：最常见的调参场景与解决方案

### 场景 1：跟随时机器人老是冲撞（太近了还在走）

**问题根因**：`follow_area_near_stop` 太小，人太近时还没达到停止阈值

**解决**：在 `full_system.launch.py` 把 `follow_area_near_stop` 改大：
```python
'follow_area_near_stop': 0.50,  # 人占50%面积就停（更保守）
```

### 场景 2：跟随时老是追不上（目标跑远了不追）

**问题根因**：`follow_area_far_walk` 太大

**解决**：
```python
'follow_area_far_walk': 0.05,   # 人只占5%才全速（更激进追远）
```

### 场景 3：转向时机器人乱抖（左右摇摆）

**问题根因1**：`turn_deadband_ratio` 太小
```python
'turn_deadband_ratio': 0.12,  # 从0.09改大到0.12
```

**问题根因2**：`control_smooth_alpha` 太大
```python
'control_smooth_alpha': 0.15,  # 从0.28改小到0.15
```

### 场景 4：语音命令说了但没反应

**排查步骤**：
```bash
# 新开终端，监听语音结果话题
ros2 topic echo /voice/result_json

# 同时说"站立"，看有没有消息打印
# 如果没有 → 问题在 voice_control_node（I2C通信）
# 检查：
i2cdetect -y 5  # 确认0x79设备存在
```

### 场景 5：手势识别出来但动作不执行

**排查步骤**：
```bash
ros2 topic echo /gesture/result_json   # 看手势值
ros2 topic echo /puppy_action           # 看决策输出
```

### 场景 6：想手动发消息测试机器狗

```bash
# 手动发送 "sit" 指令：
ros2 topic pub --once /puppy_action std_msgs/msg/String \
  '{"data": "{\"action\": \"sit\", \"source\": \"manual\"}"}'

# 手动发送跟随控制（前进0.5，不转向）：
ros2 topic pub --once /puppy_action std_msgs/msg/String \
  '{"data": "{\"mode\": \"follow_control\", \"forward\": 0.5, \"turn\": 0.0}"}'
```

### 场景 7：想在运行时修改参数（不重启）

```bash
# 修改刹车距离
ros2 param set /decision_node follow_area_near_stop 0.50

# 修改转向增益
ros2 param set /decision_node turn_gain 0.6

# 查看当前所有参数值
ros2 param list /decision_node
ros2 param get /decision_node follow_area_near_stop
```

---

## 第六章：旧版节点与新版节点的对比

| 对比项 | 旧版 `ai_vision_node` | 新版 `perception_node` + `decision_node` |
|-------|----------------------|----------------------------------------|
| 摄像头 | 自己开相机（`srcampy.Camera`） | 订阅 `/image` 话题 |
| 图像尺寸 | 写死1920×1080 | 通过参数配置 |
| 参数配置 | 全写死在代码里 | 全部通过launch参数配置 |
| 模型路径 | 写死 | launch参数可改 |
| 决策逻辑 | 离散4动作（stop/walk/turn_left/turn_right） | 连续速度控制 |
| 低通滤波 | 无 | 有 `control_smooth_alpha` 滤波 |
| 手势支持 | 无 | 有 |
| 语音支持 | 无 | 有 |
| LLM支持 | 无 | 有 |
| 幽灵记忆 | 条件苛刻（>0.35才触发） | 只要消失就触发（更鲁棒） |
| 调试可视化 | 直接在自己线程里显示 | 独立的 `debug_preview_node` |

---

## 第七章：工具脚本详解

### 7.1 `tools/asr_wakeup_loop_router.py` — 唤醒词对话循环

**用法**：
```bash
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash
python3 /app/puppy_ws/tools/asr_wakeup_loop_router.py
```

**触发方式**：说 "**小狗** + 问题"，比如"小狗你能做什么"

**内部逻辑**：
1. 循环录3秒音频（44100Hz采样，需重采样到16000Hz）
2. Vosk识别文字
3. 检测是否包含唤醒词（`WAKEUP_KEYWORDS` 列表）
4. 如果包含唤醒词，提取唤醒词后面的文字
5. 过滤掉纯控制词（坐下/站立等，让I2C模块处理）
6. 发布到 `/chat/input_text` → LLM回答

**唤醒词列表**（第37-38行）：
```python
WAKEUP_KEYWORDS = [
    "小狗", "小狗狗", "晓狗", "小够", "小苟", "小古",
    # 可以添加更多谐音或名字
]
```

**与 `usb_asr_text_node` + `intent_router_node` 的区别**：

| 对比 | usb_asr + intent_router | asr_wakeup_loop_router |
|------|------------------------|----------------------|
| 工作方式 | ROS节点，通过launch启动 | 独立脚本，手动运行 |
| 采样率 | 16000Hz（直录） | 44100Hz（需重采样） |
| 唤醒词 | 不需要 | 必须说唤醒词 |
| 控制命令 | 支持（intent_router分流） | 不支持（过滤掉了） |
| 适用场景 | 系统正式运行 | 开发调试/纯对话测试 |

### 7.2 `tools/sherpa_kws_mic_test.py` — 流式唤醒词测试

**和 Vosk 方案的区别**：

| 对比 | Vosk（当前用的） | Sherpa-ONNX（测试工具） |
|------|----------------|------------------------|
| 工作方式 | 录一段→识别（3秒延迟） | 实时流式检测（延迟极低） |
| 识别范围 | 全文字 | 只检测特定关键词 |
| CPU占用 | 识别期间偏高 | 持续低占用 |

```bash
python3 /app/puppy_ws/tools/sherpa_kws_mic_test.py
# 运行后直接说"小狗"，会打印 WAKE DETECTED
```

---

## 第八章：40pin GPIO 接口示例详解

### 8.1 GPIO 管脚编号规则

RDK X5 使用 **BCM（Broadcom）编号**，但在代码里用的是 **board编号（物理位置编号）**：

```python
import Hobot.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)  # 使用物理位置编号（1~40）
```

### 8.2 各文件详解

**`simple_out.py` — LED闪烁（最简单的GPIO输出）**
```python
output_pin = 37          # 物理管脚37
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
while True:
    GPIO.output(output_pin, GPIO.HIGH)  # 高电平（LED亮）
    time.sleep(1)
    GPIO.output(output_pin, GPIO.LOW)   # 低电平（LED灭）
    time.sleep(1)
```

**`simple_pwm.py` — PWM控制（舵机/电机转速）**
```python
output_pin = 33    # 只有32和33支持硬件PWM
p = GPIO.PWM(output_pin, 48000)  # 频率48kHz
p.start(0)
for i in range(101):       # 占空比0→100
    p.ChangeDutyCycle(i)
    time.sleep(0.01)
```

**`button_event.py` — 按键等待（阻塞模式）**
```python
button_pin = 37
GPIO.setup(button_pin, GPIO.IN)
GPIO.wait_for_edge(button_pin, GPIO.FALLING)  # 阻塞等下降沿
```

**`button_interrupt.py` — 按键中断（非阻塞）**
```python
def button_callback(channel):
    print(f'按键触发！channel={channel}')
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=50)
```

**`test_i2c.py` — I2C 读写测试**
```python
bus = smbus.SMBus(int(input("输入总线号: ")))    # 输入 5
address = int(input("输入设备地址(十进制): "), 16)  # 输入 79
bus.write_byte(address, 0x64)
value = bus.read_byte(address)
```

**`test_serial.py` — 串口回环测试**
```python
port = input("输入串口设备: ")    # 例如: /dev/ttyS1
baud = int(input("输入波特率: "))  # 例如: 115200
ser = serial.Serial(port, baud, timeout=1)
ser.write(b'\xAA\x55')
received = ser.read(2)
```

---

## 第九章：pydev_demo BPU推理示例详解

### 9.1 快速理解BPU推理流程

```
加载模型(.bin)
    ↓ dnn.load('model.bin')
准备输入（NV12格式图像）
    ↓ 摄像头直出 or BGR→NV12转换
BPU前向推理
    ↓ models[0].forward(nv12_data)
后处理（C++加速）
    ↓ libpostprocess.so 中的函数
解析检测结果JSON
    ↓ json.loads(result_str)
```

### 9.2 `probe_model.py` — 最实用的模型诊断工具

**当你拿到一个新的 `.bin` 或 `.hbm` 文件，不知道它的输入尺寸和输出形状时**：

```bash
# 修改脚本第3行的 model_path
python3 /app/pydev_demo/probe_model.py
```

**输出示例**：
```
Model: /app/model/basic/yolov5s_672x672_nv12.bin
Input[0]:
  shape: [1, 3, 672, 672]   ← 输入尺寸
  layout: NCHW
  dtype: uint8
Output[0]:
  shape: [1, 3, 84, 84, 85]  ← YOLOv5s的多尺度输出
  dtype: int32
```

---

## 第十章：常用调试命令速查

```bash
# ========= 环境设置（每个新终端必做）=========
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash

# ========= 查看当前活跃的话题列表 =========
ros2 topic list

# ========= 监听某个话题的实时消息 =========
ros2 topic echo /perception/result_json
ros2 topic echo /gesture/result_json
ros2 topic echo /voice/result_json
ros2 topic echo /puppy_action
ros2 topic echo /asr/text

# ========= 查看话题的发布频率 =========
ros2 topic hz /perception/result_json
ros2 topic hz /ros_robot_controller/imu_raw

# ========= 查看当前所有节点 =========
ros2 node list

# ========= 查看某个节点的参数 =========
ros2 param list /decision_node
ros2 param get /decision_node follow_area_near_stop

# ========= 运行时动态修改参数 =========
ros2 param set /decision_node follow_area_near_stop 0.50
ros2 param set /decision_node turn_gain 0.6

# ========= 手动发消息测试 =========
ros2 topic pub --once /voice/result_json std_msgs/msg/String \
  "{data: '{\"source\":\"voice\",\"command\":\"sit\",\"text\":\"坐下\",\"timestamp\":0}'}"

# ========= 查看节点间通信图 =========
ros2 run rqt_graph rqt_graph

# ========= 编译+重启快捷序列 =========
cd /app/puppy_ws && colcon build --packages-select puppy_brain && \
source install/setup.bash && \
ros2 launch puppy_brain full_system.launch.py
```

---

## 第十一章：扩展开发指南

### 11.1 添加一个全新的动作（比如"趴下"）

**Step 1** — 在 `voice_control_node.py` 添加词条：
```python
ok_new = self.asr.add_words(4, 'pa xia')   # ID=4 → 趴下
```
和映射：
```python
4: {'command': 'lie_down', 'text': '趴下'},
```

**Step 2** — 在 `decision_node.py` 的 `voice_callback()` 里（第215行）扩展：
```python
if command not in ('stand', 'sit', 'stop', 'lie_down'):
    return

# ...

if command == 'lie_down':
    self.follow_enabled = False
    self.voice_lock_until = now + self.voice_action_lock_sec
```

**Step 3** — 底层SDK需要识别 `"lie_down"` 这个字符串（`ros_udp_bridge` 会自动转发）。

### 11.2 切换到更好的检测模型（比如YOLOv8）

**Step 1** — 确认模型文件存在：
```bash
ls /app/model/basic/yolov8_640x640_nv12.bin
```

**Step 2** — 修改 `full_system.launch.py`：
```python
'model_path': '/app/model/basic/yolov8_640x640_nv12.bin',
'input_width': 640,
'input_height': 640,
```

**Step 3**（关键！） — `perception_node.py` 里的后处理函数名也要改：
```python
# 第93行：YOLOv5后处理 → YOLOv8后处理
get_postprocess_result = libpostprocess.Yolov8PostProcess

# 第358行：doProcess 函数名
libpostprocess.Yolov8doProcess(...)
```

### 11.3 添加摄像头可视化到WebSocket预览

`full_system.launch.py` 已配置：
```python
'websocket_image_topic': '/image',                         # 推送的图像
'websocket_smart_topic': '/hobot_mono2d_body_detection'    # 推送的检测
```

浏览器访问 `http://[RDK-IP]:8080` 即可查看。

---

> 文档结束。如有问题，查看第十章的调试命令或回到对应章节查找参数说明。
