# 🐶 PuppyPi 智能四足机器狗系统 — 开源复刻文档

> 基于 RDK X5 + ROS2 Humble 的具身智能四足机器狗，融合双目立体视觉、BPU 端侧推理、离线语音交互与动态步态控制。

---

## 📋 目录

- [一、系统概述](#一系统概述)
- [二、系统架构](#二系统架构)
- [三、目录结构](#三目录结构)
- [四、软硬件要求](#四软硬件要求)
- [五、依赖项](#五依赖项)
- [六、快速开始](#六快速开始)
- [七、启动流程详解](#七启动流程详解)
- [八、算法讲解](#八算法讲解)
- [九、配置参数](#九配置参数)
- [十、交互流程](#十交互流程)
- [十一、代码完整性检查](#十一代码完整性检查)
- [十二、创新点](#十二创新点)

---

## 一、系统概述

### 1.1 这是什么

PuppyPi 是一套跑在 **地平线 RDK X5** 开发板上的四足机器人具身智能控制系统。它把"感知—决策—执行"三层完整闭环跑在端侧，不依赖云端，全部模型与推理在板载 BPU（10 TOPS）上完成。

系统由四大子系统协同：

| 子系统 | 功能 | 关键技术 |
|--------|------|---------|
| **底层运动中枢** | 步态引擎 + 舵机控制 | HiwonderPuppy 动态步态、UDP 指令分发、低通斜坡平滑 |
| **双目视觉感知** | 立体深度 + 人脸/手势识别 | GS130W 双目、StereoNet 深度、TROS 五级 AI 链 |
| **决策仲裁** | 多模态优先级仲裁 | 语音 > 手势 > 视觉跟随，连续速度控制 |
| **离线语音交互** | 语音命令 + TTS 反馈 | Vosk 离线 ASR、WebRTC VAD、Sherpa Matcha TTS |

### 1.2 核心能力

- 🎯 **人物跟随**：YOLOv5 检测人物，连续速度控制前进/转向，带死区防抖与低通滤波
- ✋ **手势控制**：6 种手势映射动作（前进/后退/左转/右转/坐下/趴下）
- 🗣️ **语音控制**：纯离线语音识别 9 类指令 + TTS 语音反馈
- 🚧 **双目避障**：StereoNet 深度图实时分析，前方障碍自动转向/后退，IMU 航向修正防左偏
- 🤖 **LLM 对话**：Qwen2.5-0.5B 端侧大模型，唤醒词触发对话
- 🖥️ **Web 可视化**：浏览器实时查看双目图、深度图、AI 叠加画面

### 1.3 设计哲学

1. **端侧优先**：所有 AI 推理在 BPU 上完成，零云端依赖
2. **模块解耦**：ROS2 话题通信，UDP 转发给闭源底层，互不侵入
3. **平滑控制**：连续速度控制 + 低通滤波，杜绝硬切导致机体不稳
4. **多模态仲裁**：语音 > 手势 > 视觉，优先级锁防冲突
5. **容错设计**：ghost memory 防遮挡急停、手势超时自动刹车、IMU 航向修正

---

## 二、系统架构

### 2.1 整体数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                        硬件输入层                                 │
│   GS130W 双目(MIPI0+MIPI2)    USB 麦克风    I2C 语音模块    IMU  │
└──────────┬──────────────────────────┬──────────────┬───────────┘
           │ NV12 共享内存              │ arecord      │ I2C 0x79   │
           ▼                           ▼              ▼            │
┌──────────────────────┐    ┌──────────────────┐                   │
│ mipi_cam (官方)      │    │ voice_control_   │                   │
│ + hobot_codec        │    │ standalone.py    │                   │
│ → /image_combine_jpeg│    │ VAD+Vosk+模糊匹配│                   │
└──────────┬───────────┘    └────────┬─────────┘                   │
           │                         │ UDP:5005                     │
           ▼                         ▼                              │
┌───────────────────────────────────────────────────┐              │
│              AI 感知链 (TROS 五级)                  │              │
│  mono2d_body → hand_lmk → hand_gesture             │              │
│  face_landmarks → /hobot_face_landmarks            │              │
│  StereoNet → /stereonet_disp (深度图)              │              │
└──────────┬────────────────────────┬────────────────┘              │
           │                        │                               │
           ▼                        ▼                               │
┌──────────────────────┐   ┌────────────────────┐                  │
│ perception_node      │   │ stereo_avoidance_   │                  │
│ YOLOv5 BPU推理       │   │ node (避障)         │                  │
│ → /perception/       │   │ 深度分析+IMU修正    │                  │
│   result_json        │   │ → UDP follow_control│                  │
└──────────┬───────────┘   └─────────┬──────────┘                  │
           │                          │                               │
           ▼                          │                               │
┌──────────────────────┐              │                               │
│ decision_node        │              │                               │
│ 多模态仲裁(最核心)    │              │                               │
│ 语音>手势>视觉跟随    │              │                               │
│ → /puppy_action      │              │                               │
└──────────┬───────────┘              │                               │
           │                          │                               │
           ▼                          ▼                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ros_udp_bridge                                │
│         /puppy_action → UDP 5005                                 │
│         /imu_raw → UDP 5006                                       │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│              sit.py (运动中枢, 右脑)                              │
│   UDP 5005 监听 → 解析动作 → HiwonderPuppy 步态引擎               │
│   → servo_controller → ros_robot_controller_sdk                  │
│   → 串口 /dev/ttyS1 (1Mbps) → STM32 → 8 个 PWM 舵机              │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 四大子系统分层

```
┌─────────────────────────────────────────────────────────────┐
│ 应用层  voice_control_standalone / decision_node / 避障节点    │
├─────────────────────────────────────────────────────────────┤
│ 决策层  多模态仲裁 + 连续速度控制 + 低通滤波                   │
├─────────────────────────────────────────────────────────────┤
│ 感知层  YOLOv5 / mono2d / hand_lmk / hand_gesture / StereoNet  │
├─────────────────────────────────────────────────────────────┤
│ 通信层  ROS2 话题 + UDP 桥 (ros_udp_bridge)                    │
├─────────────────────────────────────────────────────────────┤
│ 运动层  sit.py + HiwonderPuppy 步态引擎 + PWM 舵机控制          │
├─────────────────────────────────────────────────────────────┤
│ 硬件层  RDK X5 BPU + GS130W 双目 + STM32 + 8 舵机 + IMU        │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 ROS2 话题通信总表

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/image_combine_jpeg` | CompressedImage | mipi_cam | websocket/mjpeg_bridge | 主摄像头 JPEG |
| `/sub_image_combine_jpeg` | CompressedImage | mipi_cam | websocket | 子摄像头 JPEG |
| `/image_combine_raw` | Image | mipi_cam | StereoNet | 立体匹配原始图 |
| `/hobot_mono2d_body_detection` | PerceptionTargets | mono2d | hand_lmk/face_lmk | 人体+手+脸检测 |
| `/hobot_hand_lmk_detection` | PerceptionTargets | hand_lmk | hand_gesture | 手部 21 关键点 |
| `/hobot_hand_gesture_detection` | PerceptionTargets | hand_gesture | gesture_adapter | 手势分类 |
| `/StereoNetNode/stereonet_disp` | Image | StereoNet | 避障节点 | 视差图(高=近) |
| `/perception/result_json` | String(JSON) | perception_node | decision_node | YOLO 检测结果 |
| `/gesture/result_json` | String(JSON) | gesture_adapter | decision_node | 手势值 |
| `/puppy_action` | String(JSON) | decision_node | ros_udp_bridge | 控制指令 |
| `/ros_robot_controller/imu_raw` | Imu | imu_node_ros2 | ros_udp_bridge/避障 | IMU 50Hz |
| `/stereo_avoidance/status` | String(JSON) | 避障节点 | (调试) | 避障状态 |

---

## 三、目录结构

```
机器狗代码/
├── start_robot.sh                    # [新增] 底层启动(sit.py+IMU)
├── start_all.sh                      # [新增] 全系统一键启动
│
├── puppy_ws/                         # ROS2 工作空间(决策大脑)
│   ├── src/puppy_brain/              # 核心ROS2包
│   │   ├── puppy_brain/              # 16个Python节点
│   │   │   ├── perception_node.py        # YOLOv5 BPU视觉感知
│   │   │   ├── decision_node.py          # 多模态决策仲裁(最核心)
│   │   │   ├── ros_udp_bridge.py         # ROS→UDP转发桥
│   │   │   ├── gesture_adapter_node.py   # 手势协议适配
│   │   │   ├── gesture_action_node.py    # 手势→动作(X5映射)
│   │   │   ├── usb_cam_publisher_node.py # YUYV摄像头采集
│   │   │   ├── yolov5_mjpeg_server.py     # HTTP MJPEG推流
│   │   │   ├── voice_control_node.py     # I2C语音模块
│   │   │   ├── usb_asr_text_node.py      # USB麦克风Vosk识别
│   │   │   ├── intent_router_node.py     # 语音意图路由
│   │   │   ├── chat_llm_bridge_node.py   # LLM对话桥
│   │   │   ├── tts_play_node.py          # TTS播放
│   │   │   ├── imu_node_ros2.py          # IMU发布
│   │   │   ├── cloud_llm_node.py         # 云端LLM(可选)
│   │   │   ├── ws_bridge_node.py         # WebSocket桥
│   │   │   └── debug_preview_node.py     # 调试可视化
│   │   ├── launch/                   # 6个launch文件
│   │   │   ├── full_system.launch.py     # 完整系统(MIPI摄像头)
│   │   │   ├── usb_gesture.launch.py     # USB摄像头手势链
│   │   │   ├── follow_only.launch.py     # 纯跟随
│   │   │   ├── gesture_only.launch.py    # 纯手势测试
│   │   │   └── minimal_llm.launch.py     # 最小LLM
│   │   ├── config/gesture_map.json   # 手势映射配置
│   │   ├── package.xml               # ROS2包依赖声明
│   │   └── setup.py                  # Python包安装入口
│   ├── config/                       # BPU模型+后处理配置
│   │   ├── multitask_body_head_face_hand_kps_960x544.hbm
│   │   ├── handLMKs.hbm
│   │   ├── gestureDet_8x21.hbm
│   │   ├── gestureDet_32x21.hbm
│   │   └── iou2_method_param.json
│   ├── models/                       # AI模型
│   │   ├── Qwen2.5-0.5B-Instruct-Q4_0.gguf   # LLM
│   │   ├── vosk-model-small-cn-0.22/         # 离线ASR
│   │   └── sherpa_kws/                       # 唤醒词
│   ├── tools/                        # 独立工具脚本
│   │   ├── voice_control_standalone.py  # 纯Python语音控制
│   │   ├── asr_test*.py                 # ASR测试
│   │   ├── sherpa_kws_mic_test.py       # 唤醒词测试
│   │   └── setup_sherpa.sh              # sherpa安装
│   └── docs/PuppyPi_从零上手极详细指南.md
│
├── gs130w_stereo/                    # 双目立体视觉子系统
│   ├── launch/
│   │   ├── gs130w_ai_overlay_v2.launch.py  # v2全AI栈
│   │   ├── gs130w_dualcam.launch.py        # 裸双目
│   │   ├── gs130w_ai_full.launch.py       # 完整AI
│   │   └── camera_info_publisher.py       # 相机标定信息
│   ├── scripts/
│   │   ├── start_v2.sh                # v2启动入口
│   │   ├── start_avoidance.sh         # 避障启动
│   │   ├── stereo_avoidance_node.py   # 双目避障节点
│   │   ├── mjpeg_bridge.py            # MJPEG桥接
│   │   └── stereo_viewer.py           # 立体查看器
│   ├── snapshots/view.html            # Web可视化页面
│   └── README.md
│
├── pydev_demo/                       # 底层运动控制
│   └── puppypi_control/
│       ├── sit.py                    # 运动中枢(UDP监听+动作分发)
│       ├── action_group_control.py   # 动作组执行器
│       ├── servo_controller.py       # 舵机控制封装
│       ├── pwm_servo_control.py      # PWM脉宽控制
│       ├── ros_robot_controller_sdk.py # 串口协议SDK
│       ├── PuppyInstantiate.py       # 步态引擎单例
│       ├── HiwonderPuppy.so          # 步态引擎(闭源)
│       ├── ActionGroups/             # 35个预设动作组(.d6ac)
│       │   ├── sit.d6ac / stand.d6ac / wave.d6ac ...
│       └── 底层运动控制详解.md
│
└── standalone/                       # 独立演示
    ├── gesture_control.py            # 独立手势控制
    └── yolo_display.py               # YOLO可视化
```

---

## 四、软硬件要求

### 4.1 硬件清单

| 部件 | 型号/规格 | 说明 |
|------|----------|------|
| **主控板** | 地平线 RDK X5 | 10 TOPS BPU, ARM Cortex-A55 8核 |
| **双目相机** | GS130W (双 sc132gs) | MIPI0 + MIPI2, 1280×1088 |
| **机器狗本体** | 幻尔 PuppyPi | 8 DOF PWM 舵机, STM32 底板 |
| **USB 麦克风** | 任意 USB 音频设备 | plughw:1,0 |
| **USB 音响** | 任意 USB 音频设备 | plughw:0,0 |
| **I2C 语音模块** | ASR 识别模块 (可选) | I2C 0x79 @ bus5 |
| **TF 卡** | ≥64GB U1 | 存模型与系统 |
| **电源** | 7.4V 锂电池 ≥2000mAh | 舵机供电 |

### 4.2 软件环境

| 组件 | 版本 | 说明 |
|------|------|------|
| OS | Ubuntu 22.04 (Jammy) aarch64 | RDK X5 官方镜像 |
| ROS2 | Humble (TROS 2.5.x) | `/opt/tros/humble/` |
| Python | 3.10 | 系统自带 |
| OpenCV | 4.x | `pip3 install opencv-python` |
| numpy | ≥1.20 | 图像处理 |
| vosk | 0.3.45 | 离线语音识别 |
| webrtcvad | 2.0.10 | 语音活动检测 |
| sherpa-onnx | ≥1.10 | TTS/KWS |
| soundfile | ≥0.12 | 音频写入 |

### 4.3 关键系统路径（板端）

| 路径 | 说明 |
|------|------|
| `/opt/tros/humble/setup.bash` | TROS 环境源 |
| `/app/puppy_ws/` | ROS2 工作空间 |
| `/app/gs130w_stereo/` | 双目视觉工程 |
| `/app/pydev_demo/puppypi_control/` | 底层运动控制 |
| `/app/model/basic/yolov5s_672x672_nv12.bin` | YOLOv5 BPU 模型 |
| `/usr/lib/libpostprocess.so` | YOLO 后处理 C++ 库 |
| `/dev/ttyS1` | 串口(连 STM32, 1Mbps) |
| `/dev/i2c-5` | I2C 总线(语音模块) |

---

## 五、依赖项

### 5.1 ROS2 包依赖 (package.xml)

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>ai_msgs</depend>
```

### 5.2 TROS 官方节点依赖

| 包 | 用途 |
|----|------|
| `mipi_cam` | MIPI 摄像头驱动 |
| `hobot_codec` | 图像格式转换 (NV12↔JPEG) |
| `mono2d_body_detection` | 人体/手/脸检测 |
| `hand_lmk_detection` | 手部关键点 |
| `hand_gesture_detection` | 手势分类 |
| `face_landmarks_detection` | 人脸关键点(可选) |
| `hobot_stereonet` | 双目立体匹配 |
| `hobot_llamacpp` | LLM 推理 (Qwen) |
| `websocket` | Web 可视化推流 |
| `ros_robot_controller_sdk` | 幻尔 SDK (Board 类) |

### 5.3 Python 依赖安装

```bash
# 视觉/数值
pip3 install opencv-python numpy

# 语音识别
pip3 install vosk webrtcvad soundfile

# TTS (sherpa, 推荐)
bash /app/puppy_ws/tools/setup_sherpa.sh
pip3 install sherpa-onnx

# 系统工具
sudo apt install -y sox alsa-utils netcat-openbsd
```

### 5.4 BPU 模型文件

| 模型 | 路径 | 用途 |
|------|------|------|
| YOLOv5s | `/app/model/basic/yolov5s_672x672_nv12.bin` | 目标检测 |
| 多任务检测 | `TROS/lib/mono2d_body_detection/config/multitask_...960x544.hbm` | 人体+手+脸 |
| 手部关键点 | `TROS/lib/hand_lmk_detection/config/handLMKs.hbm` | 21 关键点 |
| 手势分类 | `TROS/lib/hand_gesture_detection/config/gestureDet_8x21.hbm` | 静态手势 |
| StereoNet | `TROS/share/hobot_stereonet/config/DStereoV2.0.bin` | 双目深度 |
| Qwen LLM | `/app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf` | 对话 |
| Vosk ASR | `/app/puppy_ws/models/vosk-model-small-cn-0.22/` | 中文识别 |

---

## 六、快速开始

### 6.1 首次部署（5 步）

```bash
# ① 拷贝代码到板端 /app
scp -r 机器狗代码/* root@<板端IP>:/app/

# ② 安装 Python 依赖
pip3 install opencv-python numpy vosk webrtcvad soundfile
bash /app/puppy_ws/tools/setup_sherpa.sh

# ③ 编译 ROS2 包
cd /app/puppy_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select puppy_brain
source install/setup.bash

# ④ 确认硬件连接（双目排线、USB麦克风、电池）
ls /dev/video*         # 摄像头
arecord -l             # 麦克风
i2cdetect -y 5         # I2C 语音模块(可选)

# ⑤ 一键启动全系统
bash /app/start_all.sh start
```

### 6.2 验证系统

启动后浏览器访问：

| 地址 | 内容 |
|------|------|
| `http://<板端IP>:8090/view.html` | 双目 Web 总入口 |
| `http://<板端IP>:8071` | 右眼实时图 |
| `http://<板端IP>:8072` | 左眼实时图 |
| `http://<板端IP>:8073` | 深度图(彩色) |

### 6.3 一键停止

```bash
bash /app/start_all.sh stop
```

---

## 七、启动流程详解

### 7.1 四链启动顺序（关键！）

系统由四条独立链路组成，**必须按依赖顺序启动**：

```
链1: 底层运动中枢          链2: 双目视觉            链3: 避障              链4: 语音
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐
│ start_robot.sh   │    │ start_v2.sh     │    │ start_avoidance │    │ voice_       │
│   start          │    │   start         │    │   .sh start     │    │ standalone   │
│                  │    │                 │    │                 │    │   .py        │
│ ↓ sit.py         │    │ ↓ mipi_cam      │    │ ↓ 订阅深度+IMU   │    │ ↓ VAD+Vosk  │
│   (UDP 5005)     │    │ ↓ 5级AI链       │    │   分析前方障碍   │    │   9类指令    │
│ ↓ imu_node_ros2  │    │ ↓ StereoNet     │    │   UDP避障指令    │    │   UDP:5005  │
│   (50Hz IMU)     │    │ ↓ Web可视化     │    │                 │    │   +TTS反馈   │
└────────┬─────────┘    └────────┬────────┘    └────────┬────────┘    └──────┬──────┘
         │                       │                      │                    │
         └───────── UDP 5005 (sit.py 运动中枢) ─────────┴────────────────────┘
```

### 7.2 各链路详细启动命令

#### 链1：底层运动中枢

```bash
/app/start_robot.sh start
```

启动内容：
1. `sit.py` — 运动中枢，监听 UDP 5005，解析动作指令调用步态引擎
2. `imu_node_ros2` — 50Hz 读取 IMU，发布 `/ros_robot_controller/imu_raw`

#### 链2：双目视觉感知

```bash
/app/gs130w_stereo/scripts/start_v2.sh start
```

启动内容（7 个进程）：
1. `mipi_cam` — 官方双目驱动（含 codec + websocket）
2. `camera_info_publisher` — 相机标定信息
3. `gs130w_ai_overlay_v2.launch.py` — 5 模型 AI 链 + 3 个 WebSocket
4. `mjpeg_bridge` ×3 — 左眼/右眼/深度图 MJPEG 推流
5. `http.server` — Web 页面服务 (8090)

#### 链3：双目深度避障

```bash
/app/gs130w_stereo/scripts/start_avoidance.sh start
```

**前置**：链1 + 链2 必须先启动。启动 `stereo_avoidance_node.py`，订阅 StereoNet 深度图 + IMU，10Hz 决策，发 `follow_control` UDP 指令。

#### 链4：纯 Python 语音控制

```bash
python3 /app/puppy_ws/tools/voice_control_standalone.py \
    --mic plughw:1,0 \
    --speaker plughw:0,0 \
    --gain 10 \
    --aggressiveness 2 \
    --silence 1.0
```

**前置**：链1 必须先启动（sit.py 监听 UDP 5005）。

参数说明：
| 参数 | 值 | 说明 |
|------|-----|------|
| `--mic` | `plughw:1,0` | 麦克风设备（`arecord -l` 查） |
| `--speaker` | `plughw:0,0` | 音响设备（`aplay -l` 查） |
| `--gain` | `10` | 增益 dB（环境嘈杂可加大） |
| `--aggressiveness` | `2` | VAD 灵敏度 0-3（3 最激进） |
| `--silence` | `1.0` | 静音断句秒数 |

### 7.3 一键启动（推荐）

```bash
# 全量启动（4 条链全部）
bash /app/start_all.sh start

# 启动但跳过语音
bash /app/start_all.sh start no_voice

# 启动但跳过避障（切回 LLM/手势控制模式）
bash /app/start_all.sh start no_avoidance

# 停止全部
bash /app/start_all.sh stop

# 查看状态
bash /app/start_all.sh status
```

### 7.4 替代启动模式（ROS2 full_system）

若使用 MIPI F37 单目摄像头（非双目），可用 ROS2 完整系统：

```bash
cd /app/puppy_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
# 终端1: 底层
bash /app/start_robot.sh start
# 终端2: ROS2 完整系统（含 LLM 对话）
ros2 launch puppy_brain full_system.launch.py
```

> ⚠️ `full_system.launch.py` 与 `start_avoidance.sh` **不能同时运行**，两者都发 `/puppy_action` 会冲突。

---

## 八、算法讲解

### 8.1 视觉跟随算法（decision_node 核心）

#### 算法流程

```
检测到人 → 选最大 person → 算中心偏差 → 算面积占比
                                         ↓
                            ┌────────────┴────────────┐
                            ▼                          ▼
                      计算转向 turn             计算前进 forward
                      (死区+线性映射)          (面积分区+平方曲线)
                            └────────────┬────────────┘
                                         ▼
                              误差分级调整 forward
                              (小/中/大误差不同保持比)
                                         ▼
                              低通滤波平滑 (alpha=0.28)
                                         ▼
                              发布 follow_control
```

#### 关键公式

**转向计算**：
```
error = cx_ratio - center_ratio        # 中心偏差
if |error| < turn_deadband(0.09):     # 死区
    turn = 0
else:
    norm_error = (|error| - deadband) / max_turn_error(0.28)
    turn = ±turn_gain(0.85) × clamp(norm_error, 0, 1)
```

**前进计算**（面积分区）：
```
if area_ratio >= near_stop(0.42):      # 太近，停
    forward = 0
elif area_ratio <= far_walk(0.10):    # 太远，全速
    forward = forward_max(0.95)
else:                                  # 中间，平方曲线
    ratio = (near_stop - area) / (near_stop - far_walk)
    forward = min + (max - min) × ratio²   # 平方！靠近更灵敏
```

**低通滤波**：
```
smoothed = (1 - α) × last + α × target    # α=0.28
# 28% 新值 + 72% 旧值，防止速度突变
```

### 8.2 双目深度避障算法

#### 数据源选择（自动检测）

```
优先级 1: /stereonet_disp (视差图, 高值=近)     ← 有则用
优先级 2: /stereonet_visual (彩色, 红=近蓝=远)  ← 退化方案
```

#### 三区分析

```
深度图 → 180°翻转修复(修正 mipi_rotation) → 取中间垂直带(40%~80%)
                                              ↓
                                    ┌─────────┴─────────┐
                                    ▼     ▼     ▼
                                  左35%  中30%  右35%
                                  取P90  取P90  取P90
```

#### 决策逻辑（follow_control 流畅模式）

```
if center > danger(30):        # 前方太近
    if left < right - 2:       # 左空 → 左转
        turn_left (锁定 0.4s)
    elif right < left - 2:     # 右空 → 右转
        turn_right (锁定 0.4s)
    else:                      # 两侧堵 → 后退
        backward
elif center > clear(15):       # 接近障碍
    slow_forward + IMU修正×0.5
else:                          # 路径畅通
    normal_forward + IMU修正
```

#### IMU 航向修正（解决前进左偏）

```
# 进入前进时记录目标航向
target_yaw = integrated_yaw

# 每帧修正
yaw_error = integrated_yaw - target_yaw    # 正=左偏
correction = -yaw_gain(0.6) × yaw_error    # 左偏→右转修正
correction = clamp(correction, ±0.25)       # 限幅
turn += correction
```

### 8.3 手势识别链（TROS 五级）

```
图像 → mono2d_body_detection (多任务: 人体+手+脸)
              ↓
        hand_lmk_detection (手部 21 关键点回归)
              ↓
        hand_gesture_detection (手势分类, 8 类)
              ↓
        gesture_action_node (X5 映射 → 动作)
```

#### X5 手势映射表

| gesture_value | 手势名 | 动作 | 类型 |
|--------------|--------|------|------|
| 5.0 | palm (手掌) | forward 前进 | 移动 |
| 2.0 | thumb_up (点赞) | sit 坐下 | 离散 |
| 3.0 | victory (V) | crouch 趴下 | 离散 |
| 11.0 | okay (OK) | backward 后退 | 移动 |
| 12.0 | thumb_left | turn_left 左转 | 移动 |
| 双 palm | 双手张开 | turn_right 右转 | 移动 |

### 8.4 语音控制算法

```
arecord 流式录音 → WebRTC VAD 自动断句
                      ↓
              静音 1.0s → 截断
                      ↓
              sox 增益 10dB (+降噪)
                      ↓
              Vosk 离线识别 → 文本
                      ↓
              模糊匹配 (包含+相似度 0.6 阈值)
                      ↓
              UDP 发动作 + Sherpa TTS 反馈
```

#### 9 类语音指令

| 动作 | 触发词（部分） | TTS 反馈 |
|------|--------------|---------|
| forward | 前进/向前走/往前走/直走 | "好的，前进" |
| backward | 后退/倒车/向后 | "好的，后退" |
| turn_left | 左转/向左转/左拐 | "好的，左转" |
| turn_right | 右转/向右转/右拐 | "好的，右转" |
| stand | 站起来/站立/起立 | "好的，站起来" |
| sit | 坐下/坐下来/请坐 | "好的，坐下" |
| crouch | 趴下/蹲下/卧倒 | "好的，趴下" |
| wave | 摇摆/招手/挥手 | "好的，摇摆" |
| stop | 停下/停止/别动 | "好的，停止" |

---

## 九、配置参数

### 9.1 决策节点核心参数（full_system.launch.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_area_near_stop` | 0.42 | 面积占比≥此值刹车(约1米内) |
| `follow_area_far_walk` | 0.10 | 面积占比≤此值全速(约4-5米) |
| `turn_deadband_ratio` | 0.09 | 中心偏差死区(防抖) |
| `turn_gain` | 0.85 | 转向增益(大=猛) |
| `forward_max` | 0.95 | 最大前进速度 |
| `control_smooth_alpha` | 0.28 | 低通滤波系数(大=响应快) |
| `ghost_memory_time` | 0.30 | 目标消失惯性时间(秒) |
| `voice_action_lock_sec` | 2.5 | 语音指令锁定时间 |
| `gesture_action_lock_sec` | 2.5 | 手势动作锁定时间 |

### 9.2 避障节点参数（stereo_avoidance_node.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `danger_disp` | 30.0 | 视差>此值=障碍太近(~0.87m) |
| `clear_disp` | 15.0 | 视差<此值=畅通(~1.73m) |
| `decision_hz` | 10.0 | 决策频率 |
| `use_follow_control` | True | 流畅模式(推荐) |
| `use_imu_correction` | True | IMU 航向修正 |
| `yaw_gain` | 0.6 | 航向修正增益 |
| `turn_lock_sec` | 0.4 | 转向锁定时长(防抖) |

### 9.3 语音控制参数（voice_control_standalone.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--mic` | plughw:1,0 | 麦克风设备 |
| `--speaker` | plughw:0,0 | 音响设备 |
| `--gain` | 10 | 增益 dB |
| `--aggressiveness` | 2 | VAD 灵敏度(0-3) |
| `--silence` | 1.0 | 静音断句秒数 |
| `--threshold` | 0.6 | 模糊匹配阈值 |

### 9.4 运行时动态调参

```bash
# 不重启修改刹车距离
ros2 param set /decision_node follow_area_near_stop 0.50

# 修改转向增益
ros2 param set /decision_node turn_gain 0.6

# 关闭 IMU 修正
ros2 param set /stereo_avoidance use_imu_correction false
```

---

## 十、交互流程

### 10.1 场景一：语音控制机器人

```
用户说话 "前进"
    ↓
voice_control_standalone: VAD 检测语音 → 录音 → Vosk 识别 "前进"
    ↓
模糊匹配: "前进" ∈ forward.words → 命中 forward
    ↓
UDP 发送 "forward" → 127.0.0.1:5005
    ↓
sit.py 收到 "forward" → set_motion_target(WALK_X=10, 0, 0)
    ↓
平滑控制线程: 50ms 斜坡逼近 → puppy.move(10, 0, 0)
    ↓
Sherpa TTS: "好的，前进" → aplay 播放
    ↓
机器人前进（平滑加速）
```

### 10.2 场景二：手势控制机器人

```
用户比 "手掌" (palm, gesture_value=5.0)
    ↓
mipi_cam → mono2d → hand_lmk → hand_gesture → /hobot_hand_gesture_detection
    ↓
gesture_action_node: palm_count=1 → start_move('forward')
    ↓
发布 /puppy_action: {"mode":"follow_control","forward":0.55,"turn":0}
    ↓
ros_udp_bridge → UDP 5005
    ↓
sit.py: follow_control → set_motion_target(0.55×10=5.5, 0, 0)
    ↓
机器人前进（持续，手势消失 0.4s 后自动刹车）
```

### 10.3 场景三：双目自动避障

```
机器人前进中
    ↓
StereoNet 输出视差图 → 避障节点分析
    ↓
左=15 中=35 右=12  (center > danger=30, 左侧更空)
    ↓
决策: 左转 (锁定 0.4s)
    ↓
UDP: {"mode":"follow_control","forward":0,"turn":0.5}
    ↓
sit.py: set_motion_target(0, 0, 0.5×0.7=0.35) → puppy.move(0,0,0.35)
    ↓
机器人左转避障
```

### 10.4 场景四：人物跟随

```
perception_node: YOLOv5 检测到 person, bbox=[120,80,450,520]
    ↓
发布 /perception/result_json
    ↓
decision_node:
  area_ratio = (330×440)/(960×544) = 0.277
  cx_ratio = 285/960 = 0.297
  error = 0.297 - 0.50 = -0.203 (偏左)
  turn = +0.85 × 0.40 = +0.34 (左转追)
  forward = 0.95 × 0.78 (中误差) = 0.74
  smooth: forward=0.21×0.74+0.79×last, turn=...
    ↓
发布 follow_control: {forward:0.58, turn:0.24}
    ↓
机器人边转边追人
```

---

## 十一、代码完整性检查

### 11.1 已包含的核心代码 ✅

| 模块 | 路径 | 状态 |
|------|------|------|
| ROS2 决策大脑 (16 节点) | `puppy_ws/src/puppy_brain/` | ✅ 完整 |
| 6 个 launch 文件 | `puppy_ws/src/puppy_brain/launch/` | ✅ 完整 |
| BPU 模型配置 | `puppy_ws/config/*.hbm` | ✅ 完整 |
| AI 模型 (LLM/ASR/KWS) | `puppy_ws/models/` | ✅ 完整 |
| 语音控制工具 | `puppy_ws/tools/` | ✅ 完整 |
| 双目视觉子系统 | `gs130w_stereo/` | ✅ 完整 |
| 底层运动控制 | `pydev_demo/puppypi_control/` | ✅ 完整 |
| 35 个动作组 | `pydev_demo/.../ActionGroups/` | ✅ 完整 |
| 步态引擎 (.so) | `HiwonderPuppy.so` | ✅ 包含 |
| 详细文档 ×3 | docs/ | ✅ 完整 |

### 11.2 缺失/需补充项 ⚠️

| 项目 | 说明 | 处理 |
|------|------|------|
| `start_robot.sh` | 板端 `/app/start_robot.sh` 未含在源码 | **已创建**（见本目录） |
| `.gitignore` | 缺少，build/install/log/puppy_env 应排除 | 建议添加 |
| `package.xml` license | `TODO: License` | 开源前需声明 License |
| `libpostprocess.so` | 系统库 `/usr/lib/`，非源码 | 板端自带，无需包含 |
| `puppy_env/` 虚拟环境 | 含 numpy 等依赖，体积大 | **建议删除**，用 requirements.txt 替代 |
| `build/ install/ log/` | colcon 编译产物 | **建议删除**，用户自行编译 |
| `__pycache__/ *.pyc` | Python 缓存 | **建议清理** |
| `*.bak` 备份文件 | decision_node.py.bak 等 | **建议删除** |

### 11.3 开源前清理建议

```bash
# 在板端执行，清理编译产物与缓存
cd /app
rm -rf puppy_ws/build puppy_ws/install puppy_ws/log
rm -rf pydev_demo/puppypi_control/puppy_env
find . -type d -name __pycache__ -exec rm -rf {} +
find . -name "*.pyc" -delete
find . -name "*.bak" -delete

# 创建 .gitignore
cat > .gitignore << 'EOF'
__pycache__/
*.pyc
*.pyo
puppy_ws/build/
puppy_ws/install/
puppy_ws/log/
*.egg-info/
.DS_Store
*.log
EOF
```

---

## 十二、创新点

### 12.1 端侧具身智能闭环

全链路"感知—决策—执行"在单块 RDK X5 上完成，零云端依赖：
- BPU 10 TOPS 同时跑 5 个模型（YOLOv5 + mono2d + hand_lmk + hand_gesture + StereoNet）
- IMU 50Hz + 视觉 10Hz + 决策 10Hz 多模态融合

### 12.2 双轨控制协议设计

创新的 `follow_control` 连续控制 + 离散动作双协议：
- **连续模式**：归一化 `forward/turn ∈ [-1,1]`，底层斜坡平滑，流畅无卡顿
- **离散模式**：`sit/stand/wave` 等动作组，调用 `.d6ac` 预录轨迹
- sit.py 统一仲裁，兼容新旧协议

### 12.3 多模态优先级仲裁

`decision_node` 实现严格优先级：**语音 > 手势 > 视觉跟随**
- 锁机制防止低优先级覆盖高优先级指令
- ghost memory 防遮挡急停（0.3s 惯性减速）
- 手势超时自动刹车（防误识别持续运动）

### 12.4 双目深度避障 + IMU 航向修正

- StereoNet 视差图三区分析（左/中/右 P90）
- 180° 翻转修复（mipi_rotation 导致的左右颠倒）
- IMU 角速度积分航向，比例控制修正前进左偏
- 转向锁定 0.4s 防止障碍边界左右抖动

### 12.5 YUYV 摄像头兼容方案

官方 `hobot_usb_cam` 不支持 YUYV-only 摄像头，自研 `usb_cam_publisher_node`：
- OpenCV 采集 YUYV → 编码 JPEG → 发布 `/image_raw/compressed`
- hobot_codec 订阅转 NV12 shared_mem，无缝接入 TROS AI 链

### 12.6 纯 Python 离线语音控制

`voice_control_standalone.py` 不依赖 ROS2：
- WebRTC VAD 自动断句 + Vosk 离线识别 + 模糊匹配（包含+相似度）
- Sherpa Matcha TTS 离线语音反馈
- TTS 播放时天然不录音（同步阻塞防自激）

---

## 附录：常用调试命令

```bash
# 环境（每个新终端必做）
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash

# 查看 ROS2 话题
ros2 topic list
ros2 topic echo /puppy_action
ros2 topic hz /perception/result_json

# 手动发指令测试
ros2 topic pub --once /puppy_action std_msgs/msg/String \
  '{data: "{\"action\": \"sit\", \"source\": \"manual\"}"}'

# UDP 直接发指令
echo '{"action":"walk","source":"test"}' | nc -u -w1 127.0.0.1 5005

# 运行时调参
ros2 param set /decision_node turn_gain 0.6

# 查看节点图
ros2 run rqt_graph rqt_graph
```

---

> 📖 更详细的节点级文档请参阅 `puppy_ws/docs/PuppyPi_从零上手极详细指南.md` 和 `pydev_demo/puppypi_control/底层运动控制详解.md`

> 文档版本：2026-07-09 | 基于 RDK X5 + TROS 2.5.x
