# 🐶 PuppyPi 智能四足机器狗系统

> 基于 RDK X5 + ROS2 Humble 的端侧具身智能四足机器狗，融合双目立体视觉、BPU 端侧推理、离线语音交互与动态步态控制。全链路「感知—决策—执行」在单块板子上完成，零云端依赖。

> 📖 **English version**: [README_EN.md](./README_EN.md)
> 📘 **详细中文文档**: [PuppyPi_开源文档.md](./PuppyPi_开源文档.md)

---

## ✨ 核心特性

| 能力 | 说明 |
|------|------|
| 🎯 人物跟随 | YOLOv5 检测 + 连续速度控制 + 低通滤波平滑 |
| ✋ 手势控制 | 6 种手势映射动作（前进/后退/左转/右转/坐下/趴下） |
| 🗣️ 离线语音控制 | Vosk 中文 ASR + WebRTC VAD + Sherpa TTS，9 类指令 |
| 🚧 双目避障 | StereoNet 深度图三区分析 + IMU 航向修正 |
| 🤖 端侧 LLM | Qwen2.5-0.5B 对话（可选） |
| 🖥️ Web 可视化 | 浏览器实时查看双目图/深度图/AI 叠加画面 |

---

## 🏗️ 系统架构

```
┌─────────────── 硬件输入 ───────────────────┐
│ GS130W 双目  USB 麦克风  I2C 语音模块  IMU │
└──────┬─────────────┬──────────────┬────────┘
       ▼             ▼              ▼
┌──────────────┐  ┌────────────────────────┐
│ mipi_cam     │  │ voice_control_          │
│ + 5级AI链    │  │ standalone.py           │
│ + StereoNet  │  │ VAD+Vosk+模糊匹配        │
└──────┬───────┘  └───────────┬────────────┘
       ▼                      │
┌──────────────────────────┐  │ UDP:5005
│ perception_node (YOLOv5) │  │
│ decision_node (多模态仲裁) │◄─┤ 语音>手势>视觉跟随
│ stereo_avoidance (避障)   │  │
└──────────┬───────────────┘  │
           ▼                  │
┌──────────────────────────────┴─────────────┐
│ ros_udp_bridge  /puppy_action → UDP 5005    │
└──────────────────────┬─────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ sit.py (运动中枢) → HiwonderPuppy 步态引擎    │
│ → servo_controller → /dev/ttyS1 → STM32 → 8舵机 │
└─────────────────────────────────────────────┘
```

---

## 📁 目录结构

```
机器狗代码/
├── start_robot.sh                 # 底层启动 (sit.py + IMU)
├── start_all.sh                   # 全系统一键启动 (四链整合)
├── README.md                      # 中文说明 (本文件)
├── README_EN.md                   # English README
├── PuppyPi_开源文档.md             # 详细中文文档 (12 章)
│
├── puppy_ws/                      # ROS2 工作空间 (决策大脑)
│   ├── src/puppy_brain/           # 核心包: 16 个 Python 节点 + 6 个 launch
│   ├── config/                    # BPU 模型配置 (*.hbm)
│   ├── models/                    # AI 模型 (LLM/ASR/KWS)
│   ├── tools/                     # 独立工具 (语音控制等)
│   └── docs/                      # 详细使用指南
│
├── gs130w_stereo/                 # 双目立体视觉子系统
│   ├── launch/                    # 双目 launch + 相机标定
│   ├── scripts/                   # 启动脚本 + 避障节点 + MJPEG 桥
│   └── snapshots/view.html        # Web 可视化
│
├── pydev_demo/puppypi_control/   # 底层运动控制
│   ├── sit.py                     # 运动中枢 (UDP 5005 监听)
│   ├── HiwonderPuppy.so           # 步态引擎 (闭源)
│   ├── ActionGroups/              # 35 个预设动作组 (.d6ac)
│   └── 底层运动控制详解.md
│
└── standalone/                    # 独立演示
    ├── gesture_control.py
    └── yolo_display.py
```

---

## 🔧 软硬件要求

### 硬件

| 部件 | 规格 |
|------|------|
| 主控板 | 地平线 RDK X5 (10 TOPS BPU, Cortex-A55 8 核) |
| 双目相机 | GS130W (双 sc132gs, MIPI0+MIPI2, 1280×1088) |
| 机器狗本体 | 幻尔 PuppyPi (8 DOF PWM 舵机 + STM32 底板) |
| USB 麦克风/音响 | 任意 USB 音频设备 |
| TF 卡 | ≥64GB U1 |

### 软件

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 22.04 (Jammy) aarch64 |
| ROS2 | Humble (TROS 2.5.x) |
| Python | 3.10 |
| OpenCV | 4.x |
| vosk | 0.3.45 |
| webrtcvad | 2.0.10 |
| sherpa-onnx | ≥1.10 |

---

## 🚀 快速开始

### 1. 部署代码到板端

```bash
scp -r 机器狗代码/* root@<板端IP>:/app/
```

### 2. 安装依赖

```bash
pip3 install opencv-python numpy vosk webrtcvad soundfile
bash /app/puppy_ws/tools/setup_sherpa.sh
sudo apt install -y sox alsa-utils netcat-openbsd
```

### 3. 编译 ROS2 包

```bash
cd /app/puppy_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select puppy_brain
source install/setup.bash
```

### 4. 一键启动全系统

```bash
bash /app/start_all.sh start
```

### 5. 验证

浏览器访问 `http://<板端IP>:8090/view.html` 查看双目 Web 总入口。

### 停止

```bash
bash /app/start_all.sh stop
```

---

## 🎛 启动流程

系统由四条链路组成，**必须按依赖顺序启动**（`start_all.sh` 已自动处理）：

| 顺序 | 命令 | 作用 |
|------|------|------|
| ① 底层运动 | `/app/start_robot.sh start` | sit.py 运动中枢 (UDP 5005) + IMU 50Hz |
| ② 双目视觉 | `/app/gs130w_stereo/scripts/start_v2.sh start` | GS130W + 5 级 AI 链 + StereoNet + Web |
| ③ 双目避障 | `/app/gs130w_stereo/scripts/start_avoidance.sh start` | 深度分析 + IMU 修正 → UDP |
| ④ 语音控制 | `python3 /app/puppy_ws/tools/voice_control_standalone.py --mic plughw:1,0 --speaker plughw:0,0 --gain 10 --aggressiveness 2 --silence 1.0` | VAD + Vosk + TTS |

```bash
# 全量启动
bash /app/start_all.sh start

# 跳过语音
bash /app/start_all.sh start no_voice

# 跳过避障 (切回 LLM/手势控制)
bash /app/start_all.sh start no_avoidance

# 状态 / 停止 / 重启
bash /app/start_all.sh status
bash /app/start_all.sh stop
bash /app/start_all.sh restart
```

> ⚠️ `full_system.launch.py` 与 `start_avoidance.sh` 不能同时运行，两者都发 `/puppy_action` 会冲突。

---

## ⚙️ 关键配置参数

### 决策节点 (decision_node)

| 参数 | 默认 | 说明 |
|------|------|------|
| `follow_area_near_stop` | 0.42 | 面积占比≥此值刹车 |
| `follow_area_far_walk` | 0.10 | 面积占比≤此值全速 |
| `turn_deadband_ratio` | 0.09 | 转向死区 (防抖) |
| `turn_gain` | 0.85 | 转向增益 |
| `control_smooth_alpha` | 0.28 | 低通滤波系数 |
| `ghost_memory_time` | 0.30 | 目标消失惯性 (秒) |

### 避障节点 (stereo_avoidance_node)

| 参数 | 默认 | 说明 |
|------|------|------|
| `danger_disp` | 30.0 | 视差>此值=太近 |
| `clear_disp` | 15.0 | 视差<此值=畅通 |
| `use_imu_correction` | True | IMU 航向修正 |
| `yaw_gain` | 0.6 | 航向修正增益 |

运行时动态调参：

```bash
ros2 param set /decision_node turn_gain 0.6
```

---

## ✋ X5 手势映射

| 手势 | gesture_value | 动作 |
|------|--------------|------|
| 手掌 palm | 5.0 | 前进 |
| 点赞 thumb_up | 2.0 | 坐下 |
| V (victory) | 3.0 | 趴下 |
| OK (okay) | 11.0 | 后退 |
| 左指 thumb_left | 12.0 | 左转 |
| 双手张开 | 双 palm | 右转 |

---

## 🗣️ 语音指令

| 动作 | 触发词示例 |
|------|-----------|
| forward | 前进 / 向前走 / 直走 |
| backward | 后退 / 倒车 |
| turn_left | 左转 / 左拐 |
| turn_right | 右转 / 右拐 |
| stand | 站起来 / 起立 |
| sit | 坐下 / 请坐 |
| crouch | 趴下 / 蹲下 |
| stop | 停下 / 别动 |

---

## 💡 创新点

1. **端侧具身智能闭环** — 5 个模型同时跑在 BPU 10 TOPS，零云端依赖
2. **双轨控制协议** — `follow_control` 连续速度 + 离散动作组双协议，底层斜坡平滑
3. **多模态优先级仲裁** — 语音 > 手势 > 视觉跟随，锁机制防冲突 + ghost memory 防遮挡急停
4. **双目避障 + IMU 航向修正** — StereoNet 视差三区分析，角速度积分修正前进左偏
5. **YUYV 摄像头兼容方案** — 自研 `usb_cam_publisher_node` 替代不支持 YUYV 的官方 `hobot_usb_cam`
6. **纯 Python 离线语音** — 不依赖 ROS2，VAD 自动断句 + 模糊匹配 + TTS 同步防自激

---

## 📚 详细文档

- [PuppyPi_开源文档.md](./PuppyPi_开源文档.md) — 12 章完整中文文档 (系统/架构/算法/参数/交互流程/创新点)
- [puppy_ws/docs/PuppyPi_从零上手极详细指南.md](./puppy_ws/docs/PuppyPi_从零上手极详细指南.md) — 节点级上手指南
- [pydev_demo/puppypi_control/底层运动控制详解.md](./pydev_demo/puppypi_control/底层运动控制详解.md) — 底层运动技术参考

---

## 🧪 常用调试

```bash
# 环境源 (每个新终端)
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash

# 查看话题
ros2 topic list
ros2 topic echo /puppy_action
ros2 topic hz /perception/result_json

# UDP 直接发指令
echo '{"action":"walk","source":"test"}' | nc -u -w1 127.0.0.1 5005
```

---

## ⚠️ 开源前清理建议

```bash
# 在板端执行，清理编译产物与缓存
cd /app
rm -rf puppy_ws/build puppy_ws/install puppy_ws/log
rm -rf pydev_demo/puppypi_control/puppy_env
find . -type d -name __pycache__ -exec rm -rf {} +
find . -name "*.pyc" -delete
```

---

## 📄 License

本项目代码采用 **MIT License** 开源。
`HiwonderPuppy.so` 步态引擎为幻尔科技闭源库，仅提供二进制，版权归原作者所有。
BPU 模型文件 (`.hbm`/`.bin`) 配置参考地平线 TROS 官方文档。

---

> 文档版本：2026-07-09 | 基于 RDK X5 + TROS 2.5.x
