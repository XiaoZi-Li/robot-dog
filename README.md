# 🐶 PuppyPi Smart Quadruped Robot System

> An on-device embodied-intelligence quadruped robot built on RDK X5 + ROS2 Humble, fusing binocular stereo vision, BPU on-device inference, offline voice interaction and dynamic gait control. The full perception→decision→action loop runs on a single board with **zero cloud dependency**.

> 📖 **中文版**: [README.md](./README.md)
> 📘 **Full Chinese docs**: [PuppyPi_开源文档.md](./PuppyPi_开源文档.md)

---

## ✨ Key Features

| Ability | Description |
|---------|-------------|
| 🎯 Person Following | YOLOv5 detection + continuous velocity control + low-pass filter smoothing |
| ✋ Gesture Control | 6 gestures mapped to actions (forward/back/turn left/right/sit/crouch) |
| 🗣️ Offline Voice | Vosk Chinese ASR + WebRTC VAD + Sherpa TTS, 9 command classes |
| 🚧 Stereo Obstacle Avoidance | StereoNet depth map 3-zone analysis + IMU heading correction |
| 🤖 On-device LLM | Qwen2.5-0.5B dialogue (optional) |
| 🖥️ Web Visualization | Live binocular/depth/AI-overlay in browser |

---

## 🏗️ System Architecture

```
┌─────────────── Hardware Input ──────────────┐
│ GS130W stereo  USB mic  I2C voice  IMU      │
└──────┬──────────────┬───────────────┬───────┘
       ▼              ▼               ▼
┌──────────────┐  ┌────────────────────────┐
│ mipi_cam     │  │ voice_control_          │
│ + 5-stage AI │  │ standalone.py           │
│ + StereoNet  │  │ VAD+Vosk+fuzzy match    │
└──────┬───────┘  └───────────┬────────────┘
       ▼                      │
┌──────────────────────────┐  │ UDP:5005
│ perception_node (YOLOv5) │  │
│ decision_node (arbiter)  │◄─┤ voice>gesture>follow
│ stereo_avoidance         │  │
└──────────┬───────────────┘  │
           ▼                  │
┌──────────────────────────────┴─────────────┐
│ ros_udp_bridge  /puppy_action → UDP 5005   │
└──────────────────────┬─────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ sit.py (motor hub) → HiwonderPuppy gait    │
│ → servo_controller → /dev/ttyS1 → STM32 → 8 servos │
└─────────────────────────────────────────────┘
```

---

## 📁 Directory Structure

```
机器狗代码/
├── start_robot.sh                 # base launch (sit.py + IMU)
├── start_all.sh                   # one-click full system (4 chains)
├── README.md                      # Chinese README
├── README_EN.md                   # English README (this file)
├── PuppyPi_开源文档.md             # full Chinese docs (12 chapters)
│
├── puppy_ws/                      # ROS2 workspace (decision brain)
│   ├── src/puppy_brain/           # core pkg: 16 Python nodes + 6 launches
│   ├── config/                    # BPU model configs (*.hbm)
│   ├── models/                    # AI models (LLM/ASR/KWS)
│   ├── tools/                     # standalone tools (voice control, etc.)
│   └── docs/                      # detailed guides
│
├── gs130w_stereo/                 # binocular stereo vision subsystem
│   ├── launch/                    # stereo launches + camera info
│   ├── scripts/                   # start scripts + avoidance node + MJPEG
│   └── snapshots/view.html        # web visualization
│
├── pydev_demo/puppypi_control/   # low-level motion control
│   ├── sit.py                     # motor hub (UDP 5005 listener)
│   ├── HiwonderPuppy.so           # gait engine (closed-source)
│   ├── ActionGroups/              # 35 preset action groups (.d6ac)
│   └── 底层运动控制详解.md
│
└── standalone/                    # standalone demos
    ├── gesture_control.py
    └── yolo_display.py
```

---

## 🔧 Hardware & Software Requirements

### Hardware

| Part | Spec |
|------|------|
| Main board | Horizon RDK X5 (10 TOPS BPU, Cortex-A55 8-core) |
| Stereo cam | GS130W (dual sc132gs, MIPI0+MIPI2, 1280×1088) |
| Robot body | Hiwonder PuppyPi (8 DOF PWM servos + STM32) |
| USB mic/speaker | any USB audio device |
| TF card | ≥64GB U1 |

### Software

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 (Jammy) aarch64 |
| ROS2 | Humble (TROS 2.5.x) |
| Python | 3.10 |
| OpenCV | 4.x |
| vosk | 0.3.45 |
| webrtcvad | 2.0.10 |
| sherpa-onnx | ≥1.10 |

---

## 🚀 Quick Start

### 1. Deploy code to the board

```bash
scp -r 机器狗代码/* root@<board_IP>:/app/
```

### 2. Install dependencies

```bash
pip3 install opencv-python numpy vosk webrtcvad soundfile
bash /app/puppy_ws/tools/setup_sherpa.sh
sudo apt install -y sox alsa-utils netcat-openbsd
```

### 3. Build the ROS2 package

```bash
cd /app/puppy_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select puppy_brain
source install/setup.bash
```

### 4. One-click full system start

```bash
bash /app/start_all.sh start
```

### 5. Verify

Open `http://<board_IP>:8090/view.html` in a browser.

### Stop

```bash
bash /app/start_all.sh stop
```

---

## 🎛 Launch Flow

The system consists of 4 chains that **must start in dependency order** (handled automatically by `start_all.sh`):

| # | Command | Purpose |
|---|---------|---------|
| ① Base motion | `/app/start_robot.sh start` | sit.py motor hub (UDP 5005) + IMU 50Hz |
| ② Stereo vision | `/app/gs130w_stereo/scripts/start_v2.sh start` | GS130W + 5-stage AI + StereoNet + Web |
| ③ Stereo avoidance | `/app/gs130w_stereo/scripts/start_avoidance.sh start` | depth analysis + IMU correction → UDP |
| ④ Voice control | `python3 /app/puppy_ws/tools/voice_control_standalone.py --mic plughw:1,0 --speaker plughw:0,0 --gain 10 --aggressiveness 2 --silence 1.0` | VAD + Vosk + TTS |

```bash
# full start
bash /app/start_all.sh start

# skip voice
bash /app/start_all.sh start no_voice

# skip avoidance (switch back to LLM/gesture control)
bash /app/start_all.sh start no_avoidance

# status / stop / restart
bash /app/start_all.sh status
bash /app/start_all.sh stop
bash /app/start_all.sh restart
```

> ⚠️ `full_system.launch.py` and `start_avoidance.sh` cannot run together — both publish `/puppy_action` and would conflict.

---

## ⚙️ Key Configuration

### decision_node

| Param | Default | Description |
|-------|---------|-------------|
| `follow_area_near_stop` | 0.42 | brake when area ratio ≥ this |
| `follow_area_far_walk` | 0.10 | full speed when area ratio ≤ this |
| `turn_deadband_ratio` | 0.09 | turn deadband (anti-jitter) |
| `turn_gain` | 0.85 | turn gain |
| `control_smooth_alpha` | 0.28 | low-pass filter coefficient |
| `ghost_memory_time` | 0.30 | target-lost inertia (s) |

### stereo_avoidance_node

| Param | Default | Description |
|-------|---------|-------------|
| `danger_disp` | 30.0 | too close when disparity > this |
| `clear_disp` | 15.0 | clear when disparity < this |
| `use_imu_correction` | True | IMU heading correction |
| `yaw_gain` | 0.6 | heading correction gain |

Runtime tuning:

```bash
ros2 param set /decision_node turn_gain 0.6
```

---

## ✋ X5 Gesture Map

| Gesture | gesture_value | Action |
|---------|--------------|--------|
| palm | 5.0 | forward |
| thumb_up | 2.0 | sit |
| victory (V) | 3.0 | crouch |
| okay | 11.0 | backward |
| thumb_left | 12.0 | turn left |
| two palms | dual palm | turn right |

---

## 🗣️ Voice Commands

| Action | Trigger words |
|--------|---------------|
| forward | 前进 / 向前走 / 直走 |
| backward | 后退 / 倒车 |
| turn_left | 左转 / 左拐 |
| turn_right | 右转 / 右拐 |
| stand | 站起来 / 起立 |
| sit | 坐下 / 请坐 |
| crouch | 趴下 / 蹲下 |
| stop | 停下 / 别动 |

---

## 💡 Innovations

1. **On-device embodied loop** — 5 models run simultaneously on BPU 10 TOPS, zero cloud dependency
2. **Dual control protocol** — `follow_control` continuous velocity + discrete action groups, base-layer ramp smoothing
3. **Multi-modal priority arbitration** — voice > gesture > visual follow, lock mechanism + ghost-memory emergency stop
4. **Stereo avoidance + IMU correction** — StereoNet 3-zone disparity analysis, gyro integration fixes forward drift
5. **YUYV camera compatibility** — custom `usb_cam_publisher_node` replaces official `hobot_usb_cam` that doesn't support YUYV
6. **Pure-Python offline voice** — ROS2-independent, VAD auto-segmentation + fuzzy match + TTS sync anti-self-excitation

---

## 📚 Detailed Documentation

- [PuppyPi_开源文档.md](./PuppyPi_开源文档.md) — 12-chapter full Chinese docs (system/arch/algorithms/params/flow/innovations)
- [puppy_ws/docs/PuppyPi_从零上手极详细指南.md](./puppy_ws/docs/PuppyPi_从零上手极详细指南.md) — node-level getting-started guide
- [pydev_demo/puppypi_control/底层运动控制详解.md](./pydev_demo/puppypi_control/底层运动控制详解.md) — low-level motion technical reference

---

## 🧪 Common Debugging

```bash
# source env (every new terminal)
source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash

# inspect topics
ros2 topic list
ros2 topic echo /puppy_action
ros2 topic hz /perception/result_json

# send command via UDP directly
echo '{"action":"walk","source":"test"}' | nc -u -w1 127.0.0.1 5005
```

---

## ⚠️ Pre-release Cleanup

```bash
# run on the board to clean build artifacts and caches
cd /app
rm -rf puppy_ws/build puppy_ws/install puppy_ws/log
rm -rf pydev_demo/puppypi_control/puppy_env
find . -type d -name __pycache__ -exec rm -rf {} +
find . -name "*.pyc" -delete
```

---

## 📄 License

This project's code is released under the **MIT License**.
`HiwonderPuppy.so` gait engine is a closed-source library from Hiwonder; only the binary is provided, © Hiwonder.
BPU model files (`.hbm`/`.bin`) are configured per Horizon TROS official docs.

---

> Doc version: 2026-07-09 | Based on RDK X5 + TROS 2.5.x
