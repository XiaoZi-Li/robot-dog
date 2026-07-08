# GS130W 双目视觉识别（RDK X5 / TROS）

按官方文档移植：<https://developer.d-robotics.cc/accessories_doc/stereo_camera_gs130w/installation>

硬件：GS130W 双目模组（双 sc132gs，MIPI0 + MIPI2） + RDK X5

## 目录

```
gs130w_stereo/
├── README.md
├── launch/
│   ├── gs130w_dualcam.launch.py        # 裸双目图 + 编码 + Web（无 AI）
│   ├── gs130w_ai_clean.launch.py       # 干净 AI 全栈（双图 + mono2d + face + hand + stereonet）
│   ├── gs130w_ai_full.launch.py        # 完整 AI 全栈（双图 + face_landmarks + hand_gesture + stereonet）
│   └── gs130w_ai_overlay.launch.py     # 复用现成 132gs 适配 launch 的 AI 叠加版
└── scripts/
    └── start_gs130w.sh                 # 启动脚本（统一入口）
```

## 关键路径（已在板端验证）

| 项 | 路径 |
| --- | --- |
| GDC bin | `/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin` |
| SC132gs 标定 yaml | `/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml` |
| 官方适配 launch | `/opt/tros/humble/share/mipi_cam/launch/mipi_cam_dual_channel_websocket_132gs_nocal+cal+r90.launch.py` |
| TROS 源 | `/opt/tros/humble/setup.bash` |
| TROS 版本 | `2.5.2-jammy.20260313.064039` |

## 快速开始（按官方文档）

### 1. 升级 TROS（首次需要）

```bash
sudo apt update && sudo apt upgrade
apt show tros-humble
```

### 2. 启动（最简：只跑双目图到 Web）

```bash
bash /app/gs130w_stereo/scripts/start_gs130w.sh dualcam
```

启动后浏览器访问 `http://<板端IP>:8000` 看左右眼实时图。

### 3. 启动（完整 AI：人脸 + 手势 + 立体匹配 + Web 可视化）

```bash
bash /app/gs130w_stereo/scripts/start_gs130w.sh ai_full
```

## 4 个 launch 的差异

| launch | 双目图 | 人脸/关键点 | 手势 | 立体匹配 | Web 可视化 |
| --- | :-: | :-: | :-: | :-: | :-: |
| `dualcam` | ✓ | – | – | – | ✓ |
| `ai_clean` | ✓ | ✓ (mono2d+face_lmk) | ✓ (hand_lmk+gesture) | ✓ | ✓ |
| `ai_full` | ✓ | ✓ (face_landmarks_detection) | ✓ (hand_gesture_detection) | ✓ | ✓ |
| `ai_overlay` | ✓ (复用 132gs 适配 launch) | ✓ | ✓ | ✓ | ✓ |

## 端口

- Web 显示端口：8000（被 nginx 占着，websocket 节点会复用它；如需独立调试可改 `websocket.launch.py` 的端口）
- 8000 当前被 `nginx` (pid 22499/22500) 监听 —— 这是 RDK Studio 自带的 nginx + WebSocket 反代；ros2 启动的 websocket.launch 内部会再开一个 ws 端口
