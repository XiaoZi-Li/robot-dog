#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""full_system_cloud.launch.py - 完整系统 (GS130W 双目 + 云端 LLM)

= full_system.launch.py 区别:
  1. 摄像头: F37 单目 → GS130W 双目 (官方 mipi_cam_dual_channel_websocket)
  2. AI 图像源: /image → /sub_image_combine_jpeg (双目子流 960x544)
  3. LLM: hobot_llamacpp (本地 Qwen2.5-0.5B, 吃 CPU) → cloud_llm_node (DeepSeek API)
  4. 加 stereonet 深度估计
  5. 不起官方 websocket (8000 端口), 改用 mjpeg_bridge 8071/8072/8073

用法:
  export DEEPSEEK_API_KEY="sk-xxx"
  source /opt/tros/humble/setup.bash
  source install/setup.bash
  ros2 launch puppy_brain full_system_cloud.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

# ============ 双目 topic 约定 (来自 gs130w_stereo 项目) ============
IMG_RAW = '/image_combine_raw'              # 原始 main 流 (stereonet 用)
SUB_IMG_RAW = '/sub_image_combine_raw'      # 校正后子流 (AI 用)
SUB_IMG_JPEG = '/sub_image_combine_jpeg'    # 校正后子流 jpeg (perception 用)
DEPTH_VIS_JPEG = '/StereoNetNode/stereonet_visual_jpeg'

# ============ 模型路径 ============
STEREONET_MODEL = '/opt/tros/humble/share/hobot_stereonet/config/DStereoV2.0.bin'
MONO2D_MODEL = '/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm'
HAND_LMK_MODEL = '/opt/tros/humble/lib/hand_lmk_detection/config/handLMKs.hbm'
HAND_GESTURE_MODEL = '/opt/tros/humble/lib/hand_gesture_detection/config/gestureDet_8x21.hbm'
GDC_BIN = '/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin'
CALIB_YAML = '/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml'
CAMERA_INFO_PUB = '/app/gs130w_stereo/launch/camera_info_publisher.py'


def generate_launch_description():
    # ============ 环境变量 ============
    env_lang = SetEnvironmentVariable('LANG', 'C.UTF-8')
    env_lc_all = SetEnvironmentVariable('LC_ALL', 'C.UTF-8')
    env_language = SetEnvironmentVariable('LANGUAGE', 'C.UTF-8')

    api_key = os.environ.get('DEEPSEEK_API_KEY', '')

    # ============ 视觉 + AI 推理由 start_v2.sh 单独管理 ============
    # 不在本 launch 启动: mipi_cam / camera_info / stereonet / stereonet_codec
    #                     / mono2d / hand_lmk / hand_gesture / mjpeg_bridge
    # 原因: gs130w_ai_overlay_v2.launch.py (start_v2.sh 启动) 已含这些
    #       重复启动会抢 BPU 和模型文件
    # 前置条件: 先跑 /app/gs130w_stereo/scripts/start_v2.sh start
    # 本 launch 只起: gesture_adapter / perception / 决策 / LLM / WS桥 / TTS

    # ============ 自定义视觉适配节点 (订阅 start_v2.sh 的 AI 输出) ============
    # gesture_adapter: hobot 手势结果 → 统一 JSON
    # perception_node: YOLOv5 通用检测 (订阅双目子流)
    gesture_adapter = Node(
        package='puppy_brain',
        executable='gesture_adapter_node',
        name='gesture_adapter_node',
        output='screen',
        parameters=[{
            'input_topic': '/hobot_hand_gesture_detection',
            'output_topic': '/gesture/result_json',
            'log_interval_sec': 0.5,
        }]
    )

    perception_node = Node(
        package='puppy_brain',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'model_path': '/app/model/basic/yolov5s_672x672_nv12.bin',
            'score_threshold': 0.25,
            'nms_threshold': 0.45,
            'nms_top_k': 20,
            'input_width': 672,
            'input_height': 672,
            'orig_width': 960,
            'orig_height': 544,
            'image_topic': SUB_IMG_JPEG,
            'log_interval_sec': 5.0,
        }]
    )

    # ============ 语音: USB 麦克风 + Vosk ASR ============
    usb_asr = Node(
        package='puppy_brain',
        executable='usb_asr_text_node',
        name='usb_asr_text_node',
        output='screen',
        parameters=[{
            'device': 'plughw:2,0',
            'record_seconds': 3,
            'sample_rate': 16000,
            'channels': 1,
            'model_path': '/app/puppy_ws/models/vosk-model-small-cn-0.22',
            'loop_sleep_sec': 0.3,
            'min_text_length': 1,
        }]
    )

    intent_router = Node(
        package='puppy_brain',
        executable='intent_router_node',
        name='intent_router_node',
        output='screen',
        parameters=[{
            'control_cooldown_sec': 1.5,
        }]
    )

    decision_node = Node(
        package='puppy_brain',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'image_width': 960.0,
            'image_height': 544.0,
            'follow_area_near_stop': 0.42,
            'follow_area_far_walk': 0.10,
            'min_valid_area_ratio': 0.015,
            'center_ratio': 0.50,
            'turn_deadband_ratio': 0.09,
            'max_turn_error_ratio': 0.28,
            'turn_gain': 0.85,
            'forward_min': 0.0,
            'forward_max': 0.95,
            'ghost_memory_time': 0.30,
            'publish_repeat_sec': 0.15,
            'gesture_hold_sec': 0.8,
            'follow_default_enabled': True,
            'gesture_action_lock_sec': 2.5,
            'gesture_stop_lock_sec': 1.0,
            'voice_action_lock_sec': 2.5,
            'voice_priority_enabled': True,
            'voice_move_sec': 2.5,
            'voice_forward_speed': 0.55,
            'voice_backward_speed': 0.35,
            'voice_turn_speed': 0.75,
            'control_smooth_alpha': 0.28,
            'turn_zero_threshold': 0.05,
            'forward_zero_threshold': 0.05,
            'debug_print_sec': 0.5,
        }]
    )

    udp_bridge = Node(
        package='puppy_brain',
        executable='ros_udp_bridge',
        name='ros_udp_bridge',
        output='screen',
        parameters=[{
            'udp_ip': '127.0.0.1',
            'udp_port': 5005,
            'imu_udp_ip': '127.0.0.1',
            'imu_udp_port': 5006,
        }]
    )

    imu_node = Node(
        package='puppy_brain',
        executable='imu_node_ros2',
        name='imu_node_ros2',
        output='screen',
        parameters=[{
            'topic_name': '/ros_robot_controller/imu_raw',
            'publish_hz': 50.0,
        }],
        arguments=['--ros-args', '--log-level', 'error']
    )

    ws_bridge = Node(
        package='puppy_brain',
        executable='ws_bridge_node',
        name='ws_bridge_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # ============ 云端 LLM + 桥接 ============
    cloud_llm_node = Node(
        package='puppy_brain',
        executable='cloud_llm_node',
        name='cloud_llm_node',
        output='screen',
        parameters=[{
            'api_key': api_key,
            'base_url': 'https://api.deepseek.com',
            'model': 'deepseek-chat',
            'sub_topic': '/prompt_text',
            'pub_topic': '/tts_text',
            'max_tokens': 200,
            'temperature': 0.7,
            'max_history': 6,
            'request_timeout': 30.0,
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    chat_bridge = Node(
        package='puppy_brain',
        executable='chat_llm_bridge_node',
        name='chat_llm_bridge_node',
        output='screen',
        parameters=[{
            'chat_input_topic': '/chat/input_text',
            'chat_output_topic': '/chat/response_text',
            'llm_input_topic': '/prompt_text',
            'llm_output_topic': '/tts_text',
            'flush_timeout_sec': 2.0,
        }]
    )

    tts_node = Node(
        package='puppy_brain',
        executable='tts_play_node',
        name='tts_play_node',
        output='screen',
        parameters=[{
            'backend': 'auto',
            'play_device': 'plughw:1,0',
            'cache_dir': '/tmp/tts_cache',
            'flush_timeout_sec': 0.8,
            'volume_db': '+2',
            'sherpa_model_root': '/opt/sherpa-models',
            'sherpa_tts_subdir': 'matcha-zh-baker',
            'sherpa_vocoder_subdir': 'vocoder',
            'sherpa_vocoder_file': 'hifigan_v2.onnx',
            'sherpa_speed': 1.0,
            'edge_voice': 'zh-CN-XiaoxiaoNeural',
        }]
    )

    # ============ MJPEG 桥由 start_vision.sh 单独管理 ============
    # 不在 launch 里起 mjpeg_bridge, 避免和 start_vision.sh 冲突
    # 如需上位机看视频, 先跑: ./start_vision.sh start

    return LaunchDescription([
        env_lang, env_lc_all, env_language,

        # 视觉适配 (订阅 start_v2.sh 产生的 AI 结果 topic)
        gesture_adapter, perception_node,

        # 语音 + 决策 + 控制
        usb_asr, intent_router, decision_node,
        udp_bridge, imu_node,

        # 上位机 + LLM + TTS
        ws_bridge, cloud_llm_node, chat_bridge, tts_node,
    ])
