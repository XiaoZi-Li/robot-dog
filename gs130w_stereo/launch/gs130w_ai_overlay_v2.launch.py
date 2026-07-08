#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GS130W AI overlay v2: 在已有 mipi_cam/codec/websocket 之上叠加
   mono2d_body_detection + face_landmarks + hand_lmk + hand_gesture + stereonet
   所有 AI 节点用 Node() 形式启动，不重启 mipi_cam。
   stereonet 已显式禁用所有 save_* 标志（避免再涨 100G+）。
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

# ============ 路径 ============
TROS = '/opt/tros/humble'
MONO2D_MODEL = f'{TROS}/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm'
FACE_LMK_MODEL = f'{TROS}/share/face_landmarks_detection/config/faceLandmark106pts.hbm'
HAND_LMK_MODEL = f'{TROS}/lib/hand_lmk_detection/config/handLMKs.hbm'
HAND_GEST_MODEL = f'{TROS}/lib/hand_gesture_detection/config/gestureDet_32x21.hbm'
# stereonet 模型实际在 share/ 下（不是 lib/）
STEREONET_MODEL = f'{TROS}/share/hobot_stereonet/config/DStereoV2.0.bin'

# ============ 订阅源（当前 mipi_cam 已在跑，topic 已知） ============
IMG_RAW = '/image_combine_raw'          # 原始 main 流（stereonet 用这个）
SUB_IMG_RAW = '/sub_image_combine_raw'  # 校正后子流（face/hand/mono2d 用这个）
SUB_IMG_JPEG = '/sub_image_combine_jpeg'  # 校正后子流 jpeg（websocket 推流用）

# ============ AI 输出 topic ============
T_MONO2D = '/hobot_mono2d_body_detection'
T_FACE_LMK = '/hobot_face_landmarks_detection'
T_HAND_LMK = '/hobot_hand_lmk_detection'
T_HAND_GEST = '/hobot_hand_gesture_detection'

# ============ WebSocket 通道 + 端口分配（不与 8000/8080/8082 冲突） ============
# 0/1 已被 C 方案占（main / sub jpeg），这里新增 2/3/4
# websocket 节点要 int，**不是 str**
CH_FACE = 2
CH_HAND = 3
CH_DEPTH = 4
WS_PORT_FACE = 8084
WS_PORT_HAND = 8086
WS_PORT_DEPTH = 8088

# ============ 节点 1: mono2d_body_detection（前置，订阅 SUB_IMG_RAW） ============
mono2d_node = Node(
    package='mono2d_body_detection',
    executable='mono2d_body_detection',
    name='mono2d_body_detection',
    output='screen',
    parameters=[{
        'model_file_name': MONO2D_MODEL,
        'model_type': 0,
        'is_shared_mem_sub': 0,
        'ros_img_topic_name': SUB_IMG_RAW,
        'ai_msg_pub_topic_name': T_MONO2D,
        'is_sync_mode': 0,
        'image_gap': 1,
        'dump_render_img': 0,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ 节点 2: face_landmarks_detection（订阅 mono2d 的 face 输出） ============
face_lmk_node = Node(
    package='face_landmarks_detection',
    executable='face_landmarks_detection',
    name='face_landmarks_detection',
    output='screen',
    parameters=[{
        'model_file_name': FACE_LMK_MODEL,
        'feed_type': 0,
        'is_sync_mode': 0,
        'is_shared_mem_sub': 0,
        'ros_img_topic_name': SUB_IMG_RAW,
        'ai_msg_sub_topic_name': T_MONO2D,
        'ai_msg_pub_topic_name': T_FACE_LMK,
        'roi_xyxy': '0,0,1280,1088',
        'dump_render_img': 0,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ 节点 3a: hand_lmk_detection（订阅 mono2d 的 hand 输出） ============
hand_lmk_node = Node(
    package='hand_lmk_detection',
    executable='hand_lmk_detection',
    name='hand_lmk_detection',
    output='screen',
    parameters=[{
        'model_file_name': HAND_LMK_MODEL,
        'is_shared_mem_sub': 0,
        'ros_img_topic_name': SUB_IMG_RAW,
        'ai_msg_sub_topic_name': T_MONO2D,
        'ai_msg_pub_topic_name': T_HAND_LMK,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ 节点 3b: hand_gesture_detection（订阅 hand_lmk 输出） ============
hand_gesture_node = Node(
    package='hand_gesture_detection',
    executable='hand_gesture_detection',
    name='hand_gesture_detection',
    output='screen',
    parameters=[{
        'model_file_name': HAND_GEST_MODEL,
        'ai_msg_sub_topic_name': T_HAND_LMK,
        'ai_msg_pub_topic_name': T_HAND_GEST,
        'is_dynamic_gesture': False,
        'time_interval_sec': 0.25,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ 节点 4: stereonet 立体匹配（直接订阅 /image_combine_raw） ============
# 【重要】stereonet 默认会把每帧的 left/right/disp/depth/visual/pcd 全存到 cwd
# 半小时能写 100G+，必须显式禁用全部 save_* 标志 + save_dir 写到 /dev/null
stereonet_node = Node(
    package='hobot_stereonet',
    executable='stereonet_model_node',
    name='StereoNetNode',
    output='screen',
    parameters=[{
        'stereonet_model_file_path': STEREONET_MODEL,
        'stereo_image_topic': IMG_RAW,
        'publish_visual_enabled': True,
        'publish_pcd_enabled': False,  # 无人订阅，关掉省 CPU/带宽
        'publish_rectify_bgr': False,
        'render_type': 'indoor',
        'render_perf': True,
        'log_level': 'warn',
        # === 禁止存盘（修盘满的关键修复）===
        'save_result_flag': False,
        'save_stereo_flag': False,
        'save_origin_flag': False,
        'save_disp_flag': False,
        'save_uncert_flag': False,
        'save_depth_flag': False,
        'save_visual_flag': False,
        'save_pcd_flag': False,
        'save_dir': '/dev/null',
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ Codec: stereonet_visual (bgr8) -> jpeg（喂给 websocket） ============
# hobot_codec_republish 实际参数名: in_mode/out_mode/sub_topic/pub_topic/in_format/out_format/jpg_quality
stereonet_codec_node = Node(
    package='hobot_codec',
    executable='hobot_codec_republish',
    name='stereonet_visual_codec',
    output='screen',
    parameters=[{
        'in_mode': 'ros',
        'out_mode': 'ros',
        'sub_topic': '/StereoNetNode/stereonet_visual',
        'pub_topic': '/StereoNetNode/stereonet_visual_jpeg',
        'in_format': 'bgr8',
        'out_format': 'jpeg',
        'jpg_quality': 85.0,
        'log_level': 'warn',
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ WebSocket 1: 人脸（订阅 /sub_image_combine_jpeg + T_FACE_LMK） ============
ws_face_node = Node(
    package='websocket',
    executable='websocket',
    name='websocket_face',
    output='screen',
    parameters=[{
        'image_topic': SUB_IMG_JPEG,
        'image_type': 'mjpeg',
        'only_show_image': False,
        'output_fps': 0,
        'channel': CH_FACE,
        'smart_topic': T_FACE_LMK,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ WebSocket 2: 手势（订阅 /sub_image_combine_jpeg + T_HAND_GEST） ============
ws_hand_node = Node(
    package='websocket',
    executable='websocket',
    name='websocket_hand',
    output='screen',
    parameters=[{
        'image_topic': SUB_IMG_JPEG,
        'image_type': 'mjpeg',
        'only_show_image': False,
        'output_fps': 0,
        'channel': CH_HAND,
        'smart_topic': T_HAND_GEST,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)

# ============ WebSocket 3: 深度可视化（订阅 stereonet_visual_jpeg） ============
ws_depth_node = Node(
    package='websocket',
    executable='websocket',
    name='websocket_depth',
    output='screen',
    parameters=[{
        'image_topic': '/StereoNetNode/stereonet_visual_jpeg',
        'image_type': 'mjpeg',
        'only_show_image': True,
        'output_fps': 0,
        'channel': CH_DEPTH,
    }],
    arguments=['--ros-args', '--log-level', 'warn']
)


def generate_launch_description():
    return LaunchDescription([
        # AI 模型推理
        mono2d_node,
        face_lmk_node,
        hand_lmk_node,
        hand_gesture_node,
        stereonet_node,
        # stereonet bgr8 -> jpeg
        stereonet_codec_node,
        # 3 个新 WebSocket 通道（端口 8084 / 8086 / 8088）
        ws_face_node,
        ws_hand_node,
        ws_depth_node,
    ])
