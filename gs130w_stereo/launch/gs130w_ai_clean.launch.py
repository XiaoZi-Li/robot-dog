#!/usr/bin/env python3
"""GS130W AI full pipeline: dual-cam (fixed GDC path) + face + hand + stereonet."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def lp(pkg, *sub):
    return os.path.join(get_package_share_directory(pkg), *sub)


GDC = '/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin'
MONO2D_MODEL = '/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm'
FACE_LMK_MODEL = '/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm'
HAND_LMK_MODEL = '/opt/tros/humble/lib/hand_lmk_detection/config/handLMKs.hbm'
HAND_GEST_MODEL = '/opt/tros/humble/lib/hand_gesture_detection/config/gestureDet_32x21.hbm'


def generate_launch_description():
    items = []

    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/tmp/gs130w_dualcam.launch.py'),
        launch_arguments={'mipi_gdc_bin_file': GDC, 'mipi_image_framerate': '10.0'}.items(),
    ))

    mono2d_topic = '/hobot_mono2d_body_detection'
    items.append(Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        output='screen',
        parameters=[{
            'model_file_name': MONO2D_MODEL,
            'model_type': 0,
            'is_shared_mem_sub': 0,
            'ros_img_topic_name': '/sub_image_combine_raw',
            'ai_msg_pub_topic_name': mono2d_topic,
            'is_sync_mode': 0,
            'image_gap': 1,
            'dump_render_img': 0,
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    ))

    face_topic = '/hobot_face_landmarks_detection'
    items.append(Node(
        package='face_landmarks_detection',
        executable='face_landmarks_detection',
        output='screen',
        parameters=[{
            'model_file_name': FACE_LMK_MODEL,
            'feed_type': 0,
            'is_sync_mode': 0,
            'is_shared_mem_sub': 0,
            'ros_img_topic_name': '/sub_image_combine_raw',
            'ai_msg_sub_topic_name': mono2d_topic,
            'ai_msg_pub_topic_name': face_topic,
            'roi_xyxy': '0,0,1280,1088',
            'dump_render_img': 0,
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    ))

    hand_lmk_topic = '/hobot_hand_lmk_detection'
    items.append(Node(
        package='hand_lmk_detection',
        executable='hand_lmk_detection',
        output='screen',
        parameters=[{
            'model_file_name': HAND_LMK_MODEL,
            'is_shared_mem_sub': 0,
            'ros_img_topic_name': '/sub_image_combine_raw',
            'ai_msg_sub_topic_name': mono2d_topic,
            'ai_msg_pub_topic_name': hand_lmk_topic,
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    ))

    hand_gesture_topic = '/hobot_hand_gesture_detection'
    items.append(Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        output='screen',
        parameters=[{
            'model_file_name': HAND_GEST_MODEL,
            'ai_msg_sub_topic_name': hand_lmk_topic,
            'ai_msg_pub_topic_name': hand_gesture_topic,
            'is_dynamic_gesture': False,
            'time_interval_sec': 0.25,
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    ))

    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_only_show_image': 'False',
            'websocket_smart_topic': face_topic,
            'websocket_channel': '2',
        }.items(),
    ))
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_only_show_image': 'False',
            'websocket_smart_topic': hand_gesture_topic,
            'websocket_channel': '3',
        }.items(),
    ))

    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_stereonet',
                                        'launch', 'stereonet_model_web_visual.launch.py')),
        launch_arguments={
            'use_mipi_cam': 'False',
            'stereonet_pub_web': 'True',
            'log_level': 'warn',
        }.items(),
    ))

    return LaunchDescription(items)