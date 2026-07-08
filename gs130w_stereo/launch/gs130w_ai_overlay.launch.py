#!/usr/bin/env python3
"""GS130W AI overlay: directly reuse the working 132gs launch as image source,
then add face_landmarks + hand_gesture + stereonet, each with own codec+ws."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def lp(pkg, *sub):
    return os.path.join(get_package_share_directory(pkg), *sub)


GDC = '/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin'


def generate_launch_description():
    items = []

    # 1) fastdds shm zero-copy
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_shm', 'launch', 'hobot_shm.launch.py'))
    ))

    # 2) REUSE the proven 132gs launch (mipi_cam dual + dual codec/ws)
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('mipi_cam', 'launch',
                                        'mipi_cam_dual_channel_websocket_132gs_nocal+cal+r90.launch.py')),
        launch_arguments={
            'mipi_gdc_bin_file': GDC,
            'mipi_image_framerate': '10.0',
        }.items(),
    ))

    # 3) Face detection + landmarks (consumes hbmem; face_landmarks default)
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('face_landmarks_detection',
                                        'launch', 'body_det_face_landmarks_det.launch.py')),
        launch_arguments={'log_level': 'warn'}.items(),
    ))
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_only_show_image': 'False',
            'websocket_smart_topic': '/hobot_face_landmarks_detection',
            'websocket_channel': '2',
        }.items(),
    ))

    # 4) Hand gesture detection
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hand_gesture_detection',
                                        'launch', 'hand_gesture_detection.launch.py')),
        launch_arguments={'log_level': 'warn'}.items(),
    ))
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_only_show_image': 'False',
            'websocket_smart_topic': '/hobot_hand_gesture_detection',
            'websocket_channel': '3',
        }.items(),
    ))

    # 5) StereoNet (web_visual w/ own codec+ws)
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_stereonet',
                                        'launch', 'stereonet_model_web_visual.launch.py')),
        launch_arguments={
            'use_mipi_cam': 'False',
            'stereonet_pub_web': 'True',
            'stereonet_model_file_path': lp('hobot_stereonet', 'config', 'DStereoV2.0.bin'),
            'log_level': 'warn',
        }.items(),
    ))

    return LaunchDescription(items)
