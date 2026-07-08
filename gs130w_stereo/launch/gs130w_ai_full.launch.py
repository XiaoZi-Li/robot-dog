#!/usr/bin/env python3
"""GS130W AI full-stack: dual-cam + face_landmarks + hand_gesture + stereonet."""

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

    # 1) fastdds shm zero-copy profile
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_shm', 'launch', 'hobot_shm.launch.py'))
    ))

    # 2) mipi_cam dual_channel ros-topic mode + GDC + 132gs tuning
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('mipi_cam', 'launch', 'mipi_cam_dual_channel.launch.py')),
        launch_arguments={
            'mipi_io_method': 'ros',
            'mipi_image_width': '1280', 'mipi_image_height': '1088',
            'mipi_sub_image_width': '1280', 'mipi_sub_image_height': '1088',
            'mipi_image_framerate': '10.0',
            'device_mode': 'dual', 'dual_combine': '2',
            'mipi_channel': '2', 'mipi_channel2': '0',
            'mipi_lpwm_enable': 'True',
            'mipi_rotation': '90.0', 'mipi_cal_rotation': '0.0',
            'mipi_gdc_enable': 'True', 'mipi_gdc_bin_file': GDC,
            'mipi_sub_stream_enable': 'True', 'mipi_stream_mode': '1',
            'mipi_frame_ts_type': 'sensor',
            'log_level': 'warn',
        }.items(),
    ))

    # 3) main dual image -> jpeg codec -> websocket channel=0
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_codec', 'launch', 'hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros', 'codec_in_format': 'nv12',
            'codec_out_mode': 'ros', 'codec_out_format': 'jpeg',
            'codec_sub_topic': '/image_combine_raw',
            'codec_pub_topic': '/image_combine_jpeg',
            'codec_jpg_quality': '85.0', 'codec_input_framerate': '10',
        }.items(),
    ))
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_combine_jpeg',
            'websocket_only_show_image': 'True',
            'websocket_channel': '0',
        }.items(),
    ))

    # 4) rectified sub image -> jpeg codec -> websocket channel=1
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('hobot_codec', 'launch', 'hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros', 'codec_in_format': 'nv12',
            'codec_out_mode': 'ros', 'codec_out_format': 'jpeg',
            'codec_sub_topic': '/sub_image_combine_raw',
            'codec_pub_topic': '/sub_image_combine_jpeg',
            'codec_jpg_quality': '85.0', 'codec_input_framerate': '10',
        }.items(),
    ))
    items.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lp('websocket', 'launch', 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_only_show_image': 'True',
            'websocket_channel': '1',
        }.items(),
    ))

    # 5) face detection + landmarks (default subscribes hbmem - may need mipi_cam shm mode)
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

    # 6) hand gesture detection (lmk + static gesture)
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

    # 7) stereonet depth + web visual (own codec+websocket)
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
