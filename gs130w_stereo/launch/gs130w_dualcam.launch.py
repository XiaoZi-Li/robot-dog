#!/usr/bin/env python3
"""GS130W dual-cam launch - same as the official 132gs launch, but with
absolute GDC bin path, real SC132GS dual calibration yaml, and overridable."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


GDC_DEFAULT = '/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin'
CAL_DEFAULT = '/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml'


def generate_launch_description():
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    mipi_gdc_bin_file_arg = DeclareLaunchArgument(
        'mipi_gdc_bin_file',
        default_value=GDC_DEFAULT,
        description='mipi camera gdc bin file (absolute path)')
    mipi_camera_calibration_file_path_arg = DeclareLaunchArgument(
        'mipi_camera_calibration_file_path',
        default_value=CAL_DEFAULT,
        description='absolute path to SC132GS dual calibration yaml')
    mipi_image_framerate_arg = DeclareLaunchArgument(
        'mipi_image_framerate',
        default_value='10.0',
        description='mipi camera framerate')

    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch/mipi_cam_dual_channel.launch.py')),
        launch_arguments={
            'mipi_image_width': '1280',
            'mipi_image_height': '1088',
            'mipi_sub_image_width': '1280',
            'mipi_sub_image_height': '1088',
            'mipi_image_framerate': LaunchConfiguration('mipi_image_framerate'),
            'mipi_io_method': 'ros',
            'device_mode': 'dual',
            'dual_combine': '2',
            'mipi_channel': '2',
            'mipi_channel2': '0',
            'mipi_lpwm_enable': 'True',
            'mipi_camera_calibration_file_path':
                LaunchConfiguration('mipi_camera_calibration_file_path'),
            'mipi_gdc_bin_file': LaunchConfiguration('mipi_gdc_bin_file'),
            'mipi_rotation': '90.0',
            'mipi_cal_rotation': '0.0',
            'mipi_gdc_enable': 'True',
            'mipi_stream_mode': '1',
            'mipi_sub_stream_enable': 'True',
            'mipi_frame_ts_type': 'sensor'
        }.items()
    )

    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_name': 'jpeg_codec_node',
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'nv12',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_combine_raw',
            'codec_pub_topic': '/image_combine_jpeg'
        }.items()
    )
    sub_jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_name': 'sub_jpeg_codec_node',
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'nv12',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/sub_image_combine_raw',
            'codec_pub_topic': '/sub_image_combine_jpeg'
        }.items()
    )
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_combine_jpeg',
            'websocket_channel': '0',
            'websocket_only_show_image': 'True'
        }.items()
    )
    sub_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_combine_jpeg',
            'websocket_channel': '1',
            'websocket_only_show_image': 'True'
        }.items()
    )
    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    return LaunchDescription([
        mipi_gdc_bin_file_arg,
        mipi_camera_calibration_file_path_arg,
        mipi_image_framerate_arg,
        shared_mem_node,
        mipi_node,
        jpeg_codec_node,
        sub_jpeg_codec_node,
        web_node,
        sub_web_node,
    ])