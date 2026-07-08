#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""usb_gesture.launch.py - USB 摄像头(YUYV) + 手势识别 + YOLOv5 + 机器狗手势控制

针对 icspring USB 摄像头 (仅支持 YUYV 4:2:2, 不支持 MJPEG):
  官方 hobot_usb_cam 无法采集 YUYV, 改用我们自己的 usb_cam_publisher_node.
  它发布 /image_raw/compressed (JPEG), hobot_codec 订阅后转 NV12 shared_mem.

数据流:
  usb_cam_publisher_node → /image_raw/compressed (JPEG, YUYV 采集后编码)
       │
       ├─→ hobot_codec (ros jpeg → shared_mem nv12) → /hbmem_img
       │       └─→ mono2d_body_detection → hand_lmk_detection
       │             → hand_gesture_detection → /hobot_hand_gesture_detection
       │                   │
       │                   ├─→ gesture_action_node → /puppy_action
       │                   │       └─→ ros_udp_bridge → UDP 5005 → sit.py
       │                   │
       │                   └─→ yolov5_mjpeg_server (叠加手势标签显示)
       │
       └─→ perception_node (YOLOv5) → /perception/result_json
               │
               └─→ yolov5_mjpeg_server (叠加 YOLO 框, HTTP 8093)

启动:
  终端1: ros2 launch puppy_brain usb_gesture.launch.py
  终端2: cd /app/pydev_demo/puppypi_control && python sit.py
  浏览器: http://<板端IP>:8093
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========= 可配置参数 =========
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    port_arg = DeclareLaunchArgument('port', default_value='8093')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/app/model/basic/yolov5s_672x672_nv12.bin'
    )

    device = LaunchConfiguration('device')
    port = LaunchConfiguration('port')
    model_path = LaunchConfiguration('model_path')

    return LaunchDescription([
        device_arg, port_arg, model_path_arg,

        # ========= 1. USB 摄像头采集 (YUYV → JPEG 发布) =========
        # 替代官方 hobot_usb_cam (它不支持 YUYV)
        Node(
            package='puppy_brain',
            executable='usb_cam_publisher_node',
            name='usb_cam_publisher_node',
            output='screen',
            parameters=[{
                'device': device,
                'width': 640,
                'height': 480,
                'fps': 30,
                'jpeg_quality': 60,
                'out_compressed_topic': '/image_raw/compressed',
                'out_raw_topic': '/image_raw',
            }],
        ),

        # ========= 2. hobot_codec (ROS jpeg → shared_mem nv12) =========
        # 订阅我们发布的 /image_raw/compressed, 输出 /hbmem_img 供 mono2d 消费
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='hobot_codec',
            output='screen',
            parameters=[{
                'channel': 1,
                'in_mode': 'ros',
                'out_mode': 'shared_mem',
                'sub_topic': '/image_raw/compressed',
                'pub_topic': '/hbmem_img',
                'in_format': 'jpeg',
                'out_format': 'nv12',
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # ========= 3. mono2d_body_detection (手部+人体检测) =========
        Node(
            package='mono2d_body_detection',
            executable='mono2d_body_detection',
            name='mono2d_body_detection',
            output='screen',
            parameters=[{
                'ai_msg_pub_topic_name': '/hobot_mono2d_body_detection',
                'model_file_name': '/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm',
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # ========= 4. hand_lmk_detection (手部 21 关键点) =========
        Node(
            package='hand_lmk_detection',
            executable='hand_lmk_detection',
            name='hand_lmk_detection',
            output='screen',
            parameters=[{
                'ai_msg_pub_topic_name': '/hobot_hand_lmk_detection',
                'ai_msg_sub_topic_name': '/hobot_mono2d_body_detection',
                'model_file_name': '/opt/tros/humble/lib/hand_lmk_detection/config/handLMKs.hbm',
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # ========= 5. hand_gesture_detection (手势分类) =========
        Node(
            package='hand_gesture_detection',
            executable='hand_gesture_detection',
            name='hand_gesture_detection',
            output='screen',
            parameters=[{
                'ai_msg_sub_topic_name': '/hobot_hand_lmk_detection',
                'ai_msg_pub_topic_name': '/hobot_hand_gesture_detection',
                'model_file_name': '/opt/tros/humble/lib/hand_gesture_detection/config/gestureDet_8x21.hbm',
                'is_dynamic_gesture': False,
                'time_interval_sec': 0.25,
                'threshold': 0.5,
            }],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # ========= 6. gesture_action_node (手势 → 动作, X5 手势映射) =========
        # 映射: palm(5)→前进, 双palm→右转, victory(3)→趴下,
        #       thumb_up(2)→坐下, okay(11)→后退, thumb_left(12)→左转
        Node(
            package='puppy_brain',
            executable='gesture_action_node',
            name='gesture_action_node',
            output='screen',
            parameters=[{
                'input_topic': '/hobot_hand_gesture_detection',
                'output_topic': '/puppy_action',
                'gesture_hold_sec': 0.4,
                'max_move_sec': 3.0,
                'action_lock_sec': 2.5,
                'control_rate_hz': 10.0,
                'forward_speed': 0.55,
                'backward_speed': 0.35,
                'turn_speed': 0.75,
            }],
        ),

        # ========= 7. ros_udp_bridge (/puppy_action → UDP 5005 → sit.py) =========
        Node(
            package='puppy_brain',
            executable='ros_udp_bridge',
            name='ros_udp_bridge',
            output='screen',
            parameters=[{
                'udp_ip': '127.0.0.1',
                'action_udp_port': 5005,
                'imu_udp_port': 5006,
            }],
        ),

        # ========= 8. perception_node (YOLOv5 推理) =========
        Node(
            package='puppy_brain',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'image_topic': '/image_raw/compressed',
                'score_threshold': 0.25,
                'nms_threshold': 0.45,
                'input_width': 672,
                'input_height': 672,
                'log_interval_sec': 5.0,
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        # ========= 9. yolov5_mjpeg_server (HTTP 8093 MJPEG 推流) =========
        Node(
            package='puppy_brain',
            executable='yolov5_mjpeg_server',
            name='yolov5_mjpeg_server',
            output='screen',
            parameters=[{
                'port': port,
                'image_topic': '/image_raw/compressed',
                'perception_topic': '/perception/result_json',
                'gesture_topic': '/hobot_hand_gesture_detection',
                'coco_names_file': '/app/pydev_demo/07_usb_camera_sample/coco_classes.names',
                'jpeg_quality': 70,
            }],
        ),
    ])
