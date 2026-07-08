from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mono2d_lib_dir = '/opt/tros/humble/lib/mono2d_body_detection'
    hand_lmk_lib_dir = '/opt/tros/humble/lib/hand_lmk_detection'
    hand_gesture_lib_dir = '/opt/tros/humble/lib/hand_gesture_detection'

    # 0) 用官方图片发布器给 /hbmem_img 喂一张测试图
    image_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_image_publisher'),
                'launch',
                'hobot_image_publisher.launch.py'
            )
        ),
        launch_arguments={
            'publish_image_source': '/opt/tros/humble/lib/hand_gesture_detection/config/person_face_hand.jpg',
            'publish_image_format': 'jpg',
            'publish_message_topic_name': '/hbmem_img',
            'publish_fps': '5',
            'publish_is_shared_mem': 'True',
            'publish_is_loop': 'True',
            'publish_output_image_w': '960',
            'publish_output_image_h': '544',
            'publish_encoding': 'nv12',
         }.items()
    )
    # 1) hbmem_img -> image
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch',
                'hobot_codec_encode.launch.py'
            )
        ),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # 2) websocket
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch',
                'websocket.launch.py'
            )
        ),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': '/hobot_mono2d_body_detection'
        }.items()
    )

    # 3) 官方人体检测
    mono2d_body_node = Node(
        package='mono2d_body_detection',
        executable='mono2d_body_detection',
        name='mono2d_body_detection',
        output='screen',
        parameters=[{
            'ai_msg_pub_topic_name': '/hobot_mono2d_body_detection'
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        cwd=mono2d_lib_dir,
    )

    # 4) 官方手关键点
    hand_lmk_node = Node(
        package='hand_lmk_detection',
        executable='hand_lmk_detection',
        name='hand_lmk_detection',
        output='screen',
        parameters=[{
            'ai_msg_pub_topic_name': '/hobot_hand_lmk_detection',
            'ai_msg_sub_topic_name': '/hobot_mono2d_body_detection',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        cwd=hand_lmk_lib_dir,
    )

    # 5) 官方手势识别
    hand_gesture_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        name='hand_gesture_detection',
        output='screen',
        parameters=[{
            'ai_msg_pub_topic_name': '/hobot_hand_gesture_detection',
            'ai_msg_sub_topic_name': '/hobot_hand_lmk_detection',
            'is_dynamic_gesture': False,
            'time_interval_sec': 0.25,
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
        cwd=hand_gesture_lib_dir,
    )

    return LaunchDescription([
        image_pub_node,
        jpeg_codec_node,
        mono2d_body_node,
        web_node,
        hand_lmk_node,
        hand_gesture_node,
    ])