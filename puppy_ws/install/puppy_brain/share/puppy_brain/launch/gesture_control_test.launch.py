from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            name='mipi_cam',
            output='screen',
            parameters=[{
                'out_format': 'nv12',
                'io_method': 'shared_mem',
                'video_device': 'F37',
                'image_width': 960,
                'image_height': 544,
            }],
            arguments=['--ros-args', '--log-level', 'error']
        ),

        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='hobot_codec_encoder',
            output='screen',
            parameters=[{
                'channel': 1,
                'in_mode': 'shared_mem',
                'out_mode': 'ros',
                'sub_topic': '/hbmem_img',
                'pub_topic': '/image',
                'in_format': 'nv12',
                'out_format': 'jpeg',
                'jpg_quality': 60.0,
            }],
            arguments=['--ros-args', '--log-level', 'error']
        ),

        Node(
            package='mono2d_body_detection',
            executable='mono2d_body_detection',
            name='mono2d_body_detection',
            output='screen',
            parameters=[{
                'ai_msg_pub_topic_name': '/hobot_mono2d_body_detection',
                'model_file_name': '/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm',
            }],
            arguments=['--ros-args', '--log-level', 'error']
        ),

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
            arguments=['--ros-args', '--log-level', 'error']
        ),

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
            arguments=['--ros-args', '--log-level', 'error']
        ),

        Node(
            package='puppy_brain',
            executable='gesture_adapter_node',
            name='gesture_adapter_node',
            output='screen',
            parameters=[{
                'input_topic': '/hobot_hand_gesture_detection',
                'output_topic': '/gesture/result_json',
                'log_interval_sec': 0.5,
            }]
        ),

        Node(
            package='puppy_brain',
            executable='decision_node_gesture_test',
            name='decision_node_gesture_test',
            output='screen',
            parameters=[{
                'publish_repeat_sec': 1.0,
                'log_interval_sec': 0.2,
                'gesture_hold_sec': 0.8,
            }]
        ),

        Node(
            package='puppy_brain',
            executable='ros_udp_bridge',
            name='ros_udp_bridge',
            output='screen',
            parameters=[{
                'udp_ip': '127.0.0.1',
                'udp_port': 5005,
            }]
        ),
    ])