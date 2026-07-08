from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puppy_brain',
            executable='ros_udp_bridge',
            name='ros_udp_bridge',
            output='screen'
        ),
        Node(
            package='puppy_brain',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'image_width': 1920.0,
                'image_height': 1080.0,
                'follow_area_near_stop': 0.35,
                'follow_area_far_walk': 0.15,
                'turn_left_ratio': 0.36,
                'turn_right_ratio': 0.64,
                'ghost_memory_time': 3.0,
                'publish_debounce_sec': 0.3,
                'log_interval_sec': 0.2,
                'gesture_hold_sec': 0.8,
            }]
        ),
        Node(
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
                'orig_width': 1920,
                'orig_height': 1080,
                'camera_index': 0,
                'frame_channel': 2,
                'enable_hdmi_preview': False
            }]
        ),
    ])