from setuptools import setup
from glob import glob

package_name = 'puppy_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rdk',
    maintainer_email='rdk@todo.todo',
    description='Puppy brain package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = puppy_brain.perception_node:main',
            'decision_node = puppy_brain.decision_node:main',
            'ros_udp_bridge = puppy_brain.ros_udp_bridge:main',
            'gesture_adapter_node = puppy_brain.gesture_adapter_node:main',
            'decision_node_gesture_test = puppy_brain.decision_node_gesture_test:main',
            'debug_preview_node = puppy_brain.debug_preview_node:main',
            'imu_node_ros2 = puppy_brain.imu_node_ros2:main',
            'voice_control_node = puppy_brain.voice_control_node:main',
            'chat_llm_bridge_node = puppy_brain.chat_llm_bridge_node:main',
            'intent_router_node = puppy_brain.intent_router_node:main',
            'usb_asr_text_node = puppy_brain.usb_asr_text_node:main',
            'tts_play_node = puppy_brain.tts_play_node:main',
            'ws_bridge_node = puppy_brain.ws_bridge_node:main',
            'cloud_llm_node = puppy_brain.cloud_llm_node:main',
            # USB 摄像头手势控制链路
            'usb_cam_publisher_node = puppy_brain.usb_cam_publisher_node:main',
            'gesture_action_node = puppy_brain.gesture_action_node:main',
            'yolov5_mjpeg_server = puppy_brain.yolov5_mjpeg_server:main',
        ],
    },
)