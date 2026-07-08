#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    websocket_node = IncludeLaunchDescription(
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

    llm_env_lang = SetEnvironmentVariable('LANG', 'C.UTF-8')
    llm_env_lc_all = SetEnvironmentVariable('LC_ALL', 'C.UTF-8')
    llm_env_language = SetEnvironmentVariable('LANGUAGE', 'C.UTF-8')

    llama_node = Node(
        package='hobot_llamacpp',
        executable='hobot_llamacpp',
        name='llama_cpp_node',
        output='screen',
        cwd='/home/sunrise/llm_run',
        parameters=[{
            'feed_type': 2,
            'llm_threads': 6,
            'cute_words': 'READY_IGNORE',
            'system_prompt': '/opt/tros/humble/lib/hobot_llamacpp/config/system_prompt.txt',
            'text_msg_pub_topic_name': '/tts_text',
            'ros_string_sub_topic_name': '/prompt_text',
            'llm_model_name': '/app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf',
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    chat_llm_bridge_node = Node(
        package='puppy_brain',
        executable='chat_llm_bridge_node',
        name='chat_llm_bridge_node',
        output='screen',
        parameters=[{
            'chat_input_topic': '/chat/input_text',
            'chat_output_topic': '/chat/response_text',
            'llm_input_topic': '/prompt_text',
            'llm_output_topic': '/tts_text',
            'flush_timeout_sec': 2.0,
        }]
    )

    return LaunchDescription([
        llm_env_lang,
        llm_env_lc_all,
        llm_env_language,

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
                'orig_width': 960,
                'orig_height': 544,
                'image_topic': '/image',
                'log_interval_sec': 5.0,
            }]
        ),

        # ====== 语音输入：I2C 语音模块（保留但默认注释，改用 USB ASR 链）======
        # Node(
        #     package='puppy_brain',
        #     executable='voice_control_node',
        #     name='voice_control_node',
        #     output='screen',
        #     parameters=[{
        #         'i2c_bus': 5,
        #         'i2c_addr': 0x79,
        #         'mode': 1,
        #         'init_words': True,
        #         'poll_interval': 0.10,
        #         'cooldown_sec': 1.5,
        #         'debug_log_interval_sec': 30.0,
        #     }]
        # ),

        # ====== 语音输入：USB 麦克风 + Vosk ASR（推荐）======
        Node(
            package='puppy_brain',
            executable='usb_asr_text_node',
            name='usb_asr_text_node',
            output='screen',
            parameters=[{
                'device': 'plughw:2,0',          # 改成你 USB 麦克风实际设备（arecord -l 查）
                'record_seconds': 3,
                'sample_rate': 16000,
                'channels': 1,
                'model_path': '/app/puppy_ws/models/vosk-model-small-cn-0.22',
                'loop_sleep_sec': 0.3,
                'min_text_length': 1,
            }]
        ),

        # ====== 语音意图路由：控制指令走 /voice/result_json，对话走 /chat/input_text ======
        Node(
            package='puppy_brain',
            executable='intent_router_node',
            name='intent_router_node',
            output='screen',
            parameters=[{
                'control_cooldown_sec': 1.5,
            }]
        ),

        Node(
            package='puppy_brain',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'image_width': 960.0,
                'image_height': 544.0,

                'follow_area_near_stop': 0.42,
                'follow_area_far_walk': 0.10,
                'min_valid_area_ratio': 0.015,

                'center_ratio': 0.50,
                'turn_deadband_ratio': 0.09,
                'max_turn_error_ratio': 0.28,
                'turn_gain': 0.85,

                'forward_min': 0.0,
                'forward_max': 0.95,

                'ghost_memory_time': 0.30,
                'publish_repeat_sec': 0.15,

                'gesture_hold_sec': 0.8,
                'follow_default_enabled': True,

                'gesture_action_lock_sec': 2.5,
                'gesture_stop_lock_sec': 1.0,

                'voice_action_lock_sec': 2.5,
                'voice_priority_enabled': True,

                'voice_move_sec': 2.5,
                'voice_forward_speed': 0.55,
                'voice_backward_speed': 0.35,
                'voice_turn_speed': 0.75,

                'control_smooth_alpha': 0.28,

                'turn_zero_threshold': 0.05,
                'forward_zero_threshold': 0.05,

                'debug_print_sec': 0.5,
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
                'imu_udp_ip': '127.0.0.1',
                'imu_udp_port': 5006,
            }]
        ),

        Node(
            package='puppy_brain',
            executable='imu_node_ros2',
            name='imu_node_ros2',
            output='screen',
            parameters=[{
                'topic_name': '/ros_robot_controller/imu_raw',
                'publish_hz': 50.0,
            }],
            arguments=['--ros-args', '--log-level', 'error']
        ),

        llama_node,
        chat_llm_bridge_node,
        websocket_node,

        # ====== 语音输出：TTS 播放节点（sherpa Matcha zh-baker 离线优先，edge-tts fallback）======
        Node(
            package='puppy_brain',
            executable='tts_play_node',
            name='tts_play_node',
            output='screen',
            parameters=[{
                'backend': 'auto',            # 'auto' 优先离线 sherpa，失败 fallback edge
                'play_device': 'plughw:1,0',  # 改成你 USB 音响实际设备（aplay -l 查）
                'cache_dir': '/tmp/tts_cache',
                'flush_timeout_sec': 0.8,
                'volume_db': '+2',            # 音量增益 dB
                # sherpa Matcha zh-baker（对应 setup_sherpa.sh 安装位置）
                'sherpa_model_root': '/opt/sherpa-models',
                'sherpa_tts_subdir': 'matcha-zh-baker',
                'sherpa_vocoder_subdir': 'vocoder',
                'sherpa_vocoder_file': 'hifigan_v2.onnx',
                'sherpa_speed': 1.0,
                # edge-tts 兜底
                'edge_voice': 'zh-CN-XiaoxiaoNeural',
            }]
        ),
    ])