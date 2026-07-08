#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""minimal_llm.launch.py - 最小 LLM 测试 (板载 Qwen2.5-0.5B)

只起 3 个节点, 不起摄像头/AI/双目, 专注测对话效果:
  1. hobot_llamacpp  (板载 0.5B LLM, CPU 跑)
  2. chat_llm_bridge (片段合并)
  3. tts_play        (语音播报, 可选)
  4. ws_bridge        (上位机能看到对话, 可选)

用法:
  source /opt/tros/humble/setup.bash
  source /app/puppy_ws/install/setup.bash
  ros2 launch puppy_brain minimal_llm.launch.py

测试:
  # 终端 A: 发一条聊天
  ros2 topic pub --once /chat/input_text std_msgs/String "data: 你好,你叫什么名字"

  # 终端 B: 看回复
  ros2 topic echo /chat/response_text
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    env_lang = SetEnvironmentVariable('LANG', 'C.UTF-8')
    env_lc_all = SetEnvironmentVariable('LC_ALL', 'C.UTF-8')
    env_language = SetEnvironmentVariable('LANGUAGE', 'C.UTF-8')

    # ============ 板载 LLM (Qwen2.5-0.5B) ============
    llama_node = Node(
        package='hobot_llamacpp',
        executable='hobot_llamacpp',
        name='llama_cpp_node',
        output='screen',
        cwd='/home/sunrise/llm_run',
        parameters=[{
            'feed_type': 2,
            'llm_threads': 6,                # 用 6 核 (留 2 核给系统)
            'cute_words': 'READY_IGNORE',
            'system_prompt': '/opt/tros/humble/lib/hobot_llamacpp/config/system_prompt.txt',
            'text_msg_pub_topic_name': '/tts_text',
            'ros_string_sub_topic_name': '/prompt_text',
            'llm_model_name': '/app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf',
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # ============ LLM 桥接 (片段合并) ============
    chat_bridge = Node(
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

    # ============ TTS 播报 (可选, 不想听声音可注释掉) ============
    tts_node = Node(
        package='puppy_brain',
        executable='tts_play_node',
        name='tts_play_node',
        output='screen',
        parameters=[{
            'backend': 'auto',
            'play_device': 'plughw:1,0',
            'cache_dir': '/tmp/tts_cache',
            'flush_timeout_sec': 0.8,
            'volume_db': '+2',
            'sherpa_model_root': '/opt/sherpa-models',
            'sherpa_tts_subdir': 'matcha-zh-baker',
            'sherpa_vocoder_subdir': 'vocoder',
            'sherpa_vocoder_file': 'hifigan_v2.onnx',
            'sherpa_speed': 1.0,
            'edge_voice': 'zh-CN-XiaoxiaoNeural',
        }]
    )

    # ============ 上位机 WS 桥 (可选, 让上位机能看到对话) ============
    ws_bridge = Node(
        package='puppy_brain',
        executable='ws_bridge_node',
        name='ws_bridge_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        env_lang, env_lc_all, env_language,
        llama_node,
        chat_bridge,
        tts_node,
        ws_bridge,
    ])
