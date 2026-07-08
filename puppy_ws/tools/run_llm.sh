#!/usr/bin/env bash
set -e

export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export LANGUAGE=C.UTF-8

cd /home/sunrise/llm_run

source /opt/tros/humble/setup.bash
source /app/puppy_ws/install/setup.bash

exec ros2 run hobot_llamacpp hobot_llamacpp --ros-args \
  -p feed_type:=2 \
  -p llm_threads:=6 \
  -p cute_words:=hello \
  -p system_prompt:=/opt/tros/humble/lib/hobot_llamacpp/config/system_prompt.txt \
  -p text_msg_pub_topic_name:=/tts_text \
  -p ros_string_sub_topic_name:=/prompt_text \
  -p llm_model_name:=/app/puppy_ws/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf