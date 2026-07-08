#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_llm_node.py - 云端 LLM 节点 (DeepSeek API)

接口完全兼容 hobot_llamacpp:
  - 订阅 /prompt_text (String)  -- chat_llm_bridge_node 转发来的用户输入
  - 发布 /tts_text (String)     -- LLM 回复片段, chat_llm_bridge_node 合并后发 /chat/response_text

替换关系:
  full_system.launch.py 中, 注释掉 llama_node, 换成 cloud_llm_node 即可
  其他节点 (chat_llm_bridge_node / tts_play_node / intent_router_node) 不用改

依赖:
  pip3 install requests

启动:
  export DEEPSEEK_API_KEY="sk-xxx"
  ros2 run puppy_brain cloud_llm_node --ros-args -p api_key:="sk-xxx"

特点:
  - 流式输出 (SSE), 边生成边发 /tts_text, TTS 可以边播
  - 板端 CPU 占用 <1% (只是 HTTP 请求)
  - 国内直连, 不墙
  - 支持多轮上下文 (保留最近 N 轮)
"""
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import requests
except ImportError:
    requests = None
    print('[cloud_llm_node] ERROR: requests 未安装, 请执行: pip3 install requests')


# 默认系统提示词 (机器狗人设)
DEFAULT_SYSTEM_PROMPT = """你是 PuppyPi, 一只基于 RDK X5 开发板的四足机器狗。
你正在通过语音和用户对话。请遵守:
1. 回答简洁, 每次回复不超过 50 字, 因为要通过 TTS 播放
2. 用口语化中文, 不要用书面语
3. 不要用 markdown、表情符号、标点堆叠
4. 如果用户要执行动作 (坐下/站立/前进等), 提示用户直接说动作指令"""


class CloudLlmNode(Node):
    def __init__(self):
        super().__init__('cloud_llm_node')

        # ============ 参数 ============
        self.declare_parameter('api_key', '')
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        self.declare_parameter('model', 'deepseek-chat')
        self.declare_parameter('system_prompt', '')           # 直接传字符串
        self.declare_parameter('system_prompt_file', '')      # 或传文件路径
        self.declare_parameter('max_tokens', 200)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_history', 6)              # 保留最近几轮对话
        self.declare_parameter('sub_topic', '/prompt_text')
        self.declare_parameter('pub_topic', '/tts_text')
        self.declare_parameter('request_timeout', 30.0)
        self.declare_parameter('cute_words', '')              # 兼容 hobot_llamacpp 的启动词, 忽略

        # API key: 参数 > 环境变量
        self.api_key = str(self.get_parameter('api_key').value) or os.environ.get('DEEPSEEK_API_KEY', '')
        self.base_url = str(self.get_parameter('base_url').value).rstrip('/')
        self.model = str(self.get_parameter('model').value)
        self.max_tokens = int(self.get_parameter('max_tokens').value)
        self.temperature = float(self.get_parameter('temperature').value)
        self.max_history = int(self.get_parameter('max_history').value)
        self.sub_topic = str(self.get_parameter('sub_topic').value)
        self.pub_topic = str(self.get_parameter('pub_topic').value)
        self.request_timeout = float(self.get_parameter('request_timeout').value)

        # system prompt: 参数字符串 > 文件 > 默认
        sp = str(self.get_parameter('system_prompt').value)
        sp_file = str(self.get_parameter('system_prompt_file').value)
        if sp:
            self.system_prompt = sp
        elif sp_file and os.path.isfile(sp_file):
            try:
                with open(sp_file, 'r', encoding='utf-8') as f:
                    self.system_prompt = f.read().strip()
            except Exception:
                self.system_prompt = DEFAULT_SYSTEM_PROMPT
        else:
            self.system_prompt = DEFAULT_SYSTEM_PROMPT

        # ============ 状态 ============
        self.history = []                  # [{"role":"user","content":"..."}, {"role":"assistant","content":"..."}]
        self.history_lock = threading.Lock()
        self.busy = False                  # 正在请求中, 防并发

        # ============ ROS2 ============
        self.llm_pub = self.create_publisher(String, self.pub_topic, 10)
        self.create_subscription(String, self.sub_topic, self.on_prompt, 10)

        if not self.api_key:
            self.get_logger().error(
                'DEEPSEEK_API_KEY 未设置! 请 export DEEPSEEK_API_KEY=sk-xxx 或 launch 参数 api_key:='
            )
        else:
            self.get_logger().info(
                f'cloud_llm_node started. model={self.model}, base={self.base_url}, '
                f'history={self.max_history} turns, sub={self.sub_topic}, pub={self.pub_topic}'
            )

    # -----------------------------------------------------
    # 收到用户输入 -> 调 DeepSeek 流式 API
    # -----------------------------------------------------
    def on_prompt(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if text == 'READY_IGNORE':
            # 兼容 hobot_llamacpp 的启动词, 忽略
            return
        if not self.api_key:
            self.get_logger().error('无 API key, 跳过')
            return
        if self.busy:
            self.get_logger().warn(f'上一轮还没返回, 丢弃: {text[:30]}')
            return

        self.busy = True
        # 在独立线程跑, 不阻塞 ROS2 回调
        threading.Thread(target=self._call_llm, args=(text,), daemon=True).start()

    # -----------------------------------------------------
    # 调 DeepSeek (流式 SSE)
    # -----------------------------------------------------
    def _call_llm(self, user_text: str):
        url = f'{self.base_url}/v1/chat/completions'
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json',
        }

        # 构造消息: system + 历史 + 当前
        with self.history_lock:
            messages = [{'role': 'system', 'content': self.system_prompt}]
            messages.extend(self.history)
            messages.append({'role': 'user', 'content': user_text})

        payload = {
            'model': self.model,
            'messages': messages,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
            'stream': True,           # 流式输出
        }

        self.get_logger().info(f'[LLM] 请求: "{user_text[:40]}"')

        full_reply = ''
        chunk_buf = ''
        chunk_send_interval = 0.3     # 攒够 ~10 字或 0.3s 发一次, 减少 ROS2 pub 频率
        last_send_time = time.time()

        try:
            if requests is None:
                self.get_logger().error('requests 未安装')
                self._publish_reply('大模型模块未安装, 请联系管理员')
                return

            resp = requests.post(
                url, headers=headers, json=payload,
                stream=True, timeout=self.request_timeout,
            )
            if resp.status_code != 200:
                err = resp.text[:200]
                self.get_logger().error(f'API 错误 {resp.status_code}: {err}')
                self._publish_reply('抱歉, 我现在没法回答, 稍后再试')
                return

            # 解析 SSE 流: 每行 data: {...}
            for line in resp.iter_lines(decode_unicode=True):
                if not line:
                    continue
                if not line.startswith('data:'):
                    continue
                data_str = line[5:].strip()
                if data_str == '[DONE]':
                    break
                try:
                    chunk = json.loads(data_str)
                    delta = chunk.get('choices', [{}])[0].get('delta', {})
                    token = delta.get('content', '')
                except Exception:
                    continue
                if not token:
                    continue

                full_reply += token
                chunk_buf += token

                # 攒一段发一次 (避免每个 token 都 pub, 太碎)
                now = time.time()
                if len(chunk_buf) >= 10 or (now - last_send_time) > chunk_send_interval:
                    if chunk_buf:
                        self._publish_reply(chunk_buf)
                        chunk_buf = ''
                        last_send_time = now

            # 收尾: 把剩余的 buffer 发出去
            if chunk_buf:
                self._publish_reply(chunk_buf)

            # 完整回复入历史
            if full_reply:
                with self.history_lock:
                    self.history.append({'role': 'user', 'content': user_text})
                    self.history.append({'role': 'assistant', 'content': full_reply})
                    # 保留最近 max_history 轮 (1 轮 = user + assistant = 2 条)
                    max_msgs = self.max_history * 2
                    if len(self.history) > max_msgs:
                        self.history = self.history[-max_msgs:]

            self.get_logger().info(f'[LLM] 回复完成 ({len(full_reply)} 字): "{full_reply[:40]}"')

        except requests.exceptions.Timeout:
            self.get_logger().error('LLM 请求超时')
            self._publish_reply('我思考太久了, 再问一次吧')
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f'网络错误: {e}')
            self._publish_reply('网络连不上, 检查网络')
        except Exception as e:
            self.get_logger().error(f'LLM 异常: {e}')
            self._publish_reply('出错了, 稍后再试')
        finally:
            self.busy = False

    def _publish_reply(self, text: str):
        if not text:
            return
        msg = String()
        msg.data = text
        self.llm_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CloudLlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
