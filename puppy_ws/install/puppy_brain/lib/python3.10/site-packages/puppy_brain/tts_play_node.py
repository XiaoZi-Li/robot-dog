#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TTS 播放节点（sherpa-onnx Matcha zh-baker 离线优先）

后端选择（参数 backend）：
  - 'sherpa' : sherpa-onnx + Matcha zh-baker（中文女声，官方推荐，离线 ~72MB）
  - 'edge'   : edge-tts，联网（微软），国内被墙率高，仅兜底
  - 'auto'   : 先试 sherpa，初始化失败自动 fallback 到 edge（默认）

模型默认位置（sherpa_setup 安装位置）：
  /opt/sherpa-models/
    ├── matcha-zh-baker/
    │   ├── matcha-zh-baker.onnx
    │   ├── lexicon.txt
    │   ├── tokens.txt
    │   └── dict/
    └── vocoder/
        └── hifigan_v2.onnx
"""

import asyncio
import hashlib
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import sherpa_onnx
except ImportError:
    sherpa_onnx = None

try:
    import edge_tts
except ImportError:
    edge_tts = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    import soundfile as sf
except ImportError:
    sf = None


class TtsPlayNode(Node):
    def __init__(self):
        super().__init__('tts_play_node')

        # 后端与播放参数
        self.declare_parameter('backend', 'auto')             # 'sherpa' | 'edge' | 'auto'
        self.declare_parameter('play_device', 'plughw:1,0')
        self.declare_parameter('cache_dir', '/tmp/tts_cache')
        self.declare_parameter('flush_timeout_sec', 0.8)
        self.declare_parameter('volume_db', '+2')

        # sherpa 模型路径（对应 sherpa_setup 默认安装位置）
        self.declare_parameter('sherpa_model_root', '/opt/sherpa-models')
        self.declare_parameter('sherpa_tts_subdir', 'matcha-zh-baker')
        self.declare_parameter('sherpa_vocoder_subdir', 'vocoder')
        self.declare_parameter('sherpa_vocoder_file', 'hifigan_v2.onnx')
        self.declare_parameter('sherpa_speed', 1.0)

        # edge-tts 参数（兜底）
        self.declare_parameter('edge_voice', 'zh-CN-XiaoxiaoNeural')

        self.backend_pref = str(self.get_parameter('backend').value)
        self.device = str(self.get_parameter('play_device').value)
        self.cache_dir = str(self.get_parameter('cache_dir').value)
        self.flush_timeout_sec = float(self.get_parameter('flush_timeout_sec').value)
        self.volume_db = str(self.get_parameter('volume_db').value)

        os.makedirs(self.cache_dir, exist_ok=True)

        # 初始化活跃后端
        self.active_backend = None
        self.sherpa_tts = None
        self._init_backend()

        # 订阅 + 聚合缓冲
        self.tts_sub = self.create_subscription(
            String, '/tts_text', self.on_tts_text, 10
        )
        self._lock = threading.Lock()
        self._segments = []
        self._last_segment_time = 0.0
        self._playing = False
        self._pending_queue = []

        self.flush_timer = self.create_timer(0.2, self.on_flush_timer)

    # ---------- 后端初始化 ----------
    def _init_backend(self):
        if self.backend_pref in ('sherpa', 'auto'):
            if self._init_sherpa():
                self.active_backend = 'sherpa'
                self.get_logger().info(
                    f'[TTS] backend=sherpa Matcha zh-baker (离线中文女声)'
                )
                return
            if self.backend_pref == 'sherpa':
                self.get_logger().error(
                    '[TTS] backend=sherpa 但初始化失败！请运行 setup_sherpa.sh 或改 backend=auto/edge'
                )
                return

        # fallback 到 edge
        if self._init_edge():
            self.active_backend = 'edge'
            self.get_logger().warn(
                f'[TTS] backend=edge-tts (联网兜底，国内可能被墙), voice={self.get_parameter("edge_voice").value}'
            )
        else:
            self.get_logger().error(
                '[TTS] 所有后端初始化失败！\n'
                '  sherpa: 运行 setup_sherpa.sh 安装\n'
                '  edge  : pip3 install edge-tts'
            )

    def _init_sherpa(self) -> bool:
        if sherpa_onnx is None or np is None or sf is None:
            self.get_logger().warn(
                '[sherpa] 缺依赖：pip3 install sherpa-onnx soundfile numpy'
            )
            return False

        root = str(self.get_parameter('sherpa_model_root').value)
        tts_dir = os.path.join(root, str(self.get_parameter('sherpa_tts_subdir').value))
        vocoder_dir = os.path.join(root, str(self.get_parameter('sherpa_vocoder_subdir').value))
        vocoder_file = str(self.get_parameter('sherpa_vocoder_file').value)

        # matcha 模型文件名候选（官方文档确认主文件名是 model-steps-3.onnx）
        model_candidates = [
            os.path.join(tts_dir, 'model-steps-3.onnx'),
            os.path.join(tts_dir, 'matcha-zh-baker.onnx'),
            os.path.join(tts_dir, 'model.onnx'),
        ]
        model_path = next((p for p in model_candidates if os.path.exists(p)), None)

        tokens_path = os.path.join(tts_dir, 'tokens.txt')
        lexicon_path = os.path.join(tts_dir, 'lexicon.txt')
        dict_dir = os.path.join(tts_dir, 'dict')

        # vocoder 文件名候选（官方推荐 vocos-22khz-univ.onnx）
        vocoder_candidates = [
            os.path.join(vocoder_dir, 'vocos-22khz-univ.onnx'),
            os.path.join(vocoder_dir, vocoder_file),
            os.path.join(vocoder_dir, 'hifigan_v2.onnx'),
            os.path.join(vocoder_dir, 'hifigan_v1.onnx'),
        ]
        vocoder_path = next((p for p in vocoder_candidates if os.path.exists(p)), None)

        if not model_path:
            self.get_logger().warn(
                f'[sherpa] Matcha 模型不存在: {tts_dir}/model-steps-3.onnx\n'
                f'         请运行 setup_sherpa.sh 或 sherpa_setup 工具'
            )
            return False
        if not vocoder_path:
            self.get_logger().warn(
                f'[sherpa] Vocoder 不存在: {vocoder_dir}/vocos-22khz-univ.onnx'
            )
            return False
        if not os.path.exists(tokens_path):
            self.get_logger().warn(f'[sherpa] tokens.txt 不存在: {tokens_path}')
            return False

        lexicon_val = lexicon_path if os.path.exists(lexicon_path) else ''
        speed_val = float(self.get_parameter('sherpa_speed').value)

        # 新版 sherpa-onnx (1.10+) API: 需要嵌套 config 对象
        try:
            tts_config = sherpa_onnx.OfflineTtsConfig(
                model=sherpa_onnx.OfflineTtsModelConfig(
                    matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                        acoustic_model=model_path,
                        vocoder=vocoder_path,
                        lexicon=lexicon_val,
                        tokens=tokens_path,
                    ),
                ),
            )
            self.sherpa_tts = sherpa_onnx.OfflineTts(tts_config)
            self.get_logger().info(
                f'[sherpa] 模型加载成功: matcha={os.path.basename(model_path)}, '
                f'vocoder={os.path.basename(vocoder_path)}'
            )
            return True
        except Exception as e:
            self.get_logger().error(
                f'[sherpa] 初始化异常: {repr(e)}'
            )
            return False

    def _init_edge(self) -> bool:
        if edge_tts is None:
            self.get_logger().warn('[edge] 缺依赖：pip3 install edge-tts')
            return False
        return True

    # ---------- 文本接收 + 聚合 ----------
    def on_tts_text(self, msg: String):
        text = msg.data.strip()
        if not text or text == 'READY_IGNORE':
            return
        with self._lock:
            self._segments.append(text)
            self._last_segment_time = time.time()

    def on_flush_timer(self):
        now = time.time()
        with self._lock:
            if not self._segments or self._last_segment_time <= 0.0:
                return
            if (now - self._last_segment_time) < self.flush_timeout_sec:
                return
            merged = ''.join(s.strip() for s in self._segments if s.strip())
            self._segments = []
            self._last_segment_time = 0.0

        if not merged:
            return
        self._pending_queue.append(merged)
        self.try_play_next()

    def try_play_next(self):
        if self._playing or not self._pending_queue:
            return
        text = self._pending_queue.pop(0)
        self._playing = True
        threading.Thread(
            target=self._play_blocking, args=(text,), daemon=True
        ).start()

    # ---------- 合成 + 播放 ----------
    def _play_blocking(self, text: str):
        try:
            if self.active_backend is None:
                self.get_logger().warn(f'[TTS] 无可用后端，跳过: "{text}"')
                return

            wav_path = self._cache_path(text, '.wav')

            if not os.path.exists(wav_path):
                if self.active_backend == 'sherpa':
                    self._synthesize_sherpa(text, wav_path)
                else:
                    self._synthesize_edge(text, wav_path)

            subprocess.run(
                ['aplay', '-D', self.device, wav_path],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(
                f'[TTS:{self.active_backend}] played: "{text}"'
            )

        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f'[TTS] aplay 失败 rc={e.returncode}，检查设备 {self.device}'
            )
        except Exception as e:
            self.get_logger().error(f'[TTS] 播放失败: {repr(e)}')
        finally:
            self._playing = False
            self.try_play_next()

    def _synthesize_sherpa(self, text: str, wav_path: str):
        """sherpa-onnx Matcha zh-baker 离线合成"""
        audio = self.sherpa_tts.generate(text)
        samples = audio.samples
        sr = audio.sample_rate

        if np is None or sf is None:
            raise RuntimeError('sherpa 合成需要 numpy + soundfile')

        samples_np = np.array(samples, dtype=np.float32)

        # 用 ffmpeg 应用音量增益并输出 16-bit wav
        tmp_path = wav_path + '.raw.wav'
        sf.write(tmp_path, samples_np, sr, subtype='FLOAT')
        subprocess.run(
            ['ffmpeg', '-y', '-i', tmp_path,
             '-af', f'volume={self.volume_db}dB',
             '-ar', '44100', '-ac', '1', wav_path],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            os.remove(tmp_path)
        except OSError:
            pass

    def _synthesize_edge(self, text: str, wav_path: str):
        """edge-tts 联网合成，先下 mp3 再转 wav"""
        mp3_path = self._cache_path(text, '.mp3')
        if not os.path.exists(mp3_path):
            asyncio.run(self._edge_save(text, mp3_path))
        subprocess.run(
            ['ffmpeg', '-y', '-i', mp3_path,
             '-af', f'volume={self.volume_db}dB',
             '-ar', '44100', '-ac', '1', wav_path],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    async def _edge_save(self, text: str, mp3_path: str):
        voice = str(self.get_parameter('edge_voice').value)
        communicate = edge_tts.Communicate(text, voice)
        await communicate.save(mp3_path)

    def _cache_path(self, text: str, ext: str) -> str:
        h = hashlib.md5(text.encode('utf-8')).hexdigest()[:16]
        return os.path.join(self.cache_dir, f'{h}{ext}')


def main(args=None):
    rclpy.init(args=args)
    node = TtsPlayNode()
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
