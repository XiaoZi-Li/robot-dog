#!/bin/bash
# setup_sherpa.sh - RDK X5 上一键安装 sherpa-onnx + Matcha zh-baker TTS 模型
# 对应官方 rdk-tts skill 的 sherpa_setup 工具（手动版）
# 模型装在 /opt/sherpa-models/，安装完断网可用
#
# 用法：
#   sudo bash setup_sherpa.sh              # 默认走 ghproxy 国内镜像
#   sudo bash setup_sherpa.sh --no-mirror  # 直连 GitHub（海外环境）

set -e

# ---------- 镜像开关 ----------
MIRROR="${SHERPA_MIRROR:-https://ghproxy.com/}"
if [[ "$1" == "--no-mirror" ]]; then
    MIRROR=""
fi

echo "================================================"
echo " RDK X5 sherpa-onnx 一键安装"
echo " 镜像: ${MIRROR:-直连}"
echo " 安装目录: /opt/sherpa-models"
echo "================================================"

# ---------- 1. 系统依赖 ----------
echo "[1/5] 装系统依赖..."
apt-get update -y
apt-get install -y ffmpeg alsa-utils curl wget python3-pip

# ---------- 2. Python 依赖 ----------
echo "[2/5] 装 Python 包（走清华镜像）..."
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    sherpa-onnx numpy soundfile

# ---------- 3. 准备目录 ----------
MODEL_ROOT="/opt/sherpa-models"
mkdir -p "${MODEL_ROOT}/matcha-zh-baker"
mkdir -p "${MODEL_ROOT}/vocoder"
cd /tmp

# ---------- 4. 下载 Matcha zh-baker TTS 模型 ----------
echo "[3/5] 下载 Matcha zh-baker TTS 模型（~72MB）..."
TTS_TARBALL="matcha-icefall-zh-baker.tar.bz2"
TTS_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/${TTS_TARBALL}"

if [[ -n "${MIRROR}" ]]; then
    TTS_URL="${MIRROR}${TTS_URL}"
fi

# matcha 模型解压后主文件名是 model-steps-3.onnx
if [[ ! -f "${MODEL_ROOT}/matcha-zh-baker/model-steps-3.onnx" ]]; then
    echo "  下载: ${TTS_URL}"
    wget -q --show-progress -O "${TTS_TARBALL}" "${TTS_URL}"
    tar xjf "${TTS_TARBALL}"
    # 解压后目录名是 matcha-icefall-zh-baker
    cp -r matcha-icefall-zh-baker/* "${MODEL_ROOT}/matcha-zh-baker/"
    rm -rf matcha-icefall-zh-baker "${TTS_TARBALL}"
    echo "  [OK] TTS 模型安装完成: ${MODEL_ROOT}/matcha-zh-baker/"
else
    echo "  [SKIP] 已存在，跳过"
fi

# ---------- 5. 下载 Vocoder（vocos-22khz-univ）----------
echo "[4/5] 下载 Vocoder vocos-22khz-univ（~51MB，单个 onnx 文件）..."
VOCODER_FILE="vocos-22khz-univ.onnx"
VOCODER_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/${VOCODER_FILE}"

if [[ -n "${MIRROR}" ]]; then
    VOCODER_URL="${MIRROR}${VOCODER_URL}"
fi

if [[ ! -f "${MODEL_ROOT}/vocoder/${VOCODER_FILE}" ]]; then
    echo "  下载: ${VOCODER_URL}"
    wget -q --show-progress -O "${MODEL_ROOT}/vocoder/${VOCODER_FILE}" "${VOCODER_URL}"
    echo "  [OK] Vocoder 安装完成: ${MODEL_ROOT}/vocoder/${VOCODER_FILE}"
else
    echo "  [SKIP] 已存在，跳过"
fi

# ---------- 6. 验证 ----------
echo "[5/5] 验证安装..."
echo ""
echo "模型文件清单："
ls -lh "${MODEL_ROOT}/matcha-zh-baker/" 2>/dev/null || echo "  [FAIL] TTS 目录为空"
ls -lh "${MODEL_ROOT}/vocoder/" 2>/dev/null || echo "  [FAIL] Vocoder 目录为空"
echo ""
echo "Python 包版本："
python3 -c "import sherpa_onnx; print(f'  sherpa-onnx: {sherpa_onnx.__version__}')" 2>/dev/null || echo "  [FAIL] sherpa-onnx 未装好"
python3 -c "import numpy, soundfile; print(f'  numpy/soundfile: OK')" 2>/dev/null || echo "  [FAIL] numpy/soundfile 未装好"

# ---------- 7. 合成测试 ----------
echo ""
echo "================================================"
echo " 合成测试：生成 '你好我是机器狗' 到 /tmp/test_tts.wav"
echo "================================================"
python3 << 'PYEOF'
import os, sherpa_onnx, numpy as np, soundfile as sf

model_path = '/opt/sherpa-models/matcha-zh-baker/model-steps-3.onnx'
tokens_path = '/opt/sherpa-models/matcha-zh-baker/tokens.txt'
lexicon_path = '/opt/sherpa-models/matcha-zh-baker/lexicon.txt'
vocoder_path = '/opt/sherpa-models/vocoder/vocos-22khz-univ.onnx'

# 新版 sherpa-onnx (1.10+) API: 需要嵌套 config 对象
tts_config = sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
            acoustic_model=model_path,
            vocoder=vocoder_path,
            lexicon=lexicon_path,
            tokens=tokens_path,
        ),
    ),
)
tts = sherpa_onnx.OfflineTts(tts_config)

audio = tts.generate('你好，我是机器狗')
sf.write('/tmp/test_tts.wav', np.array(audio.samples, dtype=np.float32), audio.sample_rate)
print(f'  [OK] 合成成功，采样率={audio.sample_rate}, 时长={len(audio.samples)/audio.sample_rate:.2f}s')
PYEOF

echo ""
echo "================================================"
echo " [DONE] 全部安装完成！播放测试："
echo "   aplay -D plughw:1,0 /tmp/test_tts.wav"
echo ""
echo " 如果声音不对，查设备号："
echo "   aplay -l    # 找 USB 音响 card 号"
echo "================================================"
