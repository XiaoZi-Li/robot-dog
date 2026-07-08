#!/bin/bash
# GS130W 双目视觉启动脚本（按官方 quick_start 流程）
# 用法：start_gs130w.sh {dualcam|ai_clean|ai_full|ai_overlay}
set -e

PROFILE=${1:-dualcam}
PROJECT_ROOT="/app/gs130w_stereo"
TROS_SETUP="/opt/tros/humble/setup.bash"

# 0. 基础检查
if [ ! -f "$TROS_SETUP" ]; then
    echo "[ERR] TROS 源不存在: $TROS_SETUP"
    exit 1
fi

source "$TROS_SETUP"

case "$PROFILE" in
    dualcam)
        LAUNCH_FILE="$PROJECT_ROOT/launch/gs130w_dualcam.launch.py"
        ;;
    ai_clean)
        LAUNCH_FILE="$PROJECT_ROOT/launch/gs130w_ai_clean.launch.py"
        ;;
    ai_full)
        LAUNCH_FILE="$PROJECT_ROOT/launch/gs130w_ai_full.launch.py"
        ;;
    ai_overlay)
        LAUNCH_FILE="$PROJECT_ROOT/launch/gs130w_ai_overlay.launch.py"
        ;;
    *)
        echo "[ERR] 未知 profile: $PROFILE（可选: dualcam | ai_clean | ai_full | ai_overlay）"
        exit 2
        ;;
esac

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "[ERR] 找不到 launch 文件: $LAUNCH_FILE"
    exit 3
fi

echo "[INFO] 启动 profile=$PROFILE"
echo "[INFO] launch = $LAUNCH_FILE"
echo "[INFO] TROS 已 source（version=$(dpkg -s tros-humble 2>/dev/null | awk '/^Version:/ {print $2}' || echo unknown)）"
echo "[INFO] 浏览器入口：http://$(hostname -I | awk '{print $1}'):8000"
echo ""

# 直接调本地 launch 文件（不是包内 launch）
# 因为是本地路径，用 launch 文件的绝对路径直接传给 ros2 launch
exec ros2 launch "$LAUNCH_FILE"
