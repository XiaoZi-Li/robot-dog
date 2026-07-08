#!/bin/bash
# start_all.sh - 机器狗全系统一键启动脚本
# 整合四条独立启动链, 按依赖顺序拉起:
#   1. /app/start_robot.sh start                    (sit.py 运动中枢 + IMU)
#   2. /app/gs130w_stereo/scripts/start_v2.sh start (GS130W 双目 + 立体深度)
#   3. /app/gs130w_stereo/scripts/start_avoidance.sh start (双目深度避障)
#   4. python3 voice_control_standalone.py          (纯Python语音控制)
#
# 用法:
#   ./start_all.sh start    全量启动
#   ./start_all.sh stop     全量停止
#   ./start_all.sh restart  重启
#   ./start_all.sh status   查看各链状态
#   ./start_all.sh start no_voice      启动但不启语音
#   ./start_all.sh start no_avoidance 启动但不启避障(用于切回LLM手势控制)
set -u

APP_DIR="/app"
PUPPY_WS="$APP_DIR/puppy_ws"
VOICE_SCRIPT="$PUPPY_WS/tools/voice_control_standalone.py"
LOG_DIR="/tmp/start_all"
VOICE_PID_FILE="$LOG_DIR/voice.pid"

# 语音控制默认参数 (按用户实际硬件: mic=plughw:1,0, speaker=plughw:0,0)
VOICE_MIC="plughw:1,0"
VOICE_SPEAKER="plughw:0,0"
VOICE_GAIN=10
VOICE_AGGRESSIVENESS=2
VOICE_SILENCE=1.0

mkdir -p "$LOG_DIR"

# ============ 启动 ============
start_all() {
    local skip_voice=0
    local skip_avoidance=0
    [ "${2:-}" = "no_voice" ] && skip_voice=1
    [ "${2:-}" = "no_avoidance" ] && skip_avoidance=1

    echo "================================================"
    echo " 机器狗全系统启动 (开始)"
    echo "================================================"
    echo " 语音:     $([ $skip_voice -eq 1 ] && echo '跳过' || echo '启动')"
    echo " 避障:     $([ $skip_avoidance -eq 1 ] && echo '跳过' || echo '启动')"
    echo "================================================"

    # ---- 链1: 底层运动中枢 + IMU (必须最先) ----
    echo ""
    echo "[1/4] 启动底层: start_robot.sh start"
    bash "$APP_DIR/start_robot.sh" start || { echo "[ERR] 底层启动失败, 中止"; exit 1; }
    sleep 2

    # ---- 链2: GS130W 双目视觉 + 立体深度 ----
    echo ""
    echo "[2/4] 启动双目视觉: start_v2.sh start"
    bash "$APP_DIR/gs130w_stereo/scripts/start_v2.sh" start || echo "[WARN] 双目启动异常, 继续"
    sleep 3

    # ---- 链3: 双目深度避障 (依赖链1+链2) ----
    if [ $skip_avoidance -eq 0 ]; then
        echo ""
        echo "[3/4] 启动避障: start_avoidance.sh start"
        bash "$APP_DIR/gs130w_stereo/scripts/start_avoidance.sh" start || echo "[WARN] 避障启动异常, 继续"
    else
        echo "[3/4] 避障跳过 (no_avoidance)"
    fi
    sleep 1

    # ---- 链4: 纯Python语音控制 (依赖链1) ----
    if [ $skip_voice -eq 0 ]; then
        echo ""
        echo "[4/4] 启动语音控制: voice_control_standalone.py"
        if [ -f "$VOICE_SCRIPT" ]; then
            python3 "$VOICE_SCRIPT" \
                --mic "$VOICE_MIC" \
                --speaker "$VOICE_SPEAKER" \
                --gain "$VOICE_GAIN" \
                --aggressiveness "$VOICE_AGGRESSIVENESS" \
                --silence "$VOICE_SILENCE" \
                > "$LOG_DIR/voice.log" 2>&1 &
            echo $! > "$VOICE_PID_FILE"
            echo "[4/4] 语音控制已启动 (PID=$(cat $VOICE_PID_FILE))"
        else
            echo "[WARN] 语音脚本不存在: $VOICE_SCRIPT"
        fi
    else
        echo "[4/4] 语音跳过 (no_voice)"
    fi

    sleep 2
    echo ""
    echo "================================================"
    echo " 全系统启动完成"
    echo "================================================"
    BOARD_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    [ -z "$BOARD_IP" ] && BOARD_IP="<板端IP>"
    echo " 双目Web:   http://$BOARD_IP:8090/view.html"
    echo " 左眼:       http://$BOARD_IP:8072"
    echo " 右眼:       http://$BOARD_IP:8071"
    echo " 深度图:     http://$BOARD_IP:8073"
    echo " 避障状态:   ros2 topic echo /stereo_avoidance/status"
    echo " 语音日志:   $LOG_DIR/voice.log"
    echo " 全部停止:   $0 stop"
    echo "================================================"
}

# ============ 停止 ============
stop_all() {
    echo "[STOP] 停止全系统..."

    # 反向顺序停止
    if [ -f "$VOICE_PID_FILE" ]; then
        kill "$(cat $VOICE_PID_FILE)" 2>/dev/null
        rm -f "$VOICE_PID_FILE"
        echo "[STOP] 语音控制已停止"
    fi
    pkill -f 'voice_control_standalone.py' 2>/dev/null

    bash "$APP_DIR/gs130w_stereo/scripts/start_avoidance.sh" stop 2>/dev/null && echo "[STOP] 避障已停止"
    bash "$APP_DIR/gs130w_stereo/scripts/start_v2.sh" stop 2>/dev/null && echo "[STOP] 双目已停止"
    bash "$APP_DIR/start_robot.sh" stop 2>/dev/null && echo "[STOP] 底层已停止"

    echo "[STOP] 全系统已停止"
}

# ============ 状态 ============
status_all() {
    echo "================================================"
    echo " 全系统状态  $(date '+%Y-%m-%d %H:%M:%S')"
    echo "================================================"
    echo ""
    echo "[链1] 底层运动 + IMU:"
    bash "$APP_DIR/start_robot.sh" status 2>/dev/null || echo "  脚本未找到"
    echo ""
    echo "[链2] 双目视觉:"
    bash "$APP_DIR/gs130w_stereo/scripts/start_v2.sh" status 2>/dev/null || echo "  脚本未找到"
    echo ""
    echo "[链3] 避障:"
    bash "$APP_DIR/gs130w_stereo/scripts/start_avoidance.sh" status 2>/dev/null || echo "  脚本未找到"
    echo ""
    echo "[链4] 语音控制:"
    if [ -f "$VOICE_PID_FILE" ] && kill -0 "$(cat $VOICE_PID_FILE)" 2>/dev/null; then
        echo "  [OK]   voice  pid=$(cat $VOICE_PID_FILE)"
    else
        echo "  [DEAD] voice 未运行"
    fi
    echo "================================================"
}

case "${1:-}" in
    start)   start_all "$@" ;;
    stop)    stop_all ;;
    restart) stop_all; sleep 2; start_all "$@" ;;
    status)  status_all ;;
    *)       echo "用法: $0 {start|stop|restart|status} [no_voice|no_avoidance]"; exit 1 ;;
esac
