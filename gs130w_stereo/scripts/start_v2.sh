#!/bin/bash
# GS130W v2 一键启动（对齐官方文档 + rdk-x5 skill 最佳实践）
# 用法：
#   ./start_v2.sh start    启动全部
#   ./start_v2.sh stop     停止全部
#   ./start_v2.sh restart  重启
#   ./start_v2.sh status   查看状态
#   ./start_v2.sh logs [name]  查看某进程日志
set -u

PROJECT_ROOT="/app/gs130w_stereo"
TROS_SETUP="/opt/tros/humble/setup.bash"
GDC_BIN="/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin"
CALIB_YAML="/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml"

LOG_DIR="/tmp/gs130w_v2"
PID_FILE="$LOG_DIR/pids"

# mjpeg_bridge 端口（与 view.html 一致）
PORT_RIGHT=8071
PORT_LEFT=8072
PORT_DEPTH=8073
PORT_HTTP=8090

# 进程名（按启动顺序，用于 status / logs）
NAMES=(mipi_cam camera_info ai_v2 mjpeg_left mjpeg_right mjpeg_depth http)

check_env() {
    [ -f "$TROS_SETUP" ] || { echo "[ERR] TROS 缺失: $TROS_SETUP"; exit 1; }
    [ -f "$GDC_BIN" ] || { echo "[ERR] GDC bin 缺失: $GDC_BIN"; exit 1; }
    [ -f "$CALIB_YAML" ] || { echo "[ERR] 标定 yaml 缺失: $CALIB_YAML"; exit 1; }
    [ -f "$PROJECT_ROOT/launch/gs130w_ai_overlay_v2.launch.py" ] || { echo "[ERR] v2 launch 缺失"; exit 1; }
    [ -f "$PROJECT_ROOT/scripts/mjpeg_bridge.py" ] || { echo "[ERR] mjpeg_bridge.py 缺失"; exit 1; }
    [ -f "$PROJECT_ROOT/launch/camera_info_publisher.py" ] || { echo "[ERR] camera_info_publisher.py 缺失"; exit 1; }
    [ -f "$PROJECT_ROOT/snapshots/view.html" ] || { echo "[ERR] view.html 缺失"; exit 1; }
}

# 彻底清理：按进程名 + 端口占用，避免残留
stop_all() {
    echo "[STOP] 清理所有 gs130w v2 进程..."
    # 先按 PID 文件 kill
    if [ -f "$PID_FILE" ]; then
        while read -r pid; do
            [ -n "$pid" ] && kill "$pid" 2>/dev/null
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    # 兜底：按进程名清理（按 rdk-x5-camera skill 提示，包含 codec/websocket/cam）
    pkill -f 'mipi_cam_dual_channel' 2>/dev/null
    pkill -f 'mipi_cam_dual_channel_websocket' 2>/dev/null
    pkill -f 'camera_info_publisher.py' 2>/dev/null
    pkill -f 'gs130w_ai_overlay_v2.launch.py' 2>/dev/null
    pkill -f 'gs130w_dualcam' 2>/dev/null
    pkill -f 'mjpeg_bridge.py' 2>/dev/null
    pkill -f "http.server $PORT_HTTP" 2>/dev/null
    # 按 rdk-ros skill 提示，清理 codec/websocket 链路残留
    pkill -f 'hobot_codec' 2>/dev/null
    pkill -f 'websocket' 2>/dev/null
    pkill -f 'mipi_cam' 2>/dev/null
    sleep 1
    # 强杀端口占用
    fuser -k ${PORT_LEFT}/tcp ${PORT_RIGHT}/tcp ${PORT_DEPTH}/tcp ${PORT_HTTP}/tcp 2>/dev/null
    sleep 1
    # 清共享内存（rdk-x5-tros skill 提示）
    rm -f /dev/shm/fastrtps_* 2>/dev/null
    echo "[STOP] 完成"
}

# 等待 topic 有数据（轮询，比 sleep 可靠）
wait_topic_ready() {
    local topic="$1"
    local max_wait="${2:-30}"
    local waited=0
    while [ $waited -lt $max_wait ]; do
        if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            return 0
        fi
        sleep 1
        waited=$((waited+1))
    done
    echo "[WARN] 等待 $topic 超时（${max_wait}s）"
    return 1
}

start_all() {
    check_env
    mkdir -p "$LOG_DIR"
    : > "$PID_FILE"

    # 先清残留
    stop_all 2>/dev/null
    sleep 1

    set +u
    # shellcheck disable=SC1090
    source "$TROS_SETUP"
    set -u

    echo "[START] 1/7 mipi_cam (官方 websocket launch，自带 codec+ws) ..."
    # 用官方推荐的 mipi_cam_dual_channel_websocket.launch.py（最稳定）
    # 它会自动起 mipi_cam + hobot_codec + websocket，输出 /image_combine_jpeg 和 /sub_image_combine_jpeg
    ros2 launch mipi_cam mipi_cam_dual_channel_websocket.launch.py \
        mipi_image_width:=1280 mipi_image_height:=1088 \
        mipi_sub_image_width:=1280 mipi_sub_image_height:=1088 \
        mipi_image_framerate:=10.0 \
        mipi_io_method:=ros \
        device_mode:=dual \
        dual_combine:=2 \
        mipi_channel:=2 \
        mipi_channel2:=0 \
        mipi_lpwm_enable:=True \
        mipi_gdc_enable:=True \
        mipi_gdc_bin_file:="$GDC_BIN" \
        mipi_camera_calibration_file_path:="$CALIB_YAML" \
        mipi_rotation:=90.0 \
        mipi_cal_rotation:=0.0 \
        mipi_stream_mode:=1 \
        mipi_sub_stream_enable:=True \
        mipi_frame_ts_type:=sensor \
        > "$LOG_DIR/mipi_cam.log" 2>&1 &
    echo $! >> "$PID_FILE"

    # 等 /image_combine_jpeg 出现（轮询，最多 30 秒）
    echo "[START] 等待 mipi_cam 出图..."
    if wait_topic_ready "/image_combine_jpeg" 30; then
        echo "[START] mipi_cam 出图 OK"
    else
        echo "[WARN] mipi_cam 可能没出图，继续起后续节点（看日志定位）"
    fi
    sleep 2

    echo "[START] 2/7 camera_info_publisher ..."
    python3 "$PROJECT_ROOT/launch/camera_info_publisher.py" \
        > "$LOG_DIR/camera_info.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 1

    echo "[START] 3/7 v2 AI launch（5模型 + codec + 3 ws）..."
    ros2 launch "$PROJECT_ROOT/launch/gs130w_ai_overlay_v2.launch.py" \
        > "$LOG_DIR/ai_v2.log" 2>&1 &
    echo $! >> "$PID_FILE"

    echo "[START] 等待 AI 节点起来..."
    wait_topic_ready "/hobot_mono2d_body_detection" 30 || true
    sleep 3

    echo "[START] 4/7 mjpeg_bridge 左眼 :$PORT_LEFT ..."
    python3 "$PROJECT_ROOT/scripts/mjpeg_bridge.py" \
        --port $PORT_LEFT --topic /image_combine_jpeg --region top \
        > "$LOG_DIR/mjpeg_left.log" 2>&1 &
    echo $! >> "$PID_FILE"

    echo "[START] 5/7 mjpeg_bridge 右眼 :$PORT_RIGHT ..."
    python3 "$PROJECT_ROOT/scripts/mjpeg_bridge.py" \
        --port $PORT_RIGHT --topic /image_combine_jpeg --region bottom \
        > "$LOG_DIR/mjpeg_right.log" 2>&1 &
    echo $! >> "$PID_FILE"

    echo "[START] 6/7 mjpeg_bridge 深度图 :$PORT_DEPTH ..."
    python3 "$PROJECT_ROOT/scripts/mjpeg_bridge.py" \
        --port $PORT_DEPTH --topic /StereoNetNode/stereonet_visual_jpeg --region full \
        > "$LOG_DIR/mjpeg_depth.log" 2>&1 &
    echo $! >> "$PID_FILE"

    sleep 1

    echo "[START] 7/7 http server :$PORT_HTTP ..."
    (cd "$PROJECT_ROOT/snapshots" && python3 -m http.server $PORT_HTTP) \
        > "$LOG_DIR/http.log" 2>&1 &
    echo $! >> "$PID_FILE"

    sleep 2

    BOARD_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    [ -z "$BOARD_IP" ] && BOARD_IP="<板端IP>"

    echo ""
    echo "================================================"
    echo " GS130W v2 全部启动完成"
    echo "================================================"
    echo " 浏览器入口:  http://$BOARD_IP:$PORT_HTTP/view.html"
    echo " 右眼:        http://$BOARD_IP:$PORT_RIGHT"
    echo " 左眼:        http://$BOARD_IP:$PORT_LEFT"
    echo " 深度图:      http://$BOARD_IP:$PORT_DEPTH"
    echo " 官方 8000:   http://$BOARD_IP:8000  (mipi_cam 自带 websocket)"
    echo " 日志目录:    $LOG_DIR/"
    echo " 停止:        $0 stop"
    echo " 状态:        $0 status"
    echo " 看日志:      $0 logs <name>"
    echo "================================================"
}

status_all() {
    echo "[STATUS] gs130w v2 进程状态:"
    if [ ! -f "$PID_FILE" ]; then
        echo "  未启动（PID 文件不存在：$PID_FILE）"
        return
    fi
    i=0
    while read -r pid; do
        name="${NAMES[$i]:-unknown}"
        if kill -0 "$pid" 2>/dev/null; then
            echo "  [OK]   $name  pid=$pid"
        else
            echo "  [DEAD] $name  pid=$pid"
        fi
        i=$((i+1))
    done < "$PID_FILE"

    # 顺带探一下端口
    echo ""
    echo "[STATUS] 端口探测:"
    for p in $PORT_RIGHT $PORT_LEFT $PORT_DEPTH $PORT_HTTP; do
        if curl -sI -m 2 "http://localhost:$p/health" 2>/dev/null | head -1 | grep -q 200; then
            echo "  :$p  OK"
        elif curl -sI -m 2 "http://localhost:$p/" 2>/dev/null | head -1 | grep -q 200; then
            echo "  :$p  OK (http)"
        else
            echo "  :$p  --"
        fi
    done

    # 关键 topic 检查（必须 source TROS 才能跑 ros2 CLI）
    echo ""
    echo "[STATUS] 关键 topic:"
    set +u
    source "$TROS_SETUP" 2>/dev/null
    set -u
    for t in /image_combine_raw /image_combine_jpeg /sub_image_combine_jpeg /StereoNetNode/stereonet_visual /StereoNetNode/stereonet_visual_jpeg; do
        if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
            echo "  [OK]   $t"
        else
            echo "  [MISS] $t"
        fi
    done
}

show_logs() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        echo "可用日志: ${NAMES[*]}"
        echo "用法: $0 logs <name>"
        return
    fi
    local f="$LOG_DIR/$name.log"
    if [ ! -f "$f" ]; then
        echo "[ERR] 日志不存在: $f"
        echo "可用: ${NAMES[*]}"
        return
    fi
    echo "=== tail -f $f ==="
    tail -f "$f"
}

case "${1:-}" in
    start)   start_all ;;
    stop)    stop_all ;;
    restart) stop_all; start_all ;;
    status)  status_all ;;
    logs)    show_logs "${2:-}" ;;
    *)       echo "用法: $0 {start|stop|restart|status|logs [name]}"; exit 1 ;;
esac
