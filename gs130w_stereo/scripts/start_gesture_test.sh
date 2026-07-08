#!/bin/bash
# start_gesture_test.sh - 轻量手势识别测试 (不起 stereonet 深度, 省 BPU)
#
# 只起: mipi_cam双目 + camera_info + mono2d + hand_lmk + hand_gesture
#       + gesture_adapter + 1个mjpeg看图
# 不起: stereonet / stereonet_codec / face_lmk / 3个websocket / mjpeg_depth / http
#
# 用法:
#   ./start_gesture_test.sh start    启动
#   ./start_gesture_test.sh stop     停止
#   ./start_gesture_test.sh status   状态
#   ./start_gesture_test.sh logs     实时日志
set -u

PROJECT_ROOT="/app/gs130w_stereo"
TROS_SETUP="/opt/tros/humble/setup.bash"
WS_SETUP="/app/puppy_ws/install/setup.bash"
GDC_BIN="/root/multimedia_samples/vp_sensors/gdc_bin/sc132gs_1088X1280_gdc.bin"
CALIB_YAML="/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml"

LOG_DIR="/tmp/gesture_test"
PID_FILE="$LOG_DIR/pids"
NAMES=(mipi_cam camera_info mono2d hand_lmk hand_gesture gesture_adapter mjpeg)

PORT_VIEW=8072   # 左眼预览

check_env() {
    [ -f "$TROS_SETUP" ] || { echo "[ERR] TROS 缺失"; exit 1; }
    [ -f "$GDC_BIN" ] || { echo "[ERR] GDC bin 缺失"; exit 1; }
    [ -f "$CALIB_YAML" ] || { echo "[ERR] 标定 yaml 缺失"; exit 1; }
    [ -f "$WS_SETUP" ] || { echo "[ERR] puppy_ws 未 build, 请先: colcon build --packages-select puppy_brain"; exit 1; }
}

stop_all() {
    echo "[STOP] 清理手势测试进程..."
    if [ -f "$PID_FILE" ]; then
        while read -r pid; do
            [ -n "$pid" ] && kill "$pid" 2>/dev/null
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    # 兜底: 按进程名清 (只清手势链路, 不动 stereonet/face_lmk)
    pkill -f 'mipi_cam_dual_channel' 2>/dev/null
    pkill -f 'camera_info_publisher.py' 2>/dev/null
    pkill -f 'mono2d_body_detection' 2>/dev/null
    pkill -f 'hand_lmk_detection' 2>/dev/null
    pkill -f 'hand_gesture_detection' 2>/dev/null
    pkill -f 'gesture_adapter_node' 2>/dev/null
    pkill -f 'mjpeg_bridge.py' 2>/dev/null
    sleep 1
    fuser -k ${PORT_VIEW}/tcp 2>/dev/null
    rm -f /dev/shm/fastrtps_* 2>/dev/null
    echo "[STOP] 完成"
}

wait_topic() {
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
    echo "[WARN] 等待 $topic 超时 (${max_wait}s)"
    return 1
}

start_all() {
    check_env
    mkdir -p "$LOG_DIR"
    : > "$PID_FILE"
    stop_all 2>/dev/null
    sleep 1

    set +u
    source "$TROS_SETUP"
    source "$WS_SETUP"
    set -u

    # 1. mipi_cam 双目 (只起相机+codec, 官方 launch 自带 websocket:8000 但浏览器看不了, 忽略)
    echo "[START] 1/7 mipi_cam 双目..."
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

    echo "[START] 等待双目出图..."
    wait_topic "/sub_image_combine_raw" 30 || true
    sleep 2

    # 2. camera_info (stereonet 要, 手势链路不强依赖, 但保留避免警告)
    echo "[START] 2/7 camera_info_publisher..."
    python3 "$PROJECT_ROOT/launch/camera_info_publisher.py" \
        > "$LOG_DIR/camera_info.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 1

    # 3. mono2d (人体/手部检测, 订阅校正后子流)
    echo "[START] 3/7 mono2d_body_detection..."
    ros2 run mono2d_body_detection mono2d_body_detection --ros-args \
        -p model_file_name:=/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm \
        -p model_type:=0 \
        -p is_shared_mem_sub:=0 \
        -p ros_img_topic_name:=/sub_image_combine_raw \
        -p ai_msg_pub_topic_name:=/hobot_mono2d_body_detection \
        -p is_sync_mode:=0 \
        -p image_gap:=1 \
        -p dump_render_img:=0 \
        --log-level warn \
        > "$LOG_DIR/mono2d.log" 2>&1 &
    echo $! >> "$PID_FILE"

    echo "[START] 等待 mono2d 出结果..."
    wait_topic "/hobot_mono2d_body_detection" 20 || true
    sleep 1

    # 4. hand_lmk (手部21关键点, 订阅 mono2d 的 hand 输出)
    echo "[START] 4/7 hand_lmk_detection..."
    ros2 run hand_lmk_detection hand_lmk_detection --ros-args \
        -p model_file_name:=/opt/tros/humble/lib/hand_lmk_detection/config/handLMKs.hbm \
        -p is_shared_mem_sub:=0 \
        -p ros_img_topic_name:=/sub_image_combine_raw \
        -p ai_msg_sub_topic_name:=/hobot_mono2d_body_detection \
        -p ai_msg_pub_topic_name:=/hobot_hand_lmk_detection \
        --log-level warn \
        > "$LOG_DIR/hand_lmk.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 2

    # 5. hand_gesture (手势分类, 订阅 hand_lmk)
    echo "[START] 5/7 hand_gesture_detection..."
    ros2 run hand_gesture_detection hand_gesture_detection --ros-args \
        -p model_file_name:=/opt/tros/humble/lib/hand_gesture_detection/config/gestureDet_8x21.hbm \
        -p ai_msg_sub_topic_name:=/hobot_hand_lmk_detection \
        -p ai_msg_pub_topic_name:=/hobot_hand_gesture_detection \
        -p is_dynamic_gesture:=false \
        -p time_interval_sec:=0.25 \
        -p threshold:=0.5 \
        --log-level warn \
        > "$LOG_DIR/hand_gesture.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 1

    # 6. gesture_adapter (hobot结果→统一JSON, 增强版含手势名映射)
    echo "[START] 6/7 gesture_adapter_node..."
    ros2 run puppy_brain gesture_adapter_node --ros-args \
        -p input_topic:=/hobot_hand_gesture_detection \
        -p output_topic:=/gesture/result_json \
        -p log_interval_sec:=0.5 \
        > "$LOG_DIR/gesture_adapter.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 1

    # 7. mjpeg_bridge (左眼预览, 看手在画面哪里)
    echo "[START] 7/7 mjpeg_bridge 左眼预览 :$PORT_VIEW..."
    python3 "$PROJECT_ROOT/scripts/mjpeg_bridge.py" \
        --port $PORT_VIEW --topic /image_combine_jpeg --region bottom \
        > "$LOG_DIR/mjpeg.log" 2>&1 &
    echo $! >> "$PID_FILE"
    sleep 2

    BOARD_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    [ -z "$BOARD_IP" ] && BOARD_IP="<板端IP>"

    echo ""
    echo "================================================"
    echo " 手势识别测试已启动 (轻量, 未起深度)"
    echo "================================================"
    echo " 浏览器看视频:  http://$BOARD_IP:$PORT_VIEW"
    echo " 命令行看结果:  ros2 topic echo /gesture/result_json"
    echo " 看识别频率:    ros2 topic hz /gesture/result_json"
    echo ""
    echo " 手势对照表:"
    echo "   1.0 palm         手掌张开"
    echo "   2.0 fist         握拳"
    echo "   3.0 okay         OK圈"
    echo "   4.0 thumb_up     点赞"
    echo "   5.0 index_finger 竖食指"
    echo ""
    echo " 日志: $LOG_DIR/"
    echo " 停止: $0 stop"
    echo "================================================"
}

status_all() {
    echo "[STATUS] 手势测试进程:"
    if [ ! -f "$PID_FILE" ]; then
        echo "  未启动"
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

    echo ""
    echo "[STATUS] 关键 topic:"
    set +u
    source "$TROS_SETUP" 2>/dev/null
    set -u
    for t in /sub_image_combine_raw /hobot_mono2d_body_detection /hobot_hand_lmk_detection /hobot_hand_gesture_detection /gesture/result_json; do
        if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
            echo "  [OK]   $t"
        else
            echo "  [MISS] $t"
        fi
    done
}

show_logs() {
    echo "[LOGS] 实时日志 (Ctrl+C 退出): $LOG_DIR/"
    echo "  可用: ${NAMES[*]}"
    tail -f "$LOG_DIR"/*.log 2>/dev/null
}

case "${1:-}" in
    start)  start_all ;;
    stop)   stop_all ;;
    status) status_all ;;
    logs)   show_logs ;;
    *)      echo "用法: $0 {start|stop|status|logs}"; exit 1 ;;
esac
