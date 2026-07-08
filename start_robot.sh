#!/bin/bash
# start_robot.sh - 机器狗底层运动控制启动脚本
# 功能: 启动 sit.py (运动中枢, UDP:5005) + imu_node_ros2 (IMU 发布, 50Hz)
#
# 用法:
#   ./start_robot.sh start    启动
#   ./start_robot.sh stop     停止
#   ./start_robot.sh restart  重启
#   ./start_robot.sh status   查看状态
set -u

PUPPYPI_DIR="/app/pydev_demo/puppypi_control"
PUPPY_WS="/app/puppy_ws"
TROS_SETUP="/opt/tros/humble/setup.bash"
LOG_DIR="/tmp/start_robot"
PID_FILE="$LOG_DIR/pids"

NAMES=(sit_py imu_node)

check_env() {
    [ -f "$TROS_SETUP" ] || { echo "[ERR] TROS 缺失: $TROS_SETUP"; exit 1; }
    [ -f "$PUPPYPI_DIR/sit.py" ] || { echo "[ERR] sit.py 缺失: $PUPPYPI_DIR/sit.py"; exit 1; }
    [ -d "$PUPPY_WS/install/puppy_brain" ] || { echo "[ERR] puppy_brain 未编译, 先 cd $PUPPY_WS && colcon build --packages-select puppy_brain"; exit 1; }
}

stop_all() {
    echo "[STOP] 清理 start_robot 进程..."
    if [ -f "$PID_FILE" ]; then
        while read -r pid; do
            [ -n "$pid" ] && kill "$pid" 2>/dev/null
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    pkill -f 'puppypi_control/sit.py' 2>/dev/null
    pkill -f 'imu_node_ros2' 2>/dev/null
    sleep 1
    # 紧急停车: 给 UDP 5005 发 stop
    echo '{"action":"stop","source":"emergency"}' | nc -u -w1 127.0.0.1 5005 2>/dev/null || true
    echo "[STOP] 完成"
}

start_all() {
    check_env
    mkdir -p "$LOG_DIR"
    : > "$PID_FILE"

    stop_all 2>/dev/null
    sleep 1

    # ===== 1. sit.py (运动中枢, 监听 UDP 5005) =====
    echo "[START] 1/2 sit.py (运动中枢, UDP 5005)..."
    (
        cd "$PUPPYPI_DIR"
        python3 sit.py > "$LOG_DIR/sit_py.log" 2>&1
    ) &
    echo $! >> "$PID_FILE"

    # 等 sit.py 起来 (UDP 5005 监听)
    echo "[START] 等待 sit.py 监听 UDP 5005..."
    for i in $(seq 1 15); do
        if ss -ulnp 2>/dev/null | grep -q ":5005"; then
            echo "[START] sit.py 监听 OK"
            break
        fi
        sleep 1
    done
    sleep 1

    # ===== 2. imu_node_ros2 (IMU 发布, 50Hz) =====
    echo "[START] 2/2 imu_node_ros2 (IMU 50Hz)..."
    set +u
    source "$TROS_SETUP"
    source "$PUPPY_WS/install/setup.bash"
    set -u
    ros2 run puppy_brain imu_node_ros2 > "$LOG_DIR/imu_node.log" 2>&1 &
    echo $! >> "$PID_FILE"

    sleep 2

    echo ""
    echo "================================================"
    echo " 机器狗底层启动完成"
    echo "================================================"
    echo " sit.py:     UDP 127.0.0.1:5005 (运动中枢)"
    echo " IMU:        /ros_robot_controller/imu_raw (50Hz)"
    echo " 日志:       $LOG_DIR/"
    echo " 停止:       $0 stop"
    echo " 状态:       $0 status"
    echo "================================================"
}

status_all() {
    echo "[STATUS] start_robot 进程状态:"
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
    if ss -ulnp 2>/dev/null | grep -q ":5005"; then
        echo "  [OK]   UDP 5005 (sit.py) 监听中"
    else
        echo "  [FAIL] UDP 5005 未监听"
    fi
}

case "${1:-}" in
    start)   start_all ;;
    stop)    stop_all ;;
    restart) stop_all; start_all ;;
    status)  status_all ;;
    *)       echo "用法: $0 {start|stop|restart|status}"; exit 1 ;;
esac
