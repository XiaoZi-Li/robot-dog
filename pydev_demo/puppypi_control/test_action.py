import socket
import time
import json
import threading
import math
import numpy as np

from servo_controller import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams
from action_group_control import runActionGroup, stopActionGroup


print("1. 正在唤醒神经系统与【动态步态引擎】...")

puppy = HiwonderPuppy(
    setServoPulse=setServoPulse,
    servoParams=PWMServoParams(),
    dof='8'
)

engine_started = False
robot_pose_state = "unknown"

# =========================
# 连续控制基础参数
# =========================
CONTROL_DT = 0.05
X_RAMP_PER_SEC = 10.0
YAW_RAMP_PER_SEC = 0.9

MAX_WALK_X = 6.0
MAX_TURN_YAW = 0.18

GESTURE_TURN_X = 2.0
GESTURE_TURN_YAW = 0.18

current_x = 0.0
current_y = 0.0
current_yaw_rate = 0.0

target_x = 0.0
target_y = 0.0
target_yaw_rate = 0.0

motion_lock = threading.Lock()
motion_thread_running = True

# 零控制判定：直接进入真正静止态
is_motion_stopped = False
ZERO_MOTION_X_EPS = 0.05
ZERO_MOTION_YAW_EPS = 0.02

# =========================
# 调试日志控制
# =========================
CONTROL_LOG_INTERVAL = 0.8
ENABLE_CONTROL_LOG = True
last_control_log_time = 0.0

ENABLE_IMU_LOG = False
IMU_LOG_INTERVAL = 2.0
last_imu_log_time = 0.0

# =========================
# IMU 接收
# =========================
IMU_UDP_IP = "127.0.0.1"
IMU_UDP_PORT = 5006

imu_lock = threading.Lock()
imu_thread_running = True

imu_ax = 0.0
imu_ay = 0.0
imu_az = 0.0
imu_gx = 0.0
imu_gy = 0.0
imu_gz = 0.0

imu_pitch = 0.0
imu_roll = 0.0
imu_last_recv_time = 0.0


def get_stance(x=0, y=0, z=-10, x_shift=-0.5):
    return np.array([
        [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
        [y, y, y, y],
        [z, z, z, z],
    ])


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def clamp_step(current, target, max_step):
    if target > current:
        return min(current + max_step, target)
    else:
        return max(current - max_step, target)


def maybe_log_imu(pitch, roll, ax, ay, az, gx, gy, gz):
    global last_imu_log_time

    if not ENABLE_IMU_LOG:
        return

    now = time.time()
    if now - last_imu_log_time < IMU_LOG_INTERVAL:
        return

    last_imu_log_time = now
    print(
        f"🧭 IMU: pitch={pitch:.2f}, roll={roll:.2f}, "
        f"acc=({ax:.3f}, {ay:.3f}, {az:.3f}), "
        f"gyro=({gx:.3f}, {gy:.3f}, {gz:.3f})"
    )


def maybe_log_control(local_current_x, local_target_x, local_current_yaw, local_target_yaw,
                      local_pitch=None, local_roll=None):
    global last_control_log_time

    if not ENABLE_CONTROL_LOG:
        return

    if (
        abs(local_current_x) < 0.05 and
        abs(local_target_x) < 0.05 and
        abs(local_current_yaw) < 0.02 and
        abs(local_target_yaw) < 0.02
    ):
        return

    now = time.time()
    if now - last_control_log_time < CONTROL_LOG_INTERVAL:
        return

    last_control_log_time = now

    print(
        f"🎛 控制: x={local_current_x:.2f}->{local_target_x:.2f}, "
        f"yaw={local_current_yaw:.2f}->{local_target_yaw:.2f}"
    )


def imu_receive_loop():
    global imu_thread_running
    global imu_ax, imu_ay, imu_az
    global imu_gx, imu_gy, imu_gz
    global imu_pitch, imu_roll, imu_last_recv_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IMU_UDP_IP, IMU_UDP_PORT))
    sock.settimeout(0.5)

    print(f"🧭 IMU监听线程启动，监听 {IMU_UDP_IP}:{IMU_UDP_PORT}")

    while imu_thread_running:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception:
            continue

        try:
            payload = json.loads(data.decode('utf-8'))
        except Exception:
            continue

        if payload.get("type") != "imu":
            continue

        try:
            la = payload.get("linear_acceleration", {})
            av = payload.get("angular_velocity", {})

            ax = float(la.get("x", 0.0))
            ay = float(la.get("y", 0.0))
            az = float(la.get("z", 0.0))

            gx = float(av.get("x", 0.0))
            gy = float(av.get("y", 0.0))
            gz = float(av.get("z", 0.0))

            roll = math.degrees(math.atan2(ay, az))
            pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

            with imu_lock:
                imu_ax = ax
                imu_ay = ay
                imu_az = az
                imu_gx = gx
                imu_gy = gy
                imu_gz = gz
                imu_roll = roll
                imu_pitch = pitch
                imu_last_recv_time = time.time()

            maybe_log_imu(pitch, roll, ax, ay, az, gx, gy, gz)

        except Exception:
            continue

    try:
        sock.close()
    except Exception:
        pass


def apply_basic_stance():
    puppy.stance_config(
        get_stance(x=0, y=0, z=-10, x_shift=-0.5),
        pitch=0,
        roll=0
    )
    time.sleep(0.2)


def apply_basic_gait():
    puppy.gait_config(
        overlap_time=0.23,
        swing_time=0.32,
        clearance_time=0.0,
        z_clearance=7
    )
    time.sleep(0.1)


def enter_static_stop():
    global is_motion_stopped
    global current_x, current_y, current_yaw_rate

    if is_motion_stopped:
        return

    try:
        puppy.move_stop(servo_run_time=200)
        time.sleep(0.10)
    except Exception:
        pass

    try:
        apply_basic_stance()
    except Exception:
        pass

    current_x = 0.0
    current_y = 0.0
    current_yaw_rate = 0.0
    is_motion_stopped = True


def exit_static_stop_if_needed():
    global is_motion_stopped

    if not is_motion_stopped:
        return

    try:
        apply_basic_gait()
    except Exception:
        pass

    is_motion_stopped = False


def motion_update_loop():
    global current_x, current_y, current_yaw_rate
    global target_x, target_y, target_yaw_rate
    global motion_thread_running

    while motion_thread_running:
        with motion_lock:
            local_target_x = target_x
            local_target_y = target_y
            local_target_yaw = target_yaw_rate

        # 目标已经归零：直接进真正静止态，不再持续 puppy.move()
        if (
            abs(local_target_x) < ZERO_MOTION_X_EPS and
            abs(local_target_yaw) < ZERO_MOTION_YAW_EPS
        ):
            enter_static_stop()
            time.sleep(CONTROL_DT)
            continue

        exit_static_stop_if_needed()

        max_dx = X_RAMP_PER_SEC * CONTROL_DT
        max_dyaw = YAW_RAMP_PER_SEC * CONTROL_DT

        current_x = clamp_step(current_x, local_target_x, max_dx)
        current_yaw_rate = clamp_step(current_yaw_rate, local_target_yaw, max_dyaw)
        current_y = local_target_y

        try:
            puppy.move(
                x=float(current_x),
                y=float(current_y),
                yaw_rate=float(current_yaw_rate)
            )
        except Exception:
            pass

        with imu_lock:
            local_pitch = imu_pitch
            local_roll = imu_roll

        maybe_log_control(
            current_x, local_target_x,
            current_yaw_rate, local_target_yaw,
            local_pitch, local_roll
        )

        time.sleep(CONTROL_DT)


def set_motion_target(x, y, yaw_rate):
    global target_x, target_y, target_yaw_rate

    with motion_lock:
        target_x = float(x)
        target_y = float(y)
        target_yaw_rate = float(yaw_rate)


def force_motion_zero():
    global current_x, current_y, current_yaw_rate
    global target_x, target_y, target_yaw_rate

    with motion_lock:
        target_x = 0.0
        target_y = 0.0
        target_yaw_rate = 0.0

    current_x = 0.0
    current_y = 0.0
    current_yaw_rate = 0.0

    enter_static_stop()


def safe_stop_action_group():
    try:
        stopActionGroup()
        time.sleep(0.1)
    except Exception:
        pass


def safe_move_stop(run_time=300):
    try:
        force_motion_zero()
        puppy.move_stop(servo_run_time=run_time)
        time.sleep(0.15)
    except Exception:
        pass


def recover_to_stand(restart_engine=False):
    global engine_started, robot_pose_state

    print("🔄 开始执行恢复到站立流程...")

    safe_stop_action_group()
    safe_move_stop(run_time=300)

    try:
        puppy.servo_force_run()
        time.sleep(0.1)
    except Exception:
        pass

    apply_basic_stance()
    apply_basic_gait()

    if restart_engine or (not engine_started):
        try:
            puppy.start()
            engine_started = True
            time.sleep(0.3)
            print("✅ 步态引擎已启动/重启")
        except Exception:
            pass

    safe_move_stop(run_time=300)
    robot_pose_state = "stand"
    print("✅ 已恢复到可控站立状态")


def ensure_motion_ready():
    global robot_pose_state

    if robot_pose_state != "stand":
        recover_to_stand(restart_engine=False)


def do_stop():
    global robot_pose_state

    print("🐶 执行：停止")
    safe_move_stop(run_time=200)
    robot_pose_state = "stand"


def do_stand():
    print("🐶 执行：站立/恢复")
    recover_to_stand(restart_engine=False)


def do_sit():
    global robot_pose_state

    print("🐶 执行：坐下")

    safe_stop_action_group()
    safe_move_stop(run_time=300)

    try:
        runActionGroup('sit.d6ac')
        time.sleep(0.3)
        robot_pose_state = "sit"
        print("✅ 坐下动作执行完成")
    except Exception:
        pass


def do_walk():
    ensure_motion_ready()
    print("🐶 执行：前进")
    set_motion_target(MAX_WALK_X, 0.0, 0.0)


def do_turn_left():
    ensure_motion_ready()
    print("🐶 执行：左转")
    set_motion_target(GESTURE_TURN_X, 0.0, GESTURE_TURN_YAW)


def do_turn_right():
    ensure_motion_ready()
    print("🐶 执行：右转")
    set_motion_target(GESTURE_TURN_X, 0.0, -GESTURE_TURN_YAW)


def set_follow_control_target(forward, turn):
    forward = clamp(float(forward), 0.0, 1.0)
    turn = clamp(float(turn), -1.0, 1.0)

    # 让连续跟随时的前进量更接近原来离散 walk 的体感
    FOLLOW_FORWARD_GAIN = 1.45
    FOLLOW_MIN_CRUISE_X = 2.8

    x_cmd = MAX_WALK_X * forward * FOLLOW_FORWARD_GAIN * (1.0 - 0.18 * abs(turn))
    x_cmd = clamp(x_cmd, 0.0, MAX_WALK_X)

    # 只要在跟随且 forward 不太小，就给一个最小可感知巡航前进量
    if forward > 0.22 and x_cmd < FOLLOW_MIN_CRUISE_X:
        x_cmd = FOLLOW_MIN_CRUISE_X

    yaw_cmd = MAX_TURN_YAW * turn

    if abs(x_cmd) < 0.18:
        x_cmd = 0.0
    if abs(yaw_cmd) < 0.02:
        yaw_cmd = 0.0

    set_motion_target(x_cmd, 0.0, yaw_cmd)

try:
    puppy.servo_force_run()
    time.sleep(0.1)

    apply_basic_stance()
    apply_basic_gait()

    puppy.start()
    engine_started = True
    time.sleep(0.3)

    puppy.move_stop(servo_run_time=500)
    time.sleep(1.0)

    robot_pose_state = "stand"

except Exception:
    pass


motion_thread = threading.Thread(target=motion_update_loop, daemon=True)
motion_thread.start()

imu_thread = threading.Thread(target=imu_receive_loop, daemon=True)
imu_thread.start()

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"\n✅ 右脑 (运动中枢) 启动成功！正在监听端口 {UDP_PORT}...")
print("等待左脑 (ROS2 决策系统) 发送指令...\n")

current_action = "stop"
follow_enabled = True


def parse_action(raw_text):
    try:
        payload = json.loads(raw_text)
        if isinstance(payload, dict):
            if payload.get("mode") == "follow_control":
                return "follow_control", payload.get("source", "follow"), payload

            action = payload.get("action", "").strip()
            source = payload.get("source", "unknown")
            return action, source, payload
    except Exception:
        pass

    return raw_text.strip(), "legacy", None


try:
    while True:
        data, addr = sock.recvfrom(2048)
        raw = data.decode('utf-8').strip()

        action, source, payload = parse_action(raw)

        if not action:
            continue

        if action == "follow_on":
            follow_enabled = True
            continue

        if action == "follow_off":
            follow_enabled = False
            do_stop()
            current_action = "stop"
            continue

        if action == "follow_control":
            ensure_motion_ready()
            try:
                forward = float(payload.get("forward", 0.0))
                turn = float(payload.get("turn", 0.0))
                set_follow_control_target(forward, turn)
            except Exception:
                pass
            continue

        if action == current_action:
            continue

        print(f"📥 收到并执行指令: 【{action}】 source=[{source}]")
        current_action = action

        if action == "walk":
            do_walk()

        elif action == "turn_left":
            do_turn_left()

        elif action == "turn_right":
            do_turn_right()

        elif action == "stop":
            do_stop()

        elif action == "sit":
            do_sit()

        elif action == "stand":
            do_stand()

except KeyboardInterrupt:
    print("\n🛑 右脑已安全关闭...")
    motion_thread_running = False
    imu_thread_running = False
    try:
        safe_stop_action_group()
    except Exception:
        pass
    try:
        puppy.move_stop(servo_run_time=200)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass