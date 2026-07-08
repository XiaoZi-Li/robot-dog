import socket
import time
import json
import threading
import numpy as np

from servo_controller import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams
from action_group_control import runActionGroup, stopActionGroup


print("1. 正在唤醒神经系统与【动态步态引擎】...")

# =========================================================
# 幻尔狗初始化
# =========================================================
puppy = HiwonderPuppy(
    setServoPulse=setServoPulse,
    servoParams=PWMServoParams(),
    dof='8'
)

engine_started = False
robot_pose_state = "unknown"   # stand / sit / unknown


# =========================================================
# 平滑控制参数
# =========================================================
CONTROL_DT = 0.05              # 平滑控制周期，50ms
X_RAMP_PER_SEC = 20.0          # 前进速度斜坡
YAW_RAMP_PER_SEC = 1.5         # 转向角速度斜坡

# 动作目标值
WALK_X = 10.0
TURN_X = 4.0
TURN_YAW = 0.35

# 当前控制量
current_x = 0.0
current_y = 0.0
current_yaw_rate = 0.0

# 目标控制量
target_x = 0.0
target_y = 0.0
target_yaw_rate = 0.0

motion_lock = threading.Lock()
motion_thread_running = True


def get_stance(x=0, y=0, z=-10, x_shift=-0.5):
    """计算站立时的 4 腿基础坐标矩阵"""
    return np.array([
        [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
        [y, y, y, y],
        [z, z, z, z],
    ])


def clamp_step(current, target, max_step):
    if target > current:
        return min(current + max_step, target)
    else:
        return max(current - max_step, target)


def motion_update_loop():
    """
    后台平滑控制线程：
    不再硬切 puppy.move(...)，而是缓慢逼近目标 x / yaw_rate
    """
    global current_x, current_y, current_yaw_rate
    global target_x, target_y, target_yaw_rate
    global motion_thread_running

    last_print_time = 0.0

    while motion_thread_running:
        with motion_lock:
            local_target_x = target_x
            local_target_y = target_y
            local_target_yaw = target_yaw_rate

        max_dx = X_RAMP_PER_SEC * CONTROL_DT
        max_dyaw = YAW_RAMP_PER_SEC * CONTROL_DT

        new_x = clamp_step(current_x, local_target_x, max_dx)
        new_yaw = clamp_step(current_yaw_rate, local_target_yaw, max_dyaw)

        changed = (
            abs(new_x - current_x) > 1e-6
            or abs(new_yaw - current_yaw_rate) > 1e-6
        )

        current_x = new_x
        current_yaw_rate = new_yaw
        current_y = local_target_y

        # 只要目标不是静止，就持续调用 move
        # 这样 walk/turn/切换会平滑很多
        if abs(current_x) > 1e-4 or abs(current_yaw_rate) > 1e-4:
            try:
                puppy.move(
                    x=float(current_x),
                    y=float(current_y),
                    yaw_rate=float(current_yaw_rate)
                )
            except Exception as e:
                print(f"⚠️ motion thread 调用 puppy.move 异常: {e}")

        # 调试打印不要太频繁
        now = time.time()
        if changed and (now - last_print_time > 0.4):
            print(
                f"🎛 平滑控制: current_x={current_x:.2f}, "
                f"target_x={local_target_x:.2f}, "
                f"current_yaw={current_yaw_rate:.2f}, "
                f"target_yaw={local_target_yaw:.2f}"
            )
            last_print_time = now

        time.sleep(CONTROL_DT)


def set_motion_target(x, y, yaw_rate):
    global target_x, target_y, target_yaw_rate
    with motion_lock:
        target_x = float(x)
        target_y = float(y)
        target_yaw_rate = float(yaw_rate)


def force_motion_zero():
    """
    立即把当前和目标控制量都清零，配合 move_stop 用
    """
    global current_x, current_y, current_yaw_rate
    global target_x, target_y, target_yaw_rate

    with motion_lock:
        target_x = 0.0
        target_y = 0.0
        target_yaw_rate = 0.0

    current_x = 0.0
    current_y = 0.0
    current_yaw_rate = 0.0


def safe_stop_action_group():
    try:
        stopActionGroup()
        time.sleep(0.1)
    except Exception as e:
        print(f"⚠️ stopActionGroup 异常，但继续恢复: {e}")


def safe_move_stop(run_time=300):
    try:
        force_motion_zero()
        puppy.move_stop(servo_run_time=run_time)
        time.sleep(0.15)
    except Exception as e:
        print(f"⚠️ move_stop 异常，但继续恢复: {e}")


def apply_basic_stance():
    puppy.stance_config(
        get_stance(x=0, y=0, z=-10, x_shift=-0.5),
        pitch=0,
        roll=0
    )
    time.sleep(0.2)


def apply_basic_gait():
    puppy.gait_config(
        overlap_time=0.15,
        swing_time=0.25,
        clearance_time=0.0,
        z_clearance=8
    )
    time.sleep(0.1)


def recover_to_stand(restart_engine=False):
    """
    统一恢复流程：
    - 停动作组
    - 停步态
    - 重新上扭矩
    - 恢复站姿
    - 恢复步态参数
    - 必要时重新启动步态线程
    """
    global engine_started, robot_pose_state

    print("🔄 开始执行恢复到站立流程...")

    safe_stop_action_group()
    safe_move_stop(run_time=300)

    try:
        puppy.servo_force_run()
        time.sleep(0.1)
    except Exception as e:
        print(f"⚠️ servo_force_run 异常: {e}")

    apply_basic_stance()
    apply_basic_gait()

    if restart_engine or (not engine_started):
        try:
            puppy.start()
            engine_started = True
            time.sleep(0.3)
            print("✅ 步态引擎已启动/重启")
        except Exception as e:
            print(f"⚠️ puppy.start() 异常: {e}")

    safe_move_stop(run_time=300)
    robot_pose_state = "stand"
    print("✅ 已恢复到可控站立状态")


def ensure_motion_ready():
    """
    在 walk/turn 前确保机器人处于可运动状态。
    如果刚 sit 过，先自动恢复。
    """
    global robot_pose_state

    if robot_pose_state != "stand":
        print(f"ℹ️ 当前状态为 [{robot_pose_state}]，先恢复到站立再运动")
        recover_to_stand(restart_engine=False)


def do_stop():
    global robot_pose_state

    print("🐶 执行：停止")
    safe_move_stop(run_time=200)
    apply_basic_stance()
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
    except Exception as e:
        print(f"❌ sit 动作组执行异常: {e}")


def do_crouch():
    """趴下 (执行 lie_down.d6ac 动作组)"""
    global robot_pose_state

    print("🐶 执行：趴下")
    safe_stop_action_group()
    safe_move_stop(run_time=300)
    try:
        runActionGroup('lie_down.d6ac')
        time.sleep(0.3)
        robot_pose_state = "crouch"
        print("✅ 趴下动作执行完成")
    except Exception as e:
        print(f"❌ crouch 动作组执行异常: {e}")


def do_wave():
    """招手致意 (执行 wave.d6ac 动作组, 执行后恢复站立)"""
    print("🐶 执行：招手")
    ensure_motion_ready()
    safe_stop_action_group()
    try:
        runActionGroup('wave.d6ac')
        time.sleep(0.3)
        print("✅ 招手动作执行完成")
    except Exception as e:
        print(f"❌ wave 动作组执行异常: {e}")


def do_bow():
    """鞠躬 (执行 bow.d6ac 动作组)"""
    print("🐶 执行：鞠躬")
    ensure_motion_ready()
    safe_stop_action_group()
    try:
        runActionGroup('bow.d6ac')
        time.sleep(0.3)
        print("✅ 鞠躬动作执行完成")
    except Exception as e:
        print(f"❌ bow 动作组执行异常: {e}")


def do_nod():
    """点头 (执行 nod.d6ac 动作组)"""
    print("🐶 执行：点头")
    ensure_motion_ready()
    safe_stop_action_group()
    try:
        runActionGroup('nod.d6ac')
        time.sleep(0.3)
        print("✅ 点头动作执行完成")
    except Exception as e:
        print(f"❌ nod 动作组执行异常: {e}")


def do_walk():
    ensure_motion_ready()
    print("🐶 执行：前进")
    set_motion_target(WALK_X, 0.0, 0.0)


def do_turn_left():
    ensure_motion_ready()
    print("🐶 执行：左转")
    set_motion_target(TURN_X, 0.0, TURN_YAW)


def do_turn_right():
    ensure_motion_ready()
    print("🐶 执行：右转")
    set_motion_target(TURN_X, 0.0, -TURN_YAW)


# =========================================================
# 上电初始化
# =========================================================
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

except Exception as e:
    print(f"❌ 初始化异常: {e}")


# 启动平滑控制线程
motion_thread = threading.Thread(target=motion_update_loop, daemon=True)
motion_thread.start()


# =========================================================
# UDP 监听配置
# =========================================================
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"\n✅ 右脑 (运动中枢) 启动成功！正在监听端口 {UDP_PORT}...")
print("等待左脑 (ROS2 决策系统) 发送指令...\n")

current_action = "stop"
follow_enabled = True


def parse_action(raw_text):
    """
    兼容三种协议：
    1) follow_control 连续控制: {"mode":"follow_control","forward":0.5,"turn":-0.3,...}
    2) 新协议 JSON: {"action":"walk","source":"follow",...}
    3) 旧协议纯字符串: walk
    """
    try:
        payload = json.loads(raw_text)
        if isinstance(payload, dict):
            # 连续控制模式 (摇杆/跟随/voice_move)
            if payload.get("mode") == "follow_control":
                return "follow_control", payload.get("source", "unknown"), payload
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

        # ============ 连续控制模式 (摇杆/跟随/voice_move) ============
        # 不走 current_action 去重, 每帧都更新目标速度
        if action == "follow_control":
            try:
                fwd = float(payload.get("forward", 0.0))   # [-1, 1]
                trn = float(payload.get("turn", 0.0))       # [-1, 1]
                # 归一化保护
                fwd = max(-1.0, min(1.0, fwd))
                trn = max(-1.0, min(1.0, trn))
                # 映射到实际速度: forward→WALK_X, turn→TURN_YAW
                target_vx = fwd * WALK_X
                target_vyaw = trn * TURN_YAW * 2.0     # 摇杆 turn 范围大些
                ensure_motion_ready()
                set_motion_target(target_vx, 0.0, target_vyaw)
                # 静音日志, 摇杆每秒几十帧, 打印会刷屏
            except Exception as e:
                print(f"⚠️ follow_control 解析失败: {e}")
            continue

        if action == "follow_on":
            follow_enabled = True
            print("📥 收到模式指令: 【follow_on】 -> 已开启跟随模式")
            continue

        if action == "follow_off":
            follow_enabled = False
            print("📥 收到模式指令: 【follow_off】 -> 已关闭跟随模式，并停车")
            do_stop()
            current_action = "stop"
            continue

        # 防止重复执行同一个动作
        if action == current_action:
            continue

        print(f"📥 收到并执行指令: 【{action}】 source=[{source}]")
        current_action = action

        if action in ("walk", "forward"):       # forward 别名 → walk
            do_walk()

        elif action == "backward":              # 后退 (反向 walk)
            ensure_motion_ready()
            print("🐶 执行：后退")
            set_motion_target(-WALK_X, 0.0, 0.0)

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

        elif action == "crouch":
            do_crouch()

        elif action == "wave":
            do_wave()

        elif action == "bow":
            do_bow()

        elif action == "nod":
            do_nod()

        else:
            print(f"⚠️ 未知动作，忽略: {action}")

except KeyboardInterrupt:
    print("\n🛑 右脑已安全关闭...")
    motion_thread_running = False
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