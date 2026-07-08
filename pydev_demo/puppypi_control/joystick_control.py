#!/usr/bin/env python3
"""
USB手柄 --> UDP --> 机器狗运动控制
兼容 D-Pad 十字键（按走松停）和 摇杆（比例控制）
"""
import evdev
import socket
import time
import json

# ============================================
# 配置
# ============================================
DEVICE_PATH = "/dev/input/event1"
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# 摇杆死区（中位偏移容忍）
DEADZONE = 20  # 127 ± 20 范围算"不动"

# ============================================
# 映射表
# ============================================
AXIS_MAP = {
    0: "x",
    1: "y",
    2: "rx",
    5: "ry",
}

DPAD_MAP = {
    16: {
        -1: "turn_left",
        1: "turn_right",
    },
    17: {
        -1: "walk",
        1: "walk",
    },
}

BUTTON_MAP = {
    304: "sit",
    305: "stand",
    308: "turn_left",
    309: "turn_right",
}

# ============================================
# UDP 发送
# ============================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_action(action, **kwargs):
    """发送 JSON 指令到运动控制脚本"""
    msg = json.dumps({"action": action, "source": "joystick", **kwargs})
    sock.sendto(msg.encode("utf-8"), (UDP_IP, UDP_PORT))
    extra = ""
    if kwargs:
        extra = f"  {kwargs}"
    print(f"  🎮 -> UDP: {action}{extra}")


# ============================================
# 主循环
# ============================================
dev = evdev.InputDevice(DEVICE_PATH)
print(f"🎮 手柄已连接: {dev.name}")
print(f"📡 发送至 UDP {UDP_IP}:{UDP_PORT}")
print(f"🎯 D-Pad:方向  摇杆:比例  按键:A=坐 B=站 LB/RB=快转")
print(f"🚀 开始控制！\n")

current_action = None
last_stick_time = 0

for event in dev.read_loop():
    # --- D-Pad 十字键 ---
    if event.type == evdev.ecodes.EV_ABS and event.code in DPAD_MAP:
        direction = DPAD_MAP[event.code].get(event.value)
        if direction:
            send_action(direction)
            current_action = direction
        elif event.value == 0:
            send_action("stop")
            current_action = None

    # --- 摇杆模拟量（限制频率） ---
    elif event.type == evdev.ecodes.EV_ABS and event.code in AXIS_MAP:
        now = time.time()
        if now - last_stick_time < 0.05:
            continue
        last_stick_time = now

        val_ly = dev.absinfo(1).value
        val_ly = (val_ly - 127) / 127.0

        val_rx = dev.absinfo(2).value
        val_rx = (val_rx - 127) / 127.0

        if abs(val_ly) < DEADZONE / 127.0:
            val_ly = 0.0
        if abs(val_rx) < DEADZONE / 127.0:
            val_rx = 0.0

        if val_ly == 0.0 and val_rx == 0.0:
            if current_action != "stop":
                send_action("stop")
                current_action = "stop"
        elif abs(val_rx) > abs(val_ly):
            action = "turn_left" if val_rx > 0 else "turn_right"
        else:
            action = "walk"

        if action != current_action:
            send_action(action)
            current_action = action

    # --- 按键 ---
    elif event.type == evdev.ecodes.EV_KEY and event.value == 1:
        action = BUTTON_MAP.get(event.code)
        if action:
            send_action(action)
            current_action = action
        if event.code == 311:
            print("\n🛑 已退出控制")
            break
