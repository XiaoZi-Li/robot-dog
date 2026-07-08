import evdev

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for dev in devices:
    if 'js' in dev.name.lower() or 'gamepad' in dev.name.lower() or 'controller' in dev.name.lower():
        print(f"发现控制器: {dev.name} @ {dev.path}")
        print(" 按键/轴:", [ev.type for ev in dev.capabilities().get(3, [])])
        print(" 绝对轴:", dev.capabilities().get(3, []))
        dev.grab() # 独占读取
        print("\n开始读取数据，摇杆动起来...\n")
        for event in dev.read_loop():
            print(f" event: type={event.type}, code={event.code}, value={event.value}")