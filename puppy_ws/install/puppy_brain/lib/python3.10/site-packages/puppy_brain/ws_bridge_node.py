#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ws_bridge_node.py - ROS2 话题 <-> WebSocket 双向桥接节点

板端启动后:
  - 订阅所有监控话题(perception/gesture/voice/asr/chat/imu/action) 并广播给 WS 客户端
  - 接收 WS 客户端的控制指令, 转发到 /voice/result_json 或 /puppy_action 或 /chat/input_text

上位机(PC 浏览器) 连接 ws://<板端IP>:9090 即可监控+控制.

依赖:
  pip3 install websockets

启动:
  source /opt/tros/humble/setup.bash
  source install/setup.bash
  ros2 run puppy_brain ws_bridge_node

协议说明见上位机 index.html 注释.
"""
import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu

try:
    import websockets
except ImportError:
    websockets = None
    print('[ws_bridge_node] ERROR: websockets 未安装, 请执行: pip3 install websockets')


WS_HOST = '0.0.0.0'
WS_PORT = 9090

# IMU 高频, 限制转发频率避免刷爆浏览器
IMU_FORWARD_INTERVAL_SEC = 0.05  # 20Hz


class WsBridgeNode(Node):
    def __init__(self):
        super().__init__('ws_bridge_node')

        # ============ 订阅监控话题 ============
        self.create_subscription(String, '/perception/result_json', self.on_perception, 10)
        self.create_subscription(String, '/gesture/result_json', self.on_gesture, 10)
        self.create_subscription(String, '/voice/result_json', self.on_voice, 10)
        self.create_subscription(String, '/asr/text', self.on_asr, 10)
        self.create_subscription(String, '/chat/input_text', self.on_chat_in, 10)
        self.create_subscription(String, '/chat/response_text', self.on_chat_out, 10)
        self.create_subscription(String, '/puppy_action', self.on_action, 10)
        self.create_subscription(Imu, '/ros_robot_controller/imu_raw', self.on_imu, 10)

        # ============ 发布控制话题 ============
        # 控制指令走 /voice/result_json, 由 decision_node 处理(和语音同一通道)
        self.voice_pub = self.create_publisher(String, '/voice/result_json', 10)
        # 摇杆连续控制直接走 /puppy_action, 不经 decision_node
        self.action_pub = self.create_publisher(String, '/puppy_action', 10)
        # 聊天文本走 /chat/input_text -> chat_llm_bridge -> llm
        self.chat_pub = self.create_publisher(String, '/chat/input_text', 10)

        # ============ WS 客户端管理 ============
        # 注意: websockets 15.x 的 WebSocketServerProtocol 有只读属性 clients,
        # 这里用 ws_clients 避免冲突
        self.ws_clients = set()
        self.clients_lock = threading.Lock()
        self.start_time = time.time()

        # IMU 节流
        self.last_imu_send = 0.0

        # ============ 启动 WS 服务(独立线程 + 自己的 event loop) ============
        if websockets is None:
            self.get_logger().error('websockets 未安装, 桥接服务无法启动')
            return

        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self.ws_thread.start()

        # 心跳定时器(ROS2 主线程)
        self.create_timer(1.0, self.broadcast_status)
        self.get_logger().info(f'ws_bridge_node started. WS port={WS_PORT}')

    # =========================================================
    # ROS2 回调 -> 广播给 WS 客户端
    # =========================================================
    def on_perception(self, msg: String):
        self.broadcast({'type': 'perception', 'data': self._safe_json(msg.data)})

    def on_gesture(self, msg: String):
        self.broadcast({'type': 'gesture', 'data': self._safe_json(msg.data)})

    def on_voice(self, msg: String):
        self.broadcast({'type': 'voice', 'data': self._safe_json(msg.data)})

    def on_asr(self, msg: String):
        self.broadcast({'type': 'asr', 'data': self._safe_json(msg.data)})

    def on_chat_in(self, msg: String):
        self.broadcast({'type': 'chat_in', 'data': {'text': msg.data}})

    def on_chat_out(self, msg: String):
        self.broadcast({'type': 'chat_out', 'data': {'text': msg.data}})

    def on_action(self, msg: String):
        self.broadcast({'type': 'action', 'data': self._safe_json(msg.data)})

    def on_imu(self, msg: Imu):
        # 节流, 避免高频 IMU 刷爆浏览器
        now = time.time()
        if now - self.last_imu_send < IMU_FORWARD_INTERVAL_SEC:
            return
        self.last_imu_send = now
        self.broadcast({
            'type': 'imu',
            'data': {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z,
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z,
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w,
                },
                'timestamp': now,
            }
        })

    def _safe_json(self, raw: str):
        try:
            return json.loads(raw)
        except Exception:
            return {'raw': raw}

    # =========================================================
    # WS 广播(线程安全: 从 ROS2 线程投递到 WS event loop)
    # =========================================================
    def broadcast(self, obj: dict):
        if websockets is None:
            return
        msg = json.dumps(obj, ensure_ascii=False)
        asyncio.run_coroutine_threadsafe(self._async_broadcast(msg), self.loop)

    async def _async_broadcast(self, msg: str):
        if not self.ws_clients:
            return
        dead = []
        for c in list(self.ws_clients):
            try:
                await c.send(msg)
            except Exception:
                dead.append(c)
        for c in dead:
            self.ws_clients.discard(c)

    def broadcast_status(self):
        with self.clients_lock:
            n = len(self.ws_clients)
        self.broadcast({
            'type': 'status',
            'data': {
                'clients': n,
                'uptime': time.time() - self.start_time,
                'topics': {
                    'perception': '/perception/result_json',
                    'gesture': '/gesture/result_json',
                    'voice': '/voice/result_json',
                    'asr': '/asr/text',
                    'chat_in': '/chat/input_text',
                    'chat_out': '/chat/response_text',
                    'action': '/puppy_action',
                    'imu': '/ros_robot_controller/imu_raw',
                },
            }
        })

    # =========================================================
    # WS 服务线程
    # =========================================================
    def _run_ws_server(self):
        asyncio.set_event_loop(self.loop)

        async def handler(websocket, path=None):
            with self.clients_lock:
                self.ws_clients.add(websocket)
            try:
                addr = websocket.remote_address
            except Exception:
                addr = '?'
            self.get_logger().info(
                f'WS client connected: {addr}, total={len(self.ws_clients)}'
            )
            try:
                async for raw in websocket:
                    await self._handle_command(raw)
            except Exception as e:
                self.get_logger().info(f'WS handler exit: {e}')
            finally:
                with self.clients_lock:
                    self.ws_clients.discard(websocket)
                self.get_logger().info(
                    f'WS client disconnected, total={len(self.ws_clients)}'
                )

        async def main():
            # ping_interval/timeout 防止 NAT 断连
            async with websockets.serve(
                handler, WS_HOST, WS_PORT,
                ping_interval=10, ping_timeout=5,
                max_size=2 * 1024 * 1024,
            ):
                self.get_logger().info(f'WebSocket server listening on {WS_HOST}:{WS_PORT}')
                await asyncio.Future()  # run forever

        try:
            self.loop.run_until_complete(main())
        except Exception as e:
            self.get_logger().error(f'WS server error: {e}')

    # =========================================================
    # 处理 PC 上位机下发的指令
    # =========================================================
    async def _handle_command(self, raw: str):
        try:
            obj = json.loads(raw)
        except Exception:
            return
        t = obj.get('type')
        now = time.time()

        if t == 'command':
            # 离散动作: sit/stand/stop/forward/backward/turn_left/turn_right/
            #          follow_start/follow_stop
            # 走 /voice/result_json, 由 decision_node 统一处理(和语音同通道)
            action = obj.get('action')
            if not action:
                return
            payload = {
                'source': 'pc',
                'sub_source': 'ws_bridge',
                'command': action,
                'text': f'[PC] {action}',
                'timestamp': now,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.voice_pub.publish(msg)
            self.get_logger().info(f'[PC->voice] command={action}')

        elif t == 'follow_control':
            # 连续摇杆控制: 直接发 /puppy_action, 不经 decision_node
            # 决策节点若 follow_enabled=True 会和摇杆冲突, 所以摇杆前应先发 follow_stop
            forward = float(obj.get('forward', 0.0))
            turn = float(obj.get('turn', 0.0))
            # clamp
            forward = max(-1.0, min(1.0, forward))
            turn = max(-1.0, min(1.0, turn))
            payload = {
                'mode': 'follow_control',
                'forward': forward,
                'turn': turn,
                'source': 'pc_joystick',
                'timestamp': now,
                'follow_enabled': True,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.action_pub.publish(msg)

        elif t == 'chat':
            # 聊天文本 -> /chat/input_text -> chat_llm_bridge -> llm
            text = str(obj.get('text', '')).strip()
            if not text:
                return
            msg = String()
            msg.data = text
            self.chat_pub.publish(msg)
            self.get_logger().info(f'[PC->chat] {text[:40]}')

        elif t == 'follow_set':
            # 切换跟随模式
            enabled = bool(obj.get('enabled', True))
            action = 'follow_start' if enabled else 'follow_stop'
            payload = {
                'source': 'pc',
                'sub_source': 'ws_bridge',
                'command': action,
                'text': f'[PC] {action}',
                'timestamp': now,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.voice_pub.publish(msg)
            self.get_logger().info(f'[PC->voice] {action}')

        elif t == 'ping':
            # 客户端心跳, 直接回 pong
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
