#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ***************************************************************************************************
# 逐行详细注释版 - 专门为零基础学习者编写
# ***************************************************************************************************
#
# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                        《ROS2 决策节点 - 跟随控制器》                          │
# │                                                                             │
# │  功能说明:                                                                   │
# │  decision_node是机器狗系统的"大脑"，负责做出决策                              │
# │  它接收多个输入（视觉检测、手势、语音），然后决定机器狗应该做什么               │
# │                                                                             │
# │  决策逻辑:                                                                   │
# │  1. 如果收到手势命令（坐下/站立/停止），优先执行手势命令                        │
# │  2. 如果开启跟随模式，根据目标位置决定运动方向                                │
# │  3. 如果什么都没收到，保持当前状态                                            │
# │                                                                             │
# │  学习目标:                                                                   │
# │  1. 理解ROS2的发布/订阅机制                                                  │
# │  2. 理解状态机的概念                                                         │
# │  3. 理解跟随控制的原理                                                       │
# │  4. 理解多传感器融合决策                                                    │
# └─────────────────────────────────────────────────────────────────────────────┘

# ***************************************************************************************************
# 第一部分：导入必要的库
# ***************************************************************************************************

import rclpy                    # ROS2 Python客户端库，核心！
from rclpy.node import Node     # ROS2节点的基类
from std_msgs.msg import String  # 标准字符串消息类型
from geometry_msgs.msg import Twist  # 速度命令消息（如果需要）
import json                      # JSON数据解析
import time                      # 时间控制
import math                      # 数学函数
from typing import Optional, Dict, Any  # 类型提示

# ***************************************************************************************************
# 第二部分：常量定义
# ***************************************************************************************************

# 这些常量定义了系统的各种参数
# 修改这些值可以调整机器狗的行为

# -------------------- 图像参数 --------------------
IMAGE_WIDTH = 960.0    # 摄像头图像宽度（像素）
IMAGE_HEIGHT = 544.0   # 摄像头图像高度（像素）

# 注意：图像尺寸用于计算目标的实际位置和大小
# 如果你的摄像头分辨率不同，需要修改这里

# -------------------- 跟随距离阈值 --------------------
# 什么是面积占比？
# 目标在画面中占据的面积 / 整个画面的面积 = 面积占比
# 这个值是一个比例，范围0-1

# 当目标太近时停止（面积占比 > 这个值）
# 推荐范围: 0.35 - 0.50
# 值越小：越早在远处就停止
# 值越大：需要非常近才停止
FOLLOW_AREA_NEAR_STOP = 0.42

# 当目标太远时开始行走（面积占比 < 这个值）
# 推荐范围: 0.08 - 0.15
# 值越小：很远就开始走
# 值越大：需要比较近才开始走
FOLLOW_AREA_FAR_WALK = 0.10

# 最小有效目标比例
# 如果目标太小（占比低于这个值），认为是无效检测
# 推荐范围: 0.01 - 0.02
MIN_VALID_AREA_RATIO = 0.015

# -------------------- 转向控制参数 --------------------
# 什么是中心比例？
# 目标中心点的x坐标 / 画面宽度 = 中心比例
# 0.0 = 最左边，0.5 = 正中间，1.0 = 最右边

# 中心位置比例
CENTER_RATIO = 0.50

# 转向死区（在中心附近时不转向）
# 例如：CENTER_RATIO = 0.5, turn_deadband = 0.09
# 那么中心区域是 0.41-0.59（0.5 ± 0.09）
# 在这个区域内，认为目标是"正前方"，不需要转向
# 推荐范围: 0.05 - 0.15
TURN_DEADBAND_RATIO = 0.09

# 计算出的边界值（不要修改）
TURN_LEFT_RATIO = CENTER_RATIO - TURN_DEADBAND_RATIO  # 0.41
TURN_RIGHT_RATIO = CENTER_RATIO + TURN_DEADBAND_RATIO  # 0.59

# 最大转向误差比例（用于计算转向增益）
MAX_TURN_ERROR_RATIO = 0.28

# 转向增益（控制转向的灵敏度）
# 推荐范围: 0.5 - 1.0
# 值越大：转向越灵敏
# 值越小：转向越迟钝
TURN_GAIN = 0.85

# -------------------- 速度参数 --------------------
# 这些参数控制机器狗的运动速度

# 前进速度范围
FORWARD_MIN = 0.0   # 最小前进速度
FORWARD_MAX = 0.95   # 最大前进速度

# -------------------- 时间参数 --------------------
# Ghost Memory（幽灵记忆）
# 当目标暂时消失时，保持之前动作的时间
# 这样即使目标短暂被遮挡，机器狗也不会突然停止
# 推荐: 0.2 - 0.5秒
GHOST_MEMORY_TIME = 0.30  # 秒

# 发布重复间隔
# 为了确保命令被接收，会重复发布
# 推荐: 0.1 - 0.2秒
PUBLISH_REPEAT_SEC = 0.15

# -------------------- 手势参数 --------------------
# 手势识别结果的保持时间
# 手势命令执行后，会保持这个时间
GESTURE_HOLD_SEC = 0.8  # 秒

# 手势动作锁定时间（sit/stand等动作的锁定）
GESTURE_ACTION_LOCK_SEC = 2.5  # 秒

# stop手势的锁定时间
GESTURE_STOP_LOCK_SEC = 1.0  # 秒

# -------------------- 跟随开关 --------------------
# 默认是否开启跟随模式
# True = 开机自动开启跟随
# False = 需要手势开启
FOLLOW_DEFAULT_ENABLED = True

# -------------------- 平滑参数 --------------------
# 控制平滑系数
# 0-1之间，越大响应越快但可能抖动
# 越小越平滑但响应慢
CONTROL_SMOOTH_ALPHA = 0.28

# -------------------- 零点阈值 --------------------
# 如果转向/前进值小于这个阈值，认为是0（停止）
TURN_ZERO_THRESHOLD = 0.05
FORWARD_ZERO_THRESHOLD = 0.05

# ***************************************************************************************************
# 第三部分：手势定义
# ***************************************************************************************************

# 手势到动作的映射表
# 这个字典定义了每种手势对应的动作

GESTURE_TO_ACTION = {
    # 手势值（浮点数）: 对应动作
    1.0: "follow_on",    # 手掌张开 = 开启跟随
    2.0: "follow_off",   # 握拳 = 关闭跟随
    3.0: "stop",         # OK手势 = 停止
    4.0: "sit",          # 点赞 = 坐下
    5.0: "stand",        # 竖食指 = 站立
}

# ***************************************************************************************************
# 第四部分：DecisionNode类定义
# ***************************************************************************************************

class DecisionNode(Node):
    """
    DecisionNode - 决策节点类

    这是机器狗系统的"大脑"！
    它接收来自不同传感器的信息，然后做出决策

    接收的信息：
    1. /perception/result_json - 视觉检测结果（目标检测）
    2. /gesture/result_json - 手势识别结果
    3. /voice/result_json - 语音控制结果

    发出的信息：
    1. /puppy_action - 控制指令（如"walk"、"stop"、"sit"）

    工作原理：
    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐  │
    │   │  视觉检测   │     │  手势识别   │     │  语音控制   │  │
    │   │ perception  │     │  gesture   │     │   voice    │  │
    │   └──────┬──────┘     └──────┬──────┘     └──────┬──────┘  │
    │          │                   │                   │          │
    │          └───────────────────┼───────────────────┘          │
    │                              ↓                                │
    │                    ┌─────────────────┐                        │
    │                    │  决策仲裁器     │                        │
    │                    │  DecisionNode  │                        │
    │                    │   (大脑)       │                        │
    │                    └────────┬────────┘                        │
    │                             │                                 │
    │                             ↓                                 │
    │                    ┌─────────────────┐                        │
    │                    │  动作指令       │                        │
    │                    │ /puppy_action  │                        │
    │                    └────────┬────────┘                        │
    │                             │                                 │
    │                             ↓                                 │
    │                    ┌─────────────────┐                        │
    │                    │   机器狗执行    │                        │
    │                    └─────────────────┘                        │
    └─────────────────────────────────────────────────────────────┘
    """

    def __init__(self):
        """
        __init__ - 初始化决策节点

        这个函数在节点创建时调用一次
        主要做：
        1. 调用父类初始化
        2. 创建订阅者和发布者
        3. 初始化各种状态变量
        """

        # -------------------- 调用父类初始化 --------------------
        # super().__init__('decision_node')
        # 'decision_node' 是节点的名称
        # 可以在 rqt_graph 等工具中看到这个名字
        super().__init__('decision_node')

        # -------------------- 初始化状态变量 --------------------
        # 这些变量用于跟踪系统状态

        # 当前检测结果
        self.current_detections = []

        # 当前手势结果
        self.current_gesture = None

        # 当前语音指令
        self.current_voice_command = None

        # 是否开启跟随模式
        self.follow_enabled = FOLLOW_DEFAULT_ENABLED

        # 是否正在执行手势锁定动作
        self.gesture_action_lock = False
        self.gesture_action_lock_time = 0

        # 是否正在执行stop锁定
        self.stop_lock = False
        self.stop_lock_time = 0

        # 幽灵记忆：目标消失时保持动作的时间
        self.ghost_memory_time = 0.0
        self.last_action = None

        # 平滑控制值
        self.smoothed_forward = 0.0
        self.smoothed_turn = 0.0

        # 上次发布动作的时间
        self.last_publish_time = 0.0

        # 日志打印间隔
        self.log_interval = 5.0  # 秒
        self.last_log_time = time.time()

        # -------------------- 创建订阅者 --------------------
        # subscribe() 用于接收消息
        # 参数：
        # - 消息类型
        # - 话题名称
        # - 回调函数（收到消息时调用的函数）

        # 订阅视觉检测结果
        self.perception_sub = self.create_subscription(
            String,                                    # 消息类型
            '/perception/result_json',                  # 话题名称
            self.perception_callback,                   # 回调函数
            10                                          # 队列大小
        )

        # 订阅手势识别结果
        self.gesture_sub = self.create_subscription(
            String,
            '/gesture/result_json',
            self.gesture_callback,
            10
        )

        # 订阅语音控制结果
        self.voice_sub = self.create_subscription(
            String,
            '/voice/result_json',
            self.voice_callback,
            10
        )

        # -------------------- 创建发布者 --------------------
        # create_publisher() 用于发送消息
        # 参数：
        # - 消息类型
        # - 话题名称
        # - 队列大小

        self.action_pub = self.create_publisher(
            String,                      # 消息类型
            '/puppy_action',              # 话题名称
            10                           # 队列大小
        )

        # -------------------- 打印启动信息 --------------------
        self.get_logger().info("=" * 50)
        self.get_logger().info("Decision Node 决策节点已启动!")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"跟随模式默认: {'开启' if FOLLOW_DEFAULT_ENABLED else '关闭'}")
        self.get_logger().info(f"图像分辨率: {IMAGE_WIDTH} x {IMAGE_HEIGHT}")
        self.get_logger().info("=" * 50)

        # -------------------- 启动主循环 --------------------
        # create_timer() 创建一个定时器
        # 定时调用某个函数，实现周期性执行
        # 参数：间隔时间（秒），回调函数

        # 主控制循环，每0.1秒执行一次
        self.control_timer = self.create_timer(0.1, self.control_loop)

    # ***********************************************************************
    # 回调函数（当收到消息时自动调用）
    # ***********************************************************************

    def perception_callback(self, msg: String):
        """
        perception_callback - 视觉检测结果回调

        当收到 /perception/result_json 话题的消息时调用

        参数:
            msg: String消息，包含JSON格式的检测结果

        检测结果格式:
        {
            "detections": [
                {"name": "person", "bbox": [x1, y1, x2, y2], "score": 0.9},
                ...
            ]
        }
        """
        try:
            # 解析JSON数据
            # json.loads() 把JSON字符串转换为Python字典
            data = json.loads(msg.data)

            # 提取检测结果
            # detections 是一个列表，每个元素是一个检测目标
            self.current_detections = data.get('detections', [])

            # 调试：打印检测数量
            if len(self.current_detections) > 0:
                self.get_logger().debug(f"检测到 {len(self.current_detections)} 个目标")

        except Exception as e:
            # 如果解析失败，打印错误
            self.get_logger().error(f"解析检测结果失败: {e}")

    def gesture_callback(self, msg: String):
        """
        gesture_callback - 手势识别结果回调

        当收到 /gesture/result_json 话题的消息时调用

        消息格式:
        {"gesture_value": 1.0, "gesture_name": "张开"}
        """
        try:
            # 解析JSON
            data = json.loads(msg.data)

            # 提取手势值
            gesture_value = float(data.get('gesture_value', 0))

            # 更新当前手势
            self.current_gesture = gesture_value

            # 调试打印
            gesture_name = GESTURE_TO_ACTION.get(gesture_value, "未知")
            self.get_logger().info(f"收到手势: {gesture_value} -> {gesture_name}")

        except Exception as e:
            self.get_logger().error(f"解析手势结果失败: {e}")

    def voice_callback(self, msg: String):
        """
        voice_callback - 语音控制结果回调

        当收到 /voice/result_json 话题的消息时调用

        消息格式:
        {"command": "前进", "action": "walk"}
        """
        try:
            # 解析JSON
            data = json.loads(msg.data)

            # 提取语音指令
            self.current_voice_command = data.get('action', None)

            # 如果有指令，打印日志
            if self.current_voice_command:
                self.get_logger().info(f"收到语音指令: {self.current_voice_command}")

        except Exception as e:
            self.get_logger().error(f"解析语音结果失败: {e}")

    # ***********************************************************************
    # 主控制循环
    # ***********************************************************************

    def control_loop(self):
        """
        control_loop - 主控制循环

        这个函数每0.1秒被调用一次
        执行决策流程：
        1. 检查手势命令（最高优先级）
        2. 如果开启跟随，执行跟随决策
        3. 发布动作指令
        """

        current_time = time.time()

        # -------------------- 步骤1：检查手势命令 --------------------
        # 手势命令是最高优先级！

        action = self.check_gesture_command(current_time)

        if action is not None:
            # 有手势命令，直接执行
            self.publish_action(action, source="gesture")
            self.last_action = action
            return

        # -------------------- 步骤2：检查跟随决策 --------------------
        # 如果开启跟随模式，根据视觉检测结果决定动作

        if self.follow_enabled:
            action = self.decide_follow_action()

            if action is not None:
                self.publish_action(action, source="follow")
                self.last_action = action
                return

        # -------------------- 步骤3：发送停止命令 --------------------
        # 如果没有有效输入，发送停止命令
        # （但保持一定的频率，确保机器狗持续收到停止指令）

        if current_time - self.last_publish_time > PUBLISH_REPEAT_SEC:
            self.publish_action("stop", source="idle")
            self.last_action = "stop"

    def check_gesture_command(self, current_time: float) -> Optional[str]:
        """
        check_gesture_command - 检查手势命令

        参数:
            current_time: 当前时间

        返回:
            如果有要执行的手势命令，返回动作字符串
            否则返回None

        优先级:
        1. follow_on/off 立即切换跟随模式
        2. sit/stand/stop 需要锁定，锁定期间不响应其他命令
        """

        if self.current_gesture is None:
            return None

        gesture = self.current_gesture

        # -------------------- 切换跟随模式 --------------------
        if gesture == 1.0:  # 手掌张开 - 开启跟随
            self.follow_enabled = True
            self.get_logger().info("跟随模式: 开启")
            self.current_gesture = None  # 清除手势状态
            return None  # 不执行动作，只切换模式

        if gesture == 2.0:  # 握拳 - 关闭跟随
            self.follow_enabled = False
            self.get_logger().info("跟随模式: 关闭")
            self.current_gesture = None
            return None

        # -------------------- 执行动作命令 --------------------
        # 这些命令需要锁定，锁定期间不响应其他输入

        action_name = GESTURE_TO_ACTION.get(gesture)
        if action_name is None:
            return None

        # 检查是否需要锁定
        if action_name in ["sit", "stand"]:
            # sit/stand 需要锁定
            if not self.gesture_action_lock:
                # 开始锁定
                self.gesture_action_lock = True
                self.gesture_action_lock_time = current_time
                return action_name
            else:
                # 锁定中，忽略其他输入
                if current_time - self.gesture_action_lock_time > GESTURE_ACTION_LOCK_SEC:
                    # 锁定结束
                    self.gesture_action_lock = False
                return None

        if action_name == "stop":
            # stop 命令也需要锁定
            if not self.stop_lock:
                self.stop_lock = True
                self.stop_lock_time = current_time
                return action_name
            else:
                if current_time - self.stop_lock_time > GESTURE_STOP_LOCK_SEC:
                    self.stop_lock = False
                return None

        # 其他动作直接执行
        return action_name

    def decide_follow_action(self) -> Optional[str]:
        """
        decide_follow_action - 决定跟随动作

        根据视觉检测结果，决定机器狗应该怎么运动

        返回:
            动作字符串: "walk", "turn_left", "turn_right", "stop"
            或者 None（没有有效目标）

        跟随逻辑:
        ┌─────────────────────────────────────────────────────────┐
        │                                                         │
        │   检测到目标 ──→ 计算面积占比                           │
        │                          │                              │
        │         ┌─────────────────┼─────────────────┐            │
        │         ↓                 ↓                 ↓            │
        │    面积太大          面积适中          面积太小          │
        │    (>0.42)          (0.10-0.42)       (<0.10)          │
        │         ↓                 ↓                 ↓            │
        │      STOP            计算位置            WALK             │
        │                      │                              │
        │         ┌─────────────┼─────────────┐              │
        │         ↓             ↓             ↓               │
        │     偏左          正中间          偏右            │
        │         ↓             ↓             ↓               │
        │    TURN_LEFT      WALK         TURN_RIGHT         │
        │                                                         │
        └─────────────────────────────────────────────────────────┘
        """

        # -------------------- 查找人体目标 --------------------
        # 在所有检测结果中，找到最大的人体目标
        # 这是跟随控制的核心逻辑

        person_det = None
        max_area = 0

        for det in self.current_detections:
            # 只关注"person"类别
            if det.get('name') != 'person':
                continue

            # 获取边界框
            bbox = det.get('bbox', [])
            if len(bbox) != 4:
                continue

            # 计算面积
            x1, y1, x2, y2 = bbox
            area = (x2 - x1) * (y2 - y1)

            # 计算面积占比
            area_ratio = area / (IMAGE_WIDTH * IMAGE_HEIGHT)

            # 检查是否是最大人体目标
            if area_ratio > max_area:
                max_area = area_ratio
                person_det = det

        # -------------------- 如果没有人体 --------------------
        if person_det is None:
            # 目标消失，检查是否在幽灵记忆时间内
            if self.last_action is not None and self.last_action != "stop":
                if time.time() - self.ghost_memory_time < GHOST_MEMORY_TIME:
                    # 在幽灵记忆内，保持之前的动作
                    return self.last_action
            return None

        # 更新幽灵记忆时间
        self.ghost_memory_time = time.time()

        # -------------------- 计算面积占比 --------------------
        bbox = person_det.get('bbox', [])
        x1, y1, x2, y2 = bbox
        area = (x2 - x1) * (y2 - y1)
        area_ratio = area / (IMAGE_WIDTH * IMAGE_HEIGHT)

        # -------------------- 距离判断 --------------------

        # 太近 - 停止
        if area_ratio > FOLLOW_AREA_NEAR_STOP:
            self.get_logger().debug(f"距离判断: 太近 ({area_ratio:.3f} > {FOLLOW_AREA_NEAR_STOP})")
            return "stop"

        # 太远 - 行走
        if area_ratio < FOLLOW_AREA_FAR_WALK:
            self.get_logger().debug(f"距离判断: 太远 ({area_ratio:.3f} < {FOLLOW_AREA_FAR_WALK})")
            return "walk"

        # -------------------- 位置判断 --------------------
        # 面积适中，根据目标位置决定转向

        # 计算目标中心x坐标的比例
        center_x = (x1 + x2) / 2
        center_ratio = center_x / IMAGE_WIDTH

        # 判断左右
        if center_ratio < TURN_LEFT_RATIO:
            # 目标在左边，向左转
            self.get_logger().debug(f"位置判断: 偏左 ({center_ratio:.3f} < {TURN_LEFT_RATIO})")
            return "turn_left"

        elif center_ratio > TURN_RIGHT_RATIO:
            # 目标在右边，向右转
            self.get_logger().debug(f"位置判断: 偏右 ({center_ratio:.3f} > {TURN_RIGHT_RATIO})")
            return "turn_right"

        else:
            # 目标在中间，直行
            self.get_logger().debug(f"位置判断: 正前方 ({center_ratio:.3f})")
            return "walk"

    def publish_action(self, action: str, source: str = "unknown"):
        """
        publish_action - 发布动作指令

        参数:
            action: 动作字符串 ("walk", "stop", "sit", "stand", etc.)
            source: 动作来源 ("gesture", "follow", "voice", "idle")
        """

        # 创建消息
        msg = String()
        action_data = {
            "action": action,
            "source": source,
            "timestamp": time.time()
        }
        msg.data = json.dumps(action_data)

        # 发布
        self.action_pub.publish(msg)

        # 更新发布时间
        self.last_publish_time = time.time()

        # 定期打印日志
        current_time = time.time()
        if current_time - self.last_log_time > self.log_interval:
            self.get_logger().info(f"动作发布: {action} (来源: {source})")
            self.last_log_time = current_time

# ***************************************************************************************************
# 第五部分：主函数
# ***************************************************************************************************

def main(args=None):
    """
    main - 程序入口

    标准ROS2程序结构:
    1. 初始化rclpy
    2. 创建节点
    3. spin（保持节点运行）
    4. 销毁节点
    5. 关闭rclpy
    """

    # -------------------- 初始化ROS2 --------------------
    # 这一步必须先做
    rclpy.init(args=args)

    # -------------------- 创建节点 --------------------
    node = DecisionNode()

    # -------------------- 保持节点运行 --------------------
    # rclpy.spin(node) 是一个循环
    # 它会持续处理节点的回调函数，直到节点被关闭
    # 按Ctrl+C可以退出

    try:
        node.get_logger().info("开始处理...")
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 当用户按Ctrl+C时
        node.get_logger().info("收到中断信号，正在关闭...")

    finally:
        # -------------------- 清理 --------------------
        # 确保节点被正确销毁
        node.destroy_node()

        # 关闭ROS2
        rclpy.shutdown()

        node.get_logger().info("节点已关闭")

# ***************************************************************************************************
# 第六部分：程序入口
# ***************************************************************************************************

if __name__ == '__main__':
    main()

# ***************************************************************************************************
# 课后练习
# ***************************************************************************************************
#
# 练习1：调整跟随距离
#   修改 FOLLOW_AREA_NEAR_STOP 和 FOLLOW_AREA_FAR_WALK
#   观察机器狗的跟随行为变化
#
# 练习2：调整转向灵敏度
#   修改 TURN_GAIN
#   - 增大(>1.0)：转向更灵敏，可能抖动
#   - 减小(<0.5)：转向迟钝，更平稳
#
# 练习3：改变手势映射
#   修改 GESTURE_TO_ACTION 字典
#   自定义手势和动作的对应关系
#
# 练习4：添加日志
#   在关键位置添加 get_logger().info()
#   观察程序的执行流程
#
# ***************************************************************************************************
# 进阶学习
# ***************************************************************************************************
#
# 1. ROS2发布/订阅模型
#
#    发布者(Publisher) ──── 消息 ────> 订阅者(Subscriber)
#         │                              │
#         │                              │
#         └─── 话题(Topic) ───────────────┘
#
#    例子：
#    perception_node 发布到 /perception/result_json
#    decision_node 订阅 /perception/result_json
#
# 2. 回调函数机制
#
#    当订阅的话题有新消息时
#    ROS2自动调用我们注册的回调函数
#    这就是"异步通信"
#
# 3. 状态机
#
#    决策节点内部有一个简单的状态机：
#    ┌─────────┐
#    │  FOLLOW │ ◄────────────────┐
#    │  MODE   │                  │
#    └────┬────┘                  │
#         │                       │
#         │ 关闭跟随               │ 开启跟随
#         ↓                       │
#    ┌─────────┐                  │
#    │  IDLE   │ ──────────────────┘
#    │  MODE   │
#    └─────────┘
#
# 4. 幽灵记忆机制
#
#    当目标短暂消失时（如被遮挡）
#    机器狗会保持之前的动作一段时间
#    这样看起来更自然，不会突然停止
#
# ***************************************************************************************************
# 常见问题
# ***************************************************************************************************
#
# Q: 机器狗不响应任何命令
# A: 检查：
#    1. roscore 是否在运行
#    2. /puppy_action 话题是否正常发布
#    3. ros_udp_bridge 是否正常工作
#
# Q: 跟随时机器狗抖动
# A: 原因：
#    1. 转向增益太高
#    2. 目标检测不稳定
#    解决方案：
#    减小 TURN_GAIN
#    增加 CONTROL_SMOOTH_ALPHA
#
# Q: 机器狗离目标太近不停
# A: 解决方案：
#    减小 FOLLOW_AREA_NEAR_STOP（如从0.42改为0.35）
#
# Q: 如何手动测试？
# A: 使用命令行发布消息：
#    ros2 topic pub /puppy_action std_msgs/String "data: '{\"action\":\"walk\"}'"
#
# ***************************************************************************************************

print("=" * 80)
print("恭喜你完成了Decision Node的学习！")
print("=" * 80)
print("下一步建议：学习 perception_node（视觉感知节点）")
print("=" * 80)
