import time
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from servo_controller import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams


print("=== ROS IMU 探针测试开始 ===")


class MPU6050(Node):
    def __init__(self):
        super().__init__('imu_ros_probe_node')

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.last_time = time.time()
        self.msg_count = 0

        self.create_subscription(
            Imu,
            '/ros_robot_controller/imu_raw',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg: Imu):
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z

        self.gx = msg.angular_velocity.x
        self.gy = msg.angular_velocity.y
        self.gz = msg.angular_velocity.z

        # 简单用加速度估算 pitch / roll
        try:
            self.roll = math.degrees(math.atan2(self.ay, self.az))
            self.pitch = math.degrees(math.atan2(-self.ax, math.sqrt(self.ay**2 + self.az**2)))
        except Exception:
            pass

        self.msg_count += 1


def ros_spin_thread(node):
    rclpy.spin(node)


# 1) 创建 puppy
puppy = HiwonderPuppy(
    setServoPulse=setServoPulse,
    servoParams=PWMServoParams(),
    dof='8'
)
print("✅ HiwonderPuppy 创建成功")

# 2) 初始化 ROS
rclpy.init()
mpu = MPU6050()
print("✅ MPU6050 ROS 节点创建成功")

# 3) 把 IMU 挂到 puppy 上
puppy.imu = mpu
print("✅ puppy.imu 挂接成功")

# 4) 启动 ROS spin 线程
t = threading.Thread(target=ros_spin_thread, args=(mpu,), daemon=True)
t.start()
print("✅ ROS spin 线程已启动")

# 5) 等待一会看是否收到 IMU
for i in range(10):
    time.sleep(0.5)
    print(
        f"[{i}] imu_count={mpu.msg_count}, "
        f"ax={mpu.ax:.3f}, ay={mpu.ay:.3f}, az={mpu.az:.3f}, "
        f"pitch={mpu.pitch:.2f}, roll={mpu.roll:.2f}"
    )

print("=== ROS IMU 探针测试结束 ===")

try:
    mpu.destroy_node()
except Exception:
    pass

try:
    if rclpy.ok():
        rclpy.shutdown()
except Exception:
    pass