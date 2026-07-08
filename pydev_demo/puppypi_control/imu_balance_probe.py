import time

from servo_controller import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams

print("=== IMU / 自平衡探针测试开始 ===")

# 1) 创建 puppy
puppy = HiwonderPuppy(
    setServoPulse=setServoPulse,
    servoParams=PWMServoParams(),
    dof='8'
)

print("✅ HiwonderPuppy 创建成功")

# 2) 尝试导入官方 SDK
try:
    from ros_robot_controller_sdk import Board
    print("✅ ros_robot_controller_sdk.Board 导入成功")
except Exception as e:
    print(f"❌ Board 导入失败: {e}")
    raise

# 3) 构造一个最小 IMU 包装类
class MPUProbe:
    def __init__(self, board):
        self.board = board
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self):
        """
        尝试从板卡读取 IMU。
        这里只做探针，不保证接口名一定对。
        如果失败，我们就知道下一步该改哪里。
        """
        try:
            data = self.board.get_imu()
            print("IMU raw:", data)
            return True
        except Exception as e:
            print(f"⚠️ get_imu() 调用失败: {e}")
            return False


# 4) 创建设备板对象
try:
    board = Board()
    print("✅ Board 创建成功")
except Exception as e:
    print(f"❌ Board 创建失败: {e}")
    raise

# 5) 创建 IMU 探针对象
mpu = MPUProbe(board)
print("✅ MPUProbe 创建成功")

# 6) 尝试把 IMU 挂到 puppy 上
try:
    puppy.imu = mpu
    print("✅ puppy.imu 挂接成功")
except Exception as e:
    print(f"❌ puppy.imu 挂接失败: {e}")
    raise

# 7) 尝试读一次 IMU
ok = mpu.update()
print(f"IMU update result = {ok}")

print("=== 探针测试结束 ===")