#!/usr/bin/env python3
"""静态 camera_info publisher：用 SC132gs 标定文件数据，每秒发一次 left/right。
   用于绕过 mipi_cam 在 default calibration 模式下不发布 camera_info 的问题。"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from sensor_msgs.msg import CameraInfo


# 数据从 /opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml 来
LEFT_K  = [656.7575224009456, 0.0, 636.3995967335052,
           0.0, 656.3766938297126, 540.4756754786636,
           0.0, 0.0, 1.0]
LEFT_D  = [-0.32990876095880534, 0.13662699921032978, -0.00013531813117594095,
           -9.84710031787936e-05, -0.02937890519777009, 0.00014050652399494757,
           4.560677311111928e-05, 4.667085000242849e-05]
LEFT_R  = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
LEFT_P  = [656.7575224009456, 0.0, 636.3995967335052, 0.0,
           0.0, 656.3766938297126, 540.4756754786636, 0.0,
           0.0, 0.0, 1.0, 0.0]

RIGHT_K = [659.0957261498398, 0.0, 632.5403313828651,
           0.0, 658.9494113212542, 539.3174440945205,
           0.0, 0.0, 1.0]
RIGHT_D = [-0.33648211988088883, 0.14640319389383946, -9.945169177922777e-06,
           -0.0002291888331273915, -0.034067021404055015, -4.29001168046378e-05,
           -0.00023103725828151024, 0.00012545114191076973]
RIGHT_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# 右目 P: Tx = -fx_right * baseline = -659.0957 * 0.0792 = -52.20
# baseline 0.0792m = 7.92cm（GS130W 标称基线）
RIGHT_P = [659.0957261498398, 0.0, 632.5403313828651, -52.20,
           0.0, 658.9494113212542, 539.3174440945205, 0.0,
           0.0, 0.0, 1.0, 0.0]

IMG_W = 1280
IMG_H = 1088


def make_info(frame_id, K, D, R, P, stamp_sec=0, stamp_nsec=0):
    info = CameraInfo()
    info.width = IMG_W
    info.height = IMG_H
    info.distortion_model = 'rational_polynomial'
    info.d = D
    info.k = K
    info.r = R
    info.p = P
    info.header.frame_id = frame_id
    info.header.stamp.sec = stamp_sec
    info.header.stamp.nanosec = stamp_nsec
    return info


class StaticCameraInfo(Node):
    def __init__(self):
        super().__init__('static_camera_info')
        # RELIABLE + TRANSIENT_LOCAL：标准 latched topic QoS
        # RELIABLE 发布者能兼容 RELIABLE 和 BEST_EFFORT 两种订阅者
        # TRANSIENT_LOCAL (latched)：晚连接的订阅者也能拿到最近一帧
        stereo_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_l = self.create_publisher(CameraInfo,
            '/image_combine_raw/left/camera_info', stereo_qos)
        self.pub_r = self.create_publisher(CameraInfo,
            '/image_combine_raw/right/camera_info', stereo_qos)
        # 每秒重复发 + 构造函数里立即发一次（确保 stereonet 能收到）
        self.timer = self.create_timer(1.0, self.tick)
        self.tick()

    def tick(self):
        now = self.get_clock().now().to_msg()
        self.pub_l.publish(make_info('sc132gs_left', LEFT_K, LEFT_D, LEFT_R, LEFT_P, now.sec, now.nanosec))
        self.pub_r.publish(make_info('sc132gs_right', RIGHT_K, RIGHT_D, RIGHT_R, RIGHT_P, now.sec, now.nanosec))


def main():
    rclpy.init()
    n = StaticCameraInfo()
    rclpy.spin(n)


if __name__ == '__main__':
    main()
