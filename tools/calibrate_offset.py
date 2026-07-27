"""机械零点校准工具：手扶车子在预估平衡点附近，采集若干秒的原始pitch读数，
给出均值/标准差，作为pitch_offset_deg的候选值。

用法(在树莓派上，先source好ROS2环境，保证/imu/data已在发布)：
    python3 tools/calibrate_offset.py --duration 8

流程：
    1. 运行脚本前先把车用手扶在"手感上最不费力"的平衡点
    2. 脚本开始采集后保持这个姿态不动，直到打印结果
    3. 标准差>2°说明手扶不够稳/没找准平衡点，建议重测一次取更小标准差的结果
"""
import argparse
import math
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class OffsetCalibrator(Node):
    def __init__(self, duration):
        super().__init__('offset_calibrator')
        self.duration = duration
        self.samples = []
        self.start_t = None
        self.done = False
        self.create_subscription(Imu, '/imu/data', self._cb, 10)
        self.get_logger().info(f'开始采集，请把车扶在预估平衡点，保持{duration:.0f}秒...')

    def _cb(self, msg):
        if self.done:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start_t is None:
            self.start_t = t
        if t - self.start_t > self.duration:
            self.done = True
            return
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        # 与balance_controller.py中_imu_cb的公式保持一致
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.degrees(math.asin(sinp))
        self.samples.append(pitch)

    def report(self):
        n = len(self.samples)
        if n < 2:
            print('采样太少，检查/imu/data是否在发布')
            return
        mean = statistics.mean(self.samples)
        std = statistics.pstdev(self.samples)
        print(f'\n采样{n}个点，时长约{self.duration:.0f}秒')
        print(f'pitch均值 = {mean:.2f}°   标准差 = {std:.2f}°   '
              f'min={min(self.samples):.2f}  max={max(self.samples):.2f}')
        if std > 2.0:
            print('标准差偏大(>2°)，说明手扶不够稳/没找准平衡点，建议重测')
        print(f'\n建议的 pitch_offset_deg 候选值: {mean:.2f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=8.0, help='采集时长(秒)，默认8秒')
    args = ap.parse_args()

    rclpy.init()
    node = OffsetCalibrator(args.duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
