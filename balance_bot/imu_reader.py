import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)

    def callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        # 绕Y轴(轮轴)的前后倾角才是pitch；原公式算的是绕X轴的roll
        sinp = 2*(w*y - z*x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        self.get_logger().info(f'Pitch: {math.degrees(pitch):.2f} deg')

def main():
    rclpy.init()
    node = ImuReader()
    rclpy.spin(node)
    rclpy.shutdown()
