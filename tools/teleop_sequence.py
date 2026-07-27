"""Drive a scripted velocity sequence on /cmd_vel.

Replaces a shell loop around `ros2 topic pub`.  Each `ros2 topic pub` is a fresh
process that starts publishing before DDS discovery has matched the existing
subscribers, so with VOLATILE durability the first messages are dropped for
whoever has not been discovered yet.  A 3 s command can lose every message that
way -- the first teleop test recorded 34 forward commands in the bag while the
controller's callback never fired once.

This keeps one publisher alive for the whole run and, crucially, blocks until a
subscriber is actually matched before sending anything.

    python3 teleop_sequence.py [linear m/s] [angular rad/s]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

RATE_HZ = 50.0


class Sequencer(Node):
    def __init__(self):
        super().__init__('teleop_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def wait_for_subscriber(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.pub.get_subscription_count() > 0:
                # Matched, but let the discovery handshake settle before the
                # first sample so it is not raced away.
                time.sleep(0.5)
                self.get_logger().info(
                    f'{self.pub.get_subscription_count()} subscriber(s) matched')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error('no /cmd_vel subscriber appeared')
        return False

    def hold(self, linear, angular, seconds):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        n = int(seconds * RATE_HZ)
        self.get_logger().info(f'lin={linear:+.3f} ang={angular:+.3f} for {seconds:.1f}s')
        for _ in range(n):
            self.pub.publish(msg)
            time.sleep(1.0 / RATE_HZ)

    def idle(self, seconds):
        # Deliberately publish nothing: this also exercises the command timeout.
        self.get_logger().info(f'idle {seconds:.1f}s')
        time.sleep(seconds)


def main():
    lin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.08
    ang = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    rclpy.init()
    node = Sequencer()
    if not node.wait_for_subscriber():
        sys.exit(1)

    node.idle(3.0)
    for linear, angular, label in ((lin, 0.0, 'forward'), (-lin, 0.0, 'reverse'),
                                   (0.0, ang, 'yaw left'), (0.0, -ang, 'yaw right')):
        node.get_logger().info(f'--- {label} ---')
        node.hold(linear, angular, 3.0)
        node.idle(6.0)
    node.get_logger().info('sequence done')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
