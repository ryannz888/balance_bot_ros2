"""Measure the PWM at which each elevated wheel first moves.

Keep both wheels off the ground. Each pulse is followed by a zero-command
settle period, so this tool is safe to use before closed-loop tuning.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class DeadbandMeasurer(Node):
    def __init__(self):
        super().__init__('motor_deadband_measurer')
        self.latest_enc = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/wheel/encoders', self._encoder_cb, 10)

    def _encoder_cb(self, msg):
        try:
            left, right = msg.data.split(',')
            self.latest_enc = (int(left), int(right))
        except ValueError:
            pass

    def publish_pwm(self, pwm):
        msg = Twist()
        msg.linear.x = pwm / 100.0
        self.publisher.publish(msg)

    def hold_command(self, pwm, duration):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            self.publish_pwm(pwm)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_encoder(self):
        end = time.monotonic() + 5.0
        while rclpy.ok() and self.latest_enc is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_enc is None:
            raise RuntimeError('No /wheel/encoders message received within 5 seconds')

    def measure(self, levels, pulse_s, settle_s, directions):
        self.wait_for_encoder()
        print('pwm,delta_left,delta_right,normalized_delta')
        for pwm in levels:
            if directions == 'positive':
                signed_pwms = (pwm,)
            elif directions == 'negative':
                signed_pwms = (-pwm,)
            else:
                signed_pwms = (pwm, -pwm)
            for signed_pwm in signed_pwms:
                before = self.latest_enc
                self.hold_command(signed_pwm, pulse_s)
                self.publish_pwm(0.0)
                self.hold_command(0.0, settle_s)
                after = self.latest_enc
                dl = after[0] - before[0]
                dr = after[1] - before[1]
                normalized = (dl - dr) / 2.0
                print(f'{signed_pwm},{dl},{dr},{normalized:.1f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--levels', default='10,15,20,25,30,35,40,45,50')
    parser.add_argument('--pulse', type=float, default=0.25)
    parser.add_argument('--settle', type=float, default=0.35)
    parser.add_argument('--directions', choices=('both', 'positive', 'negative'), default='both')
    args = parser.parse_args()
    levels = [int(value) for value in args.levels.split(',')]

    rclpy.init()
    node = DeadbandMeasurer()
    try:
        node.measure(levels, args.pulse, args.settle, args.directions)
    finally:
        node.publish_pwm(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
