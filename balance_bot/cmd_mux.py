"""Choose between the autonomous and manual command sources.

Both the explorer and teleop want to drive, and if they publish the same topic
they overwrite each other at whatever rate each happens to run.  This node owns
/cmd_vel_raw and picks: manual if a human is driving, autonomous otherwise.

Detection is by traffic, not by asking whether a gamepad exists.  teleop_twist_joy
publishes only while its deadman button is held, so the arrival of a manual
message already means a hand is on the controls -- there is nothing further to
infer.  Releasing the button stops the traffic, and after a short timeout
autonomy resumes.  The pad can stay powered on and idle without locking the
robot out of exploring, which "manual whenever a gamepad is connected" would do.

Handover is immediate in one direction and delayed in the other, deliberately.
Taking over must be instant because the reason a human grabs the controls is
usually that the robot is about to do something they do not want.  Handing back
waits, because a momentary gap in the stream is not a decision to let go.

The guard still sits downstream of this node, so neither source can exceed what
the obstacle scan permits.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

AUTO, MANUAL = 'AUTO', 'MANUAL'


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        # Long enough to bridge the gap between two joystick messages at 20 Hz,
        # short enough that letting go feels like letting go.
        self.declare_parameter('manual_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.manual = Twist()
        self.auto = Twist()
        self.manual_t = None
        self.auto_t = None
        self.mode = AUTO
        self.last_logged = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 1)
        self.mode_pub = self.create_publisher(String, '/cmd_mux/mode', 5)
        self.create_subscription(Twist, '/cmd_vel_manual', self._manual_cb, 1)
        self.create_subscription(Twist, '/cmd_vel_auto', self._auto_cb, 1)
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'cmd_mux: /cmd_vel_manual (priority) or /cmd_vel_auto -> /cmd_vel_raw')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _manual_cb(self, msg):
        self.manual = msg
        self.manual_t = self._now()

    def _auto_cb(self, msg):
        self.auto = msg
        self.auto_t = self._now()

    def _tick(self):
        now = self._now()
        timeout = self.get_parameter('manual_timeout_s').value
        manual_live = self.manual_t is not None and (now - self.manual_t) <= timeout

        self.mode = MANUAL if manual_live else AUTO
        if manual_live:
            out = self.manual
        elif self.auto_t is not None and (now - self.auto_t) <= 1.0:
            out = self.auto
        else:
            # Neither source is talking.  Publishing zero keeps the command
            # timeout downstream from being the only thing stopping the robot.
            out = Twist()

        self.pub.publish(out)

        if self.mode != self.last_logged:
            self.last_logged = self.mode
            self.get_logger().info(f'{self.mode}')
        m = String()
        m.data = self.mode
        self.mode_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(CmdMux())
    rclpy.shutdown()
