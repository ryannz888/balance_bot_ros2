"""Drive the robot around on its own, turning away from what is in the way.

This is the layer that makes the perception worth having.  Under manual control
an obstacle guard that only stops is close to pointless -- the operator can see
the obstacle perfectly well and stop sooner.  The guard earns its place by
sitting under a command source that cannot see, which is what this node is.

Publishes to /cmd_vel_raw, so avoidance_guard still limits forward motion
underneath.  The split is deliberate: this node decides where to go and is
allowed to be wrong, while the guard decides what is survivable and is not.  A
bug here should produce silly wandering, not a collision.

Turn direction is taken from the scan rather than fixed.  Always turning the
same way walks a robot into a corner and keeps it there, because the corner is
symmetric and the rule is not.  Comparing clearance either side and turning
toward the open one breaks that symmetry with evidence.

Disabled by default.  Something that drives itself should not start driving
because a launch file ran.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

DRIVE, TURN, BACKUP = 'DRIVE', 'TURN', 'BACKUP'


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cruise_speed_mps', 0.10)
        self.declare_parameter('turn_rate_rps', 0.6)
        self.declare_parameter('backup_speed_mps', 0.08)

        # Turn earlier than the guard stops, so the robot steers around things
        # instead of nosing up to them and halting.  The guard stops at 0.70 m.
        self.declare_parameter('turn_trigger_m', 1.10)
        self.declare_parameter('resume_clear_m', 1.60)
        self.declare_parameter('resume_hold_s', 0.4)

        # Corridor the robot actually occupies, for deciding whether to turn.
        self.declare_parameter('corridor_half_angle_rad', 0.30)
        self.declare_parameter('min_known_bearings', 4)

        # Turning forever means the way out is not visible from here.
        self.declare_parameter('turn_timeout_s', 6.0)
        self.declare_parameter('backup_duration_s', 1.5)

        self.state = TURN            # look before moving
        self.state_since = None
        self.clear_since = None
        self.turn_sign = 1.0
        self.nearest = float('nan')
        self.left_clear = self.right_clear = float('nan')
        self.have_scan = False
        self.last_logged = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 1)
        self.state_pub = self.create_publisher(String, '/explorer/state', 5)
        self.create_subscription(LaserScan, '/obstacle/scan', self._scan_cb, 5)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'explorer ready, disabled -- enable with: '
            'ros2 param set /explorer enabled true')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_cb(self, msg: LaserScan):
        half = self.get_parameter('corridor_half_angle_rad').value
        nearest = float('inf')
        known = 0
        left_min = right_min = float('inf')
        left_n = right_n = 0
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if r != r:                       # unknown bearing, no evidence
                continue
            if abs(ang) <= half:
                known += 1
                nearest = min(nearest, r)
            # Side clearance uses the whole scan, not just the corridor: the
            # question is which way is open, and that is answered off to the
            # sides.
            if ang > 0.0:
                left_min = min(left_min, r)
                left_n += 1
            elif ang < 0.0:
                right_min = min(right_min, r)
                right_n += 1

        self.have_scan = known >= self.get_parameter('min_known_bearings').value
        self.nearest = nearest
        self.left_clear = left_min if left_n else float('nan')
        self.right_clear = right_min if right_n else float('nan')

    def _enter(self, state):
        if state != self.state:
            self.state = state
            self.state_since = self._now()
            self.clear_since = None

    def _tick(self):
        now = self._now()
        if self.state_since is None:
            self.state_since = now

        enabled = self.get_parameter('enabled').value
        cmd = Twist()

        if not enabled:
            self.pub.publish(cmd)
            self._report('DISABLED')
            return

        if not self.have_scan:
            # No usable scan is not a reason to drive blind; the guard would
            # crawl, but there is no point steering on nothing.
            self.pub.publish(cmd)
            self._report('NO_SCAN')
            return

        trigger = self.get_parameter('turn_trigger_m').value
        resume = self.get_parameter('resume_clear_m').value
        hold = self.get_parameter('resume_hold_s').value

        if self.state == DRIVE:
            if self.nearest < trigger:
                # Turn toward whichever side has more room.  NaN means that
                # side is unknown, which is not a reason to prefer it.
                lc = self.left_clear if self.left_clear == self.left_clear else -1.0
                rc = self.right_clear if self.right_clear == self.right_clear else -1.0
                self.turn_sign = 1.0 if lc >= rc else -1.0
                self._enter(TURN)
            else:
                cmd.linear.x = self.get_parameter('cruise_speed_mps').value

        elif self.state == TURN:
            cmd.angular.z = self.turn_sign * self.get_parameter('turn_rate_rps').value
            if self.nearest >= resume:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= hold:
                    self._enter(DRIVE)
            else:
                self.clear_since = None
                if now - self.state_since > self.get_parameter('turn_timeout_s').value:
                    # A full sweep found nothing; the opening is not visible
                    # from this spot, so change the spot.
                    self._enter(BACKUP)

        elif self.state == BACKUP:
            cmd.linear.x = -self.get_parameter('backup_speed_mps').value
            if now - self.state_since > self.get_parameter('backup_duration_s').value:
                self.turn_sign = -self.turn_sign     # the last way did not work
                self._enter(TURN)

        self.pub.publish(cmd)
        self._report(self.state)

    def _report(self, label):
        near = f'{self.nearest:.2f}' if math.isfinite(self.nearest) else '--'
        lc = f'{self.left_clear:.2f}' if math.isfinite(self.left_clear) else '--'
        rc = f'{self.right_clear:.2f}' if math.isfinite(self.right_clear) else '--'
        text = f'{label} near={near} L={lc} R={rc}'
        if label != self.last_logged:
            self.last_logged = label
            self.get_logger().info(text)
        m = String()
        m.data = text
        self.state_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Explorer())
    rclpy.shutdown()
