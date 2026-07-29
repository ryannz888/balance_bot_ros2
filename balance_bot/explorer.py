"""Drive the robot around on its own, turning away from what is in the way.

This is the layer that makes the perception worth having.  Under manual control
an obstacle guard that only stops is close to pointless -- the operator can see
the obstacle perfectly well and stop sooner.  The guard earns its place by
sitting under a command source that cannot see, which is what this node is.

Publishes to /cmd_vel_raw, so avoidance_guard still limits forward motion
underneath.  The split is deliberate: this node decides where to go and is
allowed to be wrong, while the guard decides what is survivable and is not.  A
bug here should produce silly wandering, not a collision.

Steering follows the widest gap rather than treating the world as blocked or
clear.  The first version turned whenever anything entered a fixed corridor,
which stopped the robot dead in front of a doorway it could have driven
straight through.  Now every bearing is tested for whether the robot would
actually fit through it, contiguous passable bearings are grouped, and the
widest group wide enough for the robot is steered toward.  A gap only has to be
robot-width plus clearance to be worth taking.

Turn direction, when no gap exists at all, still comes from the scan rather than
being fixed.  Always turning the same way walks a robot into a corner and keeps
it there, because the corner is symmetric and the rule is not.

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

        # Turn later than before.  1.10 m had the robot stopping while still
        # far from anything, because the old corridor flared with distance.
        self.declare_parameter('turn_trigger_m', 0.85)
        self.declare_parameter('resume_clear_m', 1.10)
        self.declare_parameter('resume_hold_s', 0.3)

        # The strip the robot occupies, in metres.  Not an angle: an angular
        # corridor is 0.62 m wide at 1 m and 1.86 m at 3 m against a 0.22 m
        # track, so distant things well off to the side read as obstacles.
        self.declare_parameter('corridor_half_width_m', 0.18)
        self.declare_parameter('min_known_bearings', 4)

        # A gap must fit the robot with clearance to be worth aiming at.
        self.declare_parameter('gap_min_width_m', 0.36)
        self.declare_parameter('gap_lookahead_m', 1.20)
        # How hard to steer toward the chosen gap while still moving.
        self.declare_parameter('gap_steer_gain', 1.2)

        # Turning forever means the way out is not visible from here.
        self.declare_parameter('turn_timeout_s', 6.0)
        self.declare_parameter('backup_duration_s', 1.5)

        self.state = TURN            # look before moving
        self.state_since = None
        self.clear_since = None
        self.turn_sign = 1.0
        self.nearest = float('nan')
        self.left_clear = self.right_clear = float('nan')
        self.gap_bearing = None
        self.gap_width = 0.0
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
        halfw = self.get_parameter('corridor_half_width_m').value
        look = self.get_parameter('gap_lookahead_m').value
        need = self.get_parameter('gap_min_width_m').value

        nearest = float('inf')
        known = 0
        left_min = right_min = float('inf')
        left_n = right_n = 0
        passable = []          # (bearing, passable?) for gap grouping

        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if r != r:                       # unknown: no evidence either way
                passable.append((ang, False))
                continue
            if math.isinf(r):
                known += 1
                passable.append((ang, True))
                if ang > 0.0:
                    left_n += 1
                elif ang < 0.0:
                    right_n += 1
                continue

            lateral = r * math.sin(ang)
            forward = r * math.cos(ang)
            # In the strip the robot occupies, measured across, not as an angle.
            if abs(lateral) <= halfw and forward > 0.0:
                known += 1
                nearest = min(nearest, forward)
            # A bearing is passable if driving that way stays clear far enough
            # to be worth committing to.
            passable.append((ang, r >= look))
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
        self.gap_bearing, self.gap_width = self._widest_gap(passable, look, need)

    @staticmethod
    def _widest_gap(passable, look, need):
        """Widest run of passable bearings the robot would fit through.

        Width is measured in metres at the lookahead distance rather than in
        bearings, because a run of bearings is a different physical size
        depending on how far away it is.  A doorway that is plainly wide enough
        at 1 m occupies few enough bearings at 3 m to look like noise.
        """
        best_mid, best_w = None, 0.0
        start = None
        for i, (ang, ok) in enumerate(passable + [(0.0, False)]):
            if ok and start is None:
                start = i
            elif not ok and start is not None:
                a0 = passable[start][0]
                a1 = passable[i - 1][0]
                width = 2.0 * look * math.sin(max(0.0, (a1 - a0)) / 2.0)
                if width > best_w:
                    best_w, best_mid = width, 0.5 * (a0 + a1)
                start = None
        return (best_mid, best_w) if best_w >= need else (None, best_w)

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
                lc = self.left_clear if self.left_clear == self.left_clear else -1.0
                rc = self.right_clear if self.right_clear == self.right_clear else -1.0
                self.turn_sign = 1.0 if lc >= rc else -1.0
                self._enter(TURN)
            else:
                cmd.linear.x = self.get_parameter('cruise_speed_mps').value
                # Steer toward the widest gap while still driving, instead of
                # waiting until something blocks and only then reacting.  This
                # is what lets it aim at a doorway rather than stop in front of
                # the wall beside it.
                if self.gap_bearing is not None:
                    gain = self.get_parameter('gap_steer_gain').value
                    rate = self.get_parameter('turn_rate_rps').value
                    cmd.angular.z = max(-rate, min(rate, gain * self.gap_bearing))

        elif self.state == TURN:
            # If a gap has come into view, turn toward it rather than
            # continuing to sweep blindly in the chosen direction.
            if self.gap_bearing is not None:
                self.turn_sign = 1.0 if self.gap_bearing > 0.0 else -1.0
            cmd.angular.z = self.turn_sign * self.get_parameter('turn_rate_rps').value
            if self.nearest >= resume or self.gap_bearing is not None:
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
        gap = (f'{math.degrees(self.gap_bearing):+.0f}deg/{self.gap_width:.2f}m'
               if self.gap_bearing is not None else 'none')
        text = f'{label} near={near} L={lc} R={rc} gap={gap}'
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
