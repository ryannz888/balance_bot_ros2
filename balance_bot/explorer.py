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

DRIVE, TURN, COMMIT, BACKUP = 'DRIVE', 'TURN', 'COMMIT', 'BACKUP'


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
        # Half-width of the strip the robot claims.  0.18 left only 70 mm of
        # clearance either side of the 220 mm track and the robot clipped
        # edges; 0.22 gives 110 mm.  Widening this makes the robot treat more
        # things as in the way, which is the correct direction for clipping.
        self.declare_parameter('corridor_half_width_m', 0.22)
        self.declare_parameter('min_known_bearings', 4)

        # A gap must fit the robot with clearance to be worth aiming at.
        # Must exceed twice the corridor half-width, or the robot would aim
        # at gaps it does not fit through.
        self.declare_parameter('gap_min_width_m', 0.46)
        self.declare_parameter('gap_lookahead_m', 1.20)
        # How hard to steer toward the chosen gap while still moving.
        self.declare_parameter('gap_steer_gain', 1.2)
        # The gap answer jumps frame to frame, because 44% of bearings are
        # unknown and each one flickers between passable and not.  Measured
        # unfiltered: +18, +13, +2, -14, +16, +0 deg inside 0.6 s, which the
        # steering followed directly and drove the balance loop to saturation.
        # Commit to a bearing and only move off it gradually.
        self.declare_parameter('gap_smooth_alpha', 0.25)

        # Sweeping until some direction clears a threshold does not terminate
        # in a tight space, because no direction clears it -- the robot swept
        # 206 deg, backed up 12 cm, and swept again.  Instead sweep a fixed arc
        # while remembering which heading was most open, then commit to that
        # heading.  Picking the best of what was seen always terminates;
        # waiting for something good enough does not.
        self.declare_parameter('sweep_arc_rad', 3.6)      # a bit over half a turn
        self.declare_parameter('commit_tolerance_rad', 0.15)
        self.declare_parameter('backup_duration_s', 2.5)

        self.state = TURN            # look before moving
        self.state_since = None
        self.clear_since = None
        self.turn_sign = 1.0
        self.nearest = float('nan')
        self.left_clear = self.right_clear = float('nan')
        self.gap_bearing = None       # smoothed, what steering follows
        self.gap_raw = None           # this frame's answer
        self.gap_width = 0.0
        self.yaw = None               # absolute heading, for measuring a sweep
        self.sweep_start_yaw = None
        self.sweep_turned = 0.0
        self.best_clear = -1.0        # best openness seen during this sweep
        self.best_yaw = None
        self.have_scan = False
        self.last_logged = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 1)
        self.state_pub = self.create_publisher(String, '/explorer/state', 5)
        self.create_subscription(LaserScan, '/obstacle/scan', self._scan_cb, 5)
        from sensor_msgs.msg import Imu
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
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
        raw, self.gap_width = self._widest_gap(passable, look, need)
        self.gap_raw = raw
        if raw is None:
            self.gap_bearing = None
        elif self.gap_bearing is None:
            self.gap_bearing = raw
        else:
            a = self.get_parameter('gap_smooth_alpha').value
            self.gap_bearing += a * (raw - self.gap_bearing)

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

    def _imu_cb(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _enter(self, state):
        if state != self.state:
            if state != TURN:
                self.sweep_start_yaw = None      # a new sweep starts fresh
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
            # No passable gap is itself the reason to turn.  Waiting for the
            # distance threshold instead let the robot crawl straight at a
            # table leg for three seconds with gap=none the whole time: the
            # guard had already tapered speed to 4.5 cm/s by 0.87 m, while the
            # turn trigger sat at 0.85 m and never fired.
            if self.gap_bearing is None or self.nearest < trigger:
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
            if self.gap_bearing is not None:
                self.turn_sign = 1.0 if self.gap_bearing > 0.0 else -1.0
            cmd.angular.z = self.turn_sign * self.get_parameter('turn_rate_rps').value

            # Track how far this sweep has gone, and which heading looked most
            # open along the way.  Openness is the corridor distance, so it
            # answers the same question driving will ask.
            if self.yaw is not None:
                if self.sweep_start_yaw is None:
                    self.sweep_start_yaw = self.yaw
                    self.sweep_turned = 0.0
                    self.best_clear, self.best_yaw = -1.0, self.yaw
                    self._last_yaw = self.yaw
                else:
                    self.sweep_turned += abs(self._wrap(self.yaw - self._last_yaw))
                    self._last_yaw = self.yaw
                score = self.nearest if math.isfinite(self.nearest) else 99.0
                if score > self.best_clear:
                    self.best_clear, self.best_yaw = score, self.yaw

            gap_ok = self.gap_bearing is not None and self.nearest >= trigger
            if self.nearest >= resume or gap_ok:
                self._enter(DRIVE)
            elif (self.yaw is not None
                  and self.sweep_turned >= self.get_parameter('sweep_arc_rad').value):
                # A full sweep found nothing above threshold.  Rather than
                # sweeping again, go to the best heading it did see.
                self._enter(COMMIT)
            elif self.yaw is None and (
                    now - self.state_since > 6.0):
                self._enter(BACKUP)      # no heading available, fall back

        elif self.state == COMMIT:
            # Rotate onto the most open heading found during the sweep, then
            # drive whatever it turned out to be.  The guard still limits it.
            if self.yaw is None or self.best_yaw is None:
                self._enter(BACKUP)
            else:
                err = self._wrap(self.best_yaw - self.yaw)
                if abs(err) <= self.get_parameter('commit_tolerance_rad').value:
                    self._enter(DRIVE)
                else:
                    rate = self.get_parameter('turn_rate_rps').value
                    cmd.angular.z = math.copysign(rate, err)
                    if now - self.state_since > 8.0:
                        self._enter(BACKUP)   # could not get there; move first

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
