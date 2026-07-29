"""Limit forward motion when something is in the way.

Sits between whatever is producing velocity commands and the balance
controller: /cmd_vel_raw in, /cmd_vel out.  It only ever reduces the forward
component, never adds motion of its own, so the operator stays in the loop and
an autonomous command source can be added later without touching this node.

Reverse and rotation are never blocked.  A guard that can stop the robot but
not let it back out of a corner is a guard that strands it.

The stopping distance is set by the sensor, not by the brakes.  Braking from
the fastest commanded speed measured 0.10 m, but the Astra sees nothing closer
than about 0.6 m, so an obstacle being approached vanishes from the scan at the
exact moment it matters.  Stopping at 0.7 m keeps the decision inside the range
where evidence still exists.  For the same reason, leaving STOP requires
sustained clearance rather than a single clear frame: one dropout must not read
as "it moved away".

Unknown is not clear.  Roughly 44% of bearings return no data on this machine --
the camera sits 9.2 cm off the ground and sees most surfaces at a grazing angle.
Refusing to move whenever anything is unknown would immobilise the robot, so
unknown bearings are ignored while enough others carry evidence, and only a scan
that is almost entirely unknown is treated as blind.  That threshold is the
explicit trade this node exists to make.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

CLEAR, SLOW, STOP, BLIND, STALE = 'CLEAR', 'SLOW', 'STOP', 'BLIND', 'STALE'


class AvoidanceGuard(Node):
    def __init__(self):
        super().__init__('avoidance_guard')
        # 0.60 sits just above the sensor floor: the Astra's nearest valid
        # returns measure 0.67 m, so deciding any later means deciding blind.
        # Braking needs 0.10 m, so the margin is in the sensor, not the brakes.
        self.declare_parameter('stop_distance_m', 0.60)
        self.declare_parameter('slow_distance_m', 1.20)
        # Hysteresis: how far things must be, and for how long, before forward
        # motion is allowed again after a stop.
        self.declare_parameter('release_distance_m', 0.80)
        self.declare_parameter('release_hold_s', 0.6)
        # The corridor is the strip the robot physically occupies, not a wedge.
        # It was a fixed half-angle, which is wrong in a way that gets worse
        # with distance: +-0.30 rad spans 0.62 m at 1 m and 1.86 m at 3 m
        # against a 0.22 m track, so something 0.6 m off to one side at 2 m
        # blocked forward motion despite being nowhere near the path.  That is
        # what made the robot stop while still far from anything.
        self.declare_parameter('corridor_half_width_m', 0.18)
        # A scan with fewer than this many usable bearings in the corridor is
        # not evidence of anything.
        self.declare_parameter('min_known_bearings', 4)
        self.declare_parameter('blind_speed_scale', 0.35)
        # Tapering toward zero commands speeds the drivetrain cannot deliver.
        # Below roughly this, static friction wins and the wheels do not turn
        # at all -- the robot leans into the obstacle and the balance loop
        # fights itself.  Measured: 0.045 m/s commanded for three seconds with
        # the range reading 0.86 -> 0.88 m, i.e. not moving, ending in a fall
        # against a table leg.  Taper down to this floor, then to zero at STOP.
        self.declare_parameter('min_move_speed_mps', 0.07)
        self.declare_parameter('scan_timeout_s', 0.5)

        self.state = BLIND
        self.nearest = float('nan')
        self.scan_t = None
        self.clear_since = None
        self.last_state_logged = None

        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.state_pub = self.create_publisher(String, '/obstacle/guard_state', 5)
        self.create_subscription(LaserScan, '/obstacle/scan', self._scan_cb, 5)
        self.create_subscription(Twist, '/cmd_vel_raw', self._cmd_cb, 1)
        # Commands may stop arriving while the scan keeps coming; publish state
        # regardless so the mode is observable without driving.
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            'avoidance_guard: /cmd_vel_raw -> /cmd_vel, forward limited by /obstacle/scan')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_cb(self, msg: LaserScan):
        self.scan_t = self._now()
        halfw = self.get_parameter('corridor_half_width_m').value

        nearest = float('inf')
        known = 0
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if r != r:              # NaN: no evidence for this bearing
                continue
            if math.isinf(r):
                known += 1          # seen, and clear
                continue
            # Where the return actually sits, rather than which way it lies.
            lateral = r * math.sin(ang)
            forward = r * math.cos(ang)
            if abs(lateral) > halfw or forward <= 0.0:
                continue            # beside the robot, not in front of it
            known += 1
            if forward < nearest:
                nearest = forward   # distance along the path, not slant range
        self.nearest = nearest

        if known < self.get_parameter('min_known_bearings').value:
            self.state = BLIND
            self.clear_since = None
            return

        stop_d = self.get_parameter('stop_distance_m').value
        slow_d = self.get_parameter('slow_distance_m').value
        rel_d = self.get_parameter('release_distance_m').value
        hold = self.get_parameter('release_hold_s').value

        if self.state == STOP:
            # Leaving STOP needs sustained clearance, not one lucky frame.
            if nearest >= rel_d:
                if self.clear_since is None:
                    self.clear_since = self.scan_t
                elif self.scan_t - self.clear_since >= hold:
                    self.state = SLOW if nearest < slow_d else CLEAR
                    self.clear_since = None
            else:
                self.clear_since = None
            return

        self.clear_since = None
        if nearest < stop_d:
            self.state = STOP
        elif nearest < slow_d:
            self.state = SLOW
        else:
            self.state = CLEAR

    def _effective_state(self):
        if self.scan_t is None or (
                self._now() - self.scan_t) > self.get_parameter('scan_timeout_s').value:
            return STALE
        return self.state

    def _scale_for(self, state):
        if state == CLEAR:
            return 1.0
        if state == SLOW:
            # Taper linearly between stop and slow distance rather than
            # switching, so the setpoint stays smooth -- a step in commanded
            # speed is what the outer loop is least able to follow.
            stop_d = self.get_parameter('stop_distance_m').value
            slow_d = self.get_parameter('slow_distance_m').value
            if not math.isfinite(self.nearest) or slow_d <= stop_d:
                return 1.0
            frac = (self.nearest - stop_d) / (slow_d - stop_d)
            return max(0.0, min(1.0, frac))
        if state == BLIND:
            return self.get_parameter('blind_speed_scale').value
        return 0.0                      # STOP and STALE

    def _cmd_cb(self, msg: Twist):
        state = self._effective_state()
        out = Twist()
        # Rotation always passes: it is how the robot gets out of a corner.
        out.angular.z = msg.angular.z
        if msg.linear.x <= 0.0:
            # Reversing moves away from what is being avoided.  The camera
            # faces forward and has nothing to say about it either way.
            out.linear.x = msg.linear.x
        else:
            want = msg.linear.x * self._scale_for(state)
            floor = self.get_parameter('min_move_speed_mps').value
            # Anything between zero and the floor is a speed this drivetrain
            # will not actually produce; asking for it stalls the wheels while
            # the body keeps leaning.  Round it to something that moves, or to
            # a genuine stop.
            if 0.0 < want < floor:
                want = floor if want >= 0.5 * floor else 0.0
            out.linear.x = min(want, msg.linear.x)
        self.pub.publish(out)

    def _publish_state(self):
        state = self._effective_state()
        if state != self.last_state_logged:
            self.last_state_logged = state
            d = ('%.2f m' % self.nearest) if math.isfinite(self.nearest) else 'no obstacle'
            self.get_logger().info(f'{state}  (nearest {d})')
        m = String()
        m.data = f'{state} {self.nearest:.2f}' if math.isfinite(self.nearest) else state
        self.state_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(AvoidanceGuard())
    rclpy.shutdown()
