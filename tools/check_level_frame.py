"""Verify that base_link_level really is gravity-aligned.

Reading the base_link -> base_link_level rotation as Euler angles is not a test:
near the tilts this robot reaches when it falls over, those angles are hard to
interpret and pass through gimbal-degenerate regions.

The unambiguous check is physical.  The accelerometer measures gravity in
base_link.  Rotate that vector by the published transform and, if the frame is
correct, it must come out vertical -- (0, 0, ~9.81) -- no matter how the body is
oriented.  Any residual horizontal component is the error, in m/s^2, and
dividing it by g converts straight back to degrees of leftover tilt.

Run it with the robot lying on its side as well as upright: a large tilt is the
strong test, since a bug that leaves a few degrees behind is invisible when the
body is already nearly level.

    python3 check_level_frame.py [seconds]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
G = 9.80665


def quat_inverse(x, y, z, w):
    return -x, -y, -z, w


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(q, v):
    """Rotate vector v by quaternion q (xyzw)."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_multiply(quat_multiply(q, qv), quat_inverse(*q))
    return r[0], r[1], r[2]


def yaw_of(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Checker(Node):
    def __init__(self):
        super().__init__('check_level_frame')
        self.t0 = None
        self.worst = 0.0
        self.n = 0
        self.rejected = 0
        self.resid = []
        self.by_tilt = []
        self.create_subscription(Imu, '/imu/data', self.cb, 10)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
            print(f'{"body tilt":>10} {"gx":>7} {"gy":>7} {"gz":>7} {"residual":>9}')

        q = msg.orientation
        # Same construction as level_frame_publisher.
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        half = 0.5 * yaw
        q_world_level = (0.0, 0.0, math.sin(half), math.cos(half))
        q_rel = quat_multiply(quat_inverse(q.x, q.y, q.z, q.w), q_world_level)

        a = msg.linear_acceleration
        # An accelerometer measures gravity PLUS real acceleration, so it is
        # only a gravity reference while the body is not accelerating.  A
        # balancing robot always is, slightly, and being picked up swamps it
        # entirely -- an unfiltered version of this test reported 67 deg of
        # "frame error" that was really the operator lifting the robot.  Keep
        # only samples whose magnitude is close to g.
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        body_tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, abs(a.z) / max(1e-6, mag)))))
        if abs(mag - G) > 0.30:
            self.rejected += 1
            return

        # The transform maps base_link -> base_link_level, so a vector expressed
        # in base_link is rotated into the level frame by its inverse.
        gx, gy, gz = rotate(quat_inverse(*q_rel), (a.x, a.y, a.z))
        horiz = math.hypot(gx, gy)
        self.worst = max(self.worst, horiz)
        self.resid.append(horiz)
        self.by_tilt.append((body_tilt, horiz))
        self.n += 1

        if self.n % 60 == 1:
            print(f'{body_tilt:9.1f}d {gx:7.2f} {gy:7.2f} {gz:7.2f} {horiz:9.3f}')

        if t - self.t0 > SECONDS:
            self.report()
            raise SystemExit(0)

    def report(self):
        print()
        if not self.resid:
            print(f'no static samples ({self.rejected} rejected) -- hold the robot still')
            return
        r = sorted(self.resid)
        med = r[len(r) // 2]
        p95 = r[int(0.95 * (len(r) - 1))]
        deg = lambda v: math.degrees(math.asin(min(1.0, v / G)))
        print(f'kept {len(r)} static samples, rejected {self.rejected} accelerating ones')
        print(f'residual horizontal gravity: median {med:.3f}, p95 {p95:.3f}, '
              f'max {self.worst:.3f} m/s^2')
        print(f'  as leftover tilt:          median {deg(med):.2f}, p95 {deg(p95):.2f}, '
              f'max {deg(self.worst):.2f} deg')
        print()
        print('residual by body tilt -- the operating regime is the top rows:')
        deg2 = lambda v: math.degrees(math.asin(min(1.0, v / G)))
        for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 45), (45, 90)):
            sel = [h for tt, h in self.by_tilt if lo <= tt < hi]
            if not sel:
                continue
            sel.sort()
            print(f'  body {lo:2d}-{hi:2d} deg  n={len(sel):5d}  '
                  f'median {deg2(sel[len(sel)//2]):5.2f} deg  '
                  f'p95 {deg2(sel[int(0.95*(len(sel)-1))]):5.2f} deg')


def main():
    rclpy.init()
    try:
        rclpy.spin(Checker())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
