"""Turn depth frames into a forward obstacle scan, in gravity-aligned coordinates.

Publishes a LaserScan rather than a point cloud.  A 640x480 float32 depth image
is about 29 MB/s, which this project has already measured as enough to saturate
the link it runs DDS over; a scan of a few hundred ranges is small enough to
send anywhere and is what a costmap would want later anyway.

Height is measured against gravity, not against the chassis.  The body is never
level -- it rocks about a degree standing and the outer loop tilts it up to four
to drive -- and at 1 m four degrees is 7 cm of apparent height, which is the
whole difference between floor and obstacle.  The attitude used is the IMU
sample nearest the frame, which after fixing the driver's clock source lands
within about 80 ms.

The important design decision is what to do with pixels that returned nothing.
The Astra reports zero for anything nearer than ~0.6 m, anything too far, and
anything that absorbed or scattered the pattern -- dark, shiny, or steeply
angled surfaces.  Those are not free space, they are absence of evidence, and a
bin dominated by them is published as NaN rather than as clear.  Treating them
as clear is how a robot drives into a black chair leg.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32


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


def yaw_of(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_arrays(q, vx, vy, vz):
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


class ObstacleScan(Node):
    def __init__(self):
        super().__init__('obstacle_scan')
        # Geometry, matching the URDF.  Changing the mounting means changing
        # these; nothing here reads the TF tree, to keep the hot path cheap.
        self.declare_parameter('camera_xyz', [0.022, 0.0059, 0.0594])
        self.declare_parameter('wheel_radius_m', 0.0325)

        # Full resolution is 307k points per frame at 24 Hz.  The camera was
        # measured to cost the control loops nothing; doing 24 Hz of numpy over
        # every pixel would spend that margin for detail an obstacle check does
        # not use.  Step 4 leaves 160x120, still ~50 samples per angular bin.
        self.declare_parameter('pixel_step', 4)
        self.declare_parameter('num_bins', 64)

        # A point counts as an obstacle if it stands above the floor but low
        # enough that the robot would actually hit it.  Anything higher passes
        # overhead and is not this robot's problem.
        self.declare_parameter('obstacle_min_height_m', 0.03)
        self.declare_parameter('obstacle_max_height_m', 0.50)
        self.declare_parameter('min_range_m', 0.35)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('half_width_m', 0.20)

        # A bin whose pixels were mostly invalid is unknown, not clear.
        self.declare_parameter('min_valid_fraction', 0.10)
        self.declare_parameter('attitude_max_age_s', 0.5)

        self.fx = self.fy = None
        self.cx = self.cy = None
        self.imu = None
        self.imu_t = None
        self._warned_no_imu = False

        self.scan_pub = self.create_publisher(LaserScan, '/obstacle/scan', 5)
        self.near_pub = self.create_publisher(Float32, '/obstacle/nearest', 5)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 20)
        self.create_subscription(Image, '/depth/image', self._depth_cb, 2)
        from sensor_msgs.msg import CameraInfo
        self.create_subscription(CameraInfo, '/depth/camera_info', self._info_cb, 5)
        self.get_logger().info('obstacle_scan waiting for depth and camera_info')

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f'intrinsics fx={self.fx:.1f} fy={self.fy:.1f} '
                f'c=({self.cx:.1f},{self.cy:.1f})')

    def _imu_cb(self, msg):
        q = msg.orientation
        self.imu = (q.x, q.y, q.z, q.w)
        self.imu_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _depth_cb(self, msg):
        if self.fx is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu is None or abs(t - self.imu_t) > self.get_parameter(
                'attitude_max_age_s').value:
            # Without attitude the heights are meaningless.  Publishing a scan
            # anyway would look like "nothing ahead".
            if not self._warned_no_imu:
                self._warned_no_imu = True
                self.get_logger().warn(
                    'no fresh attitude; not publishing a scan (silence is safer '
                    'than a scan that reads clear)')
            return
        self._warned_no_imu = False

        step = max(1, int(self.get_parameter('pixel_step').value))
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        d = d[::step, ::step]
        h, w = d.shape
        valid = np.isfinite(d) & (d > 0.0)

        u = (np.arange(w) * step - self.cx) / self.fx
        v = (np.arange(h) * step - self.cy) / self.fy
        z = np.where(valid, d, 0.0)
        x = u[None, :] * z
        y = v[:, None] * z
        # Unit-depth ray for every pixel, valid or not.  A pixel's bearing is
        # set by the optics, not by whether the sensor got a return, and the
        # denominator of "how much of this bin did we actually see" has to
        # count pixels that looked and came back empty.  Deriving bearing from
        # the measured point instead gives every invalid pixel the same bogus
        # bearing -- they all collapse to one bin and the valid fraction
        # everywhere else is computed against nothing.
        rx = np.broadcast_to(np.ones((1, w)), (h, w))
        ry = np.broadcast_to(-u[None, :], (h, w))
        rz = np.broadcast_to(-v[:, None], (h, w))

        # optical (X right, Y down, Z fwd) -> body (X fwd, Y left, Z up)
        cam = self.get_parameter('camera_xyz').value
        fwd = z + cam[0]
        left = -x + cam[1]
        up = -y + cam[2]

        q_body = self.imu
        half = 0.5 * yaw_of(*q_body)
        q_level = (0.0, 0.0, math.sin(half), math.cos(half))
        q_rel = quat_multiply(quat_inverse(*q_body), q_level)
        fwd, left, up = rotate_arrays(quat_inverse(*q_rel), fwd, left, up)
        height = up + self.get_parameter('wheel_radius_m').value
        ray_f, ray_l, _ = rotate_arrays(quat_inverse(*q_rel), rx, ry, rz)

        lo = self.get_parameter('obstacle_min_height_m').value
        hi = self.get_parameter('obstacle_max_height_m').value
        rmin = self.get_parameter('min_range_m').value
        rmax = self.get_parameter('max_range_m').value
        halfw = self.get_parameter('half_width_m').value
        nbins = int(self.get_parameter('num_bins').value)

        rng = np.hypot(fwd, left)
        in_front = valid & (fwd > 0.0) & (rng >= rmin) & (rng <= rmax)
        is_obs = in_front & (height >= lo) & (height <= hi)

        # Bin by the pixel's own bearing, in the level frame so it does not
        # swing with the body.
        ang = np.arctan2(ray_l, np.maximum(ray_f, 1e-6))
        amax = max(math.atan2(halfw, rmin), 0.5)
        edges = np.linspace(-amax, amax, nbins + 1)
        idx = np.clip(np.digitize(ang, edges) - 1, 0, nbins - 1)

        ranges = np.full(nbins, float('nan'), dtype=np.float32)
        min_frac = self.get_parameter('min_valid_fraction').value
        within = (ang >= -amax) & (ang <= amax)
        flat_idx = idx.ravel()
        flat_within = within.ravel()
        flat_valid = valid.ravel()
        flat_obs = is_obs.ravel()
        flat_rng = rng.ravel()

        counts = np.bincount(flat_idx[flat_within], minlength=nbins)
        valids = np.bincount(flat_idx[flat_within & flat_valid], minlength=nbins)
        frac = np.where(counts > 0, valids / np.maximum(counts, 1), 0.0)

        obs_idx = flat_idx[flat_obs]
        obs_rng = flat_rng[flat_obs]
        if obs_idx.size:
            order = np.lexsort((obs_rng, obs_idx))
            oi, orng = obs_idx[order], obs_rng[order]
            first = np.ones(oi.size, dtype=bool)
            first[1:] = oi[1:] != oi[:-1]
            ranges[oi[first]] = orng[first]

        # Enough evidence and nothing found means clear; not enough evidence
        # stays NaN.
        clear = np.isnan(ranges) & (frac >= min_frac)
        ranges[clear] = float('inf')

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'base_link_level'
        scan.angle_min = -amax
        scan.angle_max = amax
        scan.angle_increment = (2.0 * amax) / nbins
        scan.range_min = rmin
        scan.range_max = rmax
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        # NaN and inf are not interchangeable here.  inf means every bin was
        # seen and none held an obstacle; NaN means there was not enough
        # evidence to say.  Collapsing them -- which an earlier version did by
        # filtering on isfinite alone -- reports "nothing ahead" for a frame
        # that saw nothing at all, and is exactly how a robot drives into a
        # dark obstacle it never registered.
        obstacles = ranges[np.isfinite(ranges)]
        near = Float32()
        if obstacles.size:
            near.data = float(obstacles.min())
        elif np.isinf(ranges).any():
            near.data = float('inf')          # looked, and it is clear
        else:
            near.data = float('nan')          # did not see enough to say
        self.near_pub.publish(near)


def main():
    rclpy.init()
    rclpy.spin(ObstacleScan())
    rclpy.shutdown()
