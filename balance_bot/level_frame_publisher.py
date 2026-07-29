"""Publish a gravity-aligned frame at the robot origin.

The TF tree roots at base_link, which means every consumer implicitly treats the
body as level.  On a balancing robot that is never true: the body sits about 1
deg either side of its balance point standing still, and the outer loop tilts it
up to 4 deg to drive.  Depth data therefore arrives in a frame that is rocking,
and a level floor looks tilted -- at 1 m, 4 deg is 7 cm of apparent height,
which is the difference between "floor" and "obstacle".

This publishes base_link -> base_link_level, co-located with base_link but with
roll and pitch removed and yaw kept, so X still points where the robot faces.
Transform depth points into base_link_level and heights are measured against
gravity rather than against the chassis.

Deliberately NOT published as odom -> base_link.  That frame is supposed to be a
world-fixed odometry frame, and this node has no position estimate -- calling it
odom while leaving translation at zero would misrepresent it to anything that
later expects real odometry.  Wheel odometry is separate work, and on this
machine it needs a correction anyway: the encoders measure wheel rotation
relative to a body that is itself pitching, so ground distance is not simply
wheel angle times radius.

The IMU is mounted rpy="0 0 0" relative to base_link (see the URDF), so its
reported orientation is the body's orientation directly, with no extra rotation
to compose.  If the IMU is ever remounted, that assumption breaks and this node
must compose the static transform.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_inverse(x, y, z, w):
    """Conjugate; the IMU quaternion is already unit length."""
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


class LevelFramePublisher(Node):
    def __init__(self):
        super().__init__('level_frame_publisher')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'base_link_level')
        # The IMU runs near 145 Hz.  TF at that rate is pointless and, on a
        # link this project has already saturated once, actively harmful.
        self.declare_parameter('publish_rate_hz', 30.0)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        rate = self.get_parameter('publish_rate_hz').value
        self.min_period = 1.0 / rate if rate > 0.0 else 0.0

        self.br = TransformBroadcaster(self)
        self.last_pub = None
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.get_logger().info(
            f'{self.parent} -> {self.child}, roll/pitch removed, at {rate:.0f} Hz')

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_pub is not None and (t - self.last_pub) < self.min_period:
            return
        self.last_pub = t

        q = msg.orientation
        # Where the level frame sits in the world: same yaw, no tilt.
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        half = 0.5 * yaw
        q_world_level = (0.0, 0.0, math.sin(half), math.cos(half))
        # base_link -> level = (world -> base_link)^-1 * (world -> level)
        q_rel = quat_multiply(quat_inverse(q.x, q.y, q.z, q.w), q_world_level)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.parent
        tf.child_frame_id = self.child
        # Co-located: this rotation carries no position information.
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q_rel[0]
        tf.transform.rotation.y = q_rel[1]
        tf.transform.rotation.z = q_rel[2]
        tf.transform.rotation.w = q_rel[3]
        self.br.sendTransform(tf)


def main():
    rclpy.init()
    rclpy.spin(LevelFramePublisher())
    rclpy.shutdown()
