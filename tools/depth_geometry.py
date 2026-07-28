"""Project a saved depth frame into robot coordinates and describe the geometry.

Answers the questions an obstacle check depends on, before any avoidance code
exists: how high above the ground is the camera really looking, where does the
floor land in the image, and how much of the near field carries any data at all.

Heights come out relative to the ground plane, so a point at 0.00 m is floor and
anything standing up reads positive.  The chain is
    pixel -> camera optical frame -> camera_link -> base_link -> ground
using the URDF offsets, then the body attitude from the frame's .json sidecar.

Applying that attitude is not optional.  Run against a frame captured while the
body was tilted, assuming level instead, and a quarter of the points land below
the floor -- which is how this script was first written and what proved the
level-frame work was needed.  If the sidecar has no attitude the script says so
and refuses to report geometry.

    python3 depth_geometry.py <frame.npy>
"""
import json
import math
import os
import sys

import numpy as np

FX = FY = 570.3422241210938
CX, CY = 319.5, 239.5

# From src/balance_bot_description/urdf/balance_bot_mesh.urdf
CAM_XYZ = (0.022, 0.0059, 0.0594)   # camera_link relative to base_link
WHEEL_RADIUS = 0.0325               # base_link sits on the wheel axle


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'results/depth/depth_02.npy'
    d = np.load(path)
    h, w = d.shape
    valid = d > 0

    side = os.path.splitext(path)[0] + '.json'
    att = None
    if os.path.exists(side):
        with open(side) as f:
            att = json.load(f).get('attitude')
    if att is None:
        print(f'no attitude recorded for {os.path.basename(path)}.')
        print('geometry needs it: this robot is never level, so a level')
        print('assumption puts real points below the floor.  Recapture with')
        print('/imu/data running.')
        return

    # Optical frame: X right, Y down, Z forward.
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = d
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY

    # optical -> camera_link (URDF rpy -1.5708 0 -1.5708): X fwd, Y left, Z up.
    fwd, left, up = z, -x, -y
    # camera_link -> base_link.
    fwd = fwd + CAM_XYZ[0]
    left = left + CAM_XYZ[1]
    up = up + CAM_XYZ[2]

    # base_link -> gravity aligned, using the attitude recorded with the frame.
    qx, qy, qz, qw = att['quat_xyzw']
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    ch, sh = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    # q_rel = inv(q) * q_yaw_only, matching level_frame_publisher.
    ix, iy, iz, iw = -qx, -qy, -qz, qw
    rx = iw * 0.0 + ix * ch + iy * sh - iz * 0.0
    ry = iw * 0.0 - ix * 0.0 + iy * ch + iz * sh
    rz = iw * sh + ix * 0.0 - iy * 0.0 + iz * ch
    rw = iw * ch - ix * 0.0 - iy * 0.0 - iz * sh
    # Rotate (fwd, left, up) by the inverse of q_rel into the level frame.
    n = math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw)
    rx, ry, rz, rw = rx / n, ry / n, rz / n, rw / n
    cx_, cy_, cz_, cw_ = -rx, -ry, -rz, rw   # inverse
    # v' = q v q^-1 expanded for a real vector
    def rot(vx, vy, vz):
        tx = 2.0 * (cy_ * vz - cz_ * vy)
        ty = 2.0 * (cz_ * vx - cx_ * vz)
        tz = 2.0 * (cx_ * vy - cy_ * vx)
        return (vx + cw_ * tx + (cy_ * tz - cz_ * ty),
                vy + cw_ * ty + (cz_ * tx - cx_ * tz),
                vz + cw_ * tz + (cx_ * ty - cy_ * tx))
    fwd, left, up = rot(fwd, left, up)
    height = up + WHEEL_RADIUS

    cam_h = CAM_XYZ[2] + WHEEL_RADIUS
    print(f'camera sits {cam_h*100:.1f} cm above the ground when level')
    print(f'body pitch at capture: {att["pitch_deg"]:+.2f} deg')
    print(f'frame {w}x{h}, valid {100*valid.mean():.1f}%')
    print()

    print('height above ground, for valid points (0.00 = floor):')
    hv = height[valid]
    for lo, hi, label in ((-0.05, 0.03, 'floor'), (0.03, 0.10, 'low obstacle'),
                          (0.10, 0.30, 'obstacle'), (0.30, 1.0, 'tall'),
                          (1.0, 9.0, 'above robot')):
        m = (hv >= lo) & (hv < hi)
        print(f'  {lo:+.2f} to {hi:+.2f} m  {label:<13} {m.sum():7d} px '
              f'({100*m.mean():5.1f}% of valid)')
    below = (hv < -0.05).sum()
    print(f'  below -0.05 m   (impossible)  {below:7d} px  <- geometry or tilt error')
    print()

    print('forward distance of points near floor level, by range ring:')
    floorish = valid & (height > -0.05) & (height < 0.05)
    fv = fwd[floorish]
    for lo, hi in ((0.0, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 2.5), (2.5, 4.0), (4.0, 8.0)):
        m = (fv >= lo) & (fv < hi)
        print(f'  {lo:.1f}-{hi:.1f} m : {m.sum():7d} px')
    print()

    print('what a forward obstacle check would have to work with:')
    for reach in (0.5, 1.0, 1.5, 2.0):
        ahead = valid & (fwd < reach) & (np.abs(left) < 0.15) & (height > 0.03)
        print(f'  within {reach:.1f} m ahead, +-15 cm wide, above 3 cm: '
              f'{ahead.sum():6d} px')


if __name__ == '__main__':
    main()
