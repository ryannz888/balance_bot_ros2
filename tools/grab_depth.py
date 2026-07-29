"""Save depth frames as viewable PNGs, plus the numbers behind them.

A 32FC1 depth image is metres per pixel, so opening it as an image shows black:
the values are 0-4 where a viewer expects 0-255.  This normalises to a colour
ramp for looking at, and prints the statistics that actually matter for tuning
an obstacle check -- how much of the frame returned nothing, and how the valid
range is distributed.

Invalid pixels are the important part and are easy to miss visually.  The Astra
returns 0 for anything closer than ~0.6 m, anything too far, and anything that
absorbed or scattered the projected pattern (dark, shiny or angled surfaces).
Those zeros are not "no obstacle" -- they are "no information", and an obstacle
check that treats them as free space will drive into things.

Each frame is saved with the body attitude at that instant.  Depth without
attitude cannot be turned into geometry on this robot: the body is never level,
and a frame analysed under a level assumption put 26% of its points below the
floor.  If /imu/data is absent the frame is still written, marked as having no
attitude, so it is obvious later that its geometry cannot be trusted.

    python3 grab_depth.py [output dir] [frame count]
"""
import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/depth'
WANT = int(sys.argv[2]) if len(sys.argv) > 2 else 3


def colourise(d, lo, hi):
    """Near = red, far = blue, invalid = black.  Matches the printed range."""
    valid = np.isfinite(d) & (d > 0)
    t = np.zeros_like(d, dtype=np.float64)
    if hi > lo:
        t[valid] = np.clip((d[valid] - lo) / (hi - lo), 0.0, 1.0)
    rgb = np.zeros(d.shape + (3,), dtype=np.uint8)
    # Simple red->green->blue ramp; enough to read shape and distance by eye.
    rgb[..., 0] = np.where(valid, np.clip(255 * (1.0 - 2.0 * t), 0, 255), 0)
    rgb[..., 1] = np.where(valid, np.clip(255 * (1.0 - np.abs(2.0 * t - 1.0)), 0, 255), 0)
    rgb[..., 2] = np.where(valid, np.clip(255 * (2.0 * t - 1.0), 0, 255), 0)
    return rgb


def write_png(path, rgb):
    """Minimal PNG writer so the Pi needs no image library installed."""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6))
           + chunk(b'IEND', b''))
    with open(path, 'wb') as f:
        f.write(png)


class Grabber(Node):
    def __init__(self):
        super().__init__('grab_depth')
        os.makedirs(OUT, exist_ok=True)
        self.n = 0
        self.imu = None
        self.create_subscription(Image, '/depth/image', self.cb, 5)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        print(f'waiting for {WANT} frames on /depth/image ...', flush=True)

    def imu_cb(self, msg):
        q = msg.orientation
        self.imu = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'quat_xyzw': [q.x, q.y, q.z, q.w],
            'pitch_deg': math.degrees(math.asin(
                max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))),
        }

    def cb(self, msg):
        if msg.encoding != '32FC1':
            print(f'unexpected encoding {msg.encoding}', flush=True)
            raise SystemExit(1)
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        d = np.where(np.isfinite(d), d, 0.0)
        valid = d > 0

        self.n += 1
        pct = 100.0 * valid.mean()
        if valid.any():
            v = d[valid]
            lo, hi = np.percentile(v, 1), np.percentile(v, 99)
            print(f'frame {self.n}: {msg.width}x{msg.height}  valid {pct:.1f}%  '
                  f'min {v.min():.2f}  p1 {lo:.2f}  median {np.median(v):.2f}  '
                  f'p99 {hi:.2f}  max {v.max():.2f} m', flush=True)
            # Row band across the middle: what a forward obstacle check would see.
            band = d[msg.height // 2 - 20:msg.height // 2 + 20, :]
            bv = band[band > 0]
            if bv.size:
                print(f'          centre band: valid {100.0*(band>0).mean():.1f}%  '
                      f'nearest {bv.min():.2f} m  median {np.median(bv):.2f} m',
                      flush=True)
        else:
            lo, hi = 0.0, 1.0
            print(f'frame {self.n}: entirely invalid -- nothing in range', flush=True)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        meta = {'stamp': stamp, 'width': msg.width, 'height': msg.height}
        if self.imu is None:
            meta['attitude'] = None
            print('          NO ATTITUDE -- /imu/data is not publishing; '
                  'this frame cannot be converted to geometry', flush=True)
        else:
            meta['attitude'] = self.imu
            meta['attitude_age_s'] = stamp - self.imu['stamp']
            print(f'          body pitch {self.imu["pitch_deg"]:+.2f} deg, '
                  f'attitude {1000*meta["attitude_age_s"]:+.0f} ms from frame',
                  flush=True)

        write_png(os.path.join(OUT, f'depth_{self.n:02d}.png'), colourise(d, lo, hi))
        np.save(os.path.join(OUT, f'depth_{self.n:02d}.npy'), d)
        with open(os.path.join(OUT, f'depth_{self.n:02d}.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        if self.n >= WANT:
            print(f'wrote {self.n} frames to {OUT} (near=red, far=blue, invalid=black)',
                  flush=True)
            raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(Grabber())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
