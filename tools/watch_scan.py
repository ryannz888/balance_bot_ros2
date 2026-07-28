import re, sys, time
import rclpy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import math
rclpy.init()
n = rclpy.create_node('watch_scan')
state = {'pitch': float('nan')}
def imu_cb(m):
    q = m.orientation
    state['pitch'] = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y - q.z*q.x)))))
def cb(m):
    r = list(m.ranges)
    nan = sum(1 for v in r if v != v)
    inf = sum(1 for v in r if v == float('inf'))
    fin = [v for v in r if v == v and v != float('inf')]
    now = time.strftime('%H:%M:%S')
    if fin:
        print('%s pitch %+6.1f  OBSTACLE nearest %.2f m  (%d bins hit, %d clear, %d unknown)'
              % (now, state['pitch'], min(fin), len(fin), inf, nan), flush=True)
    elif inf:
        print('%s pitch %+6.1f  clear      (%d bins clear, %d unknown)'
              % (now, state['pitch'], inf, nan), flush=True)
    else:
        print('%s pitch %+6.1f  UNKNOWN    (all %d bins unknown)'
              % (now, state['pitch'], nan), flush=True)
n.create_subscription(Imu, '/imu/data', imu_cb, 10)
n.create_subscription(LaserScan, '/obstacle/scan', cb, 5)
end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 30)
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
