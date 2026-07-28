import sys, time, math
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
REQ = 0.12

rclpy.init()
n = rclpy.create_node('test_guard')
pub = n.create_publisher(Twist, '/cmd_vel_raw', 1)
st = {'guard': '?', 'out': None, 'pitch': float('nan')}

def out_cb(m):
    st['out'] = m.linear.x
def g_cb(m):
    st['guard'] = m.data
def imu_cb(m):
    q = m.orientation
    st['pitch'] = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y - q.z*q.x)))))

n.create_subscription(Twist, '/cmd_vel', out_cb, 1)
n.create_subscription(String, '/obstacle/guard_state', g_cb, 5)
n.create_subscription(Imu, '/imu/data', imu_cb, 10)

msg = Twist(); msg.linear.x = REQ
end = time.time() + DUR
last = 0.0
print('requesting %.2f m/s forward on /cmd_vel_raw; watching /cmd_vel' % REQ, flush=True)
while rclpy.ok() and time.time() < end:
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.02)
    now = time.time()
    if now - last >= 1.0:
        last = now
        o = st['out']
        allowed = ('%.3f' % o) if o is not None else '  -  '
        pct = ('%3.0f%%' % (100*o/REQ)) if o is not None else '  - '
        print('%s  pitch %+6.1f  guard %-14s  allowed %s m/s (%s of request)'
              % (time.strftime('%H:%M:%S'), st['pitch'], st['guard'], allowed, pct), flush=True)
