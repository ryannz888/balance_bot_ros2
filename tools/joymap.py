"""Report which /joy button and axis indices a gamepad actually moves.

Identifying a pad one press at a time by watching `ros2 topic echo` over ssh is
slow and mistimes constantly -- the press lands outside the echo window and you
learn nothing.  This subscribes once, watches for a while, and prints every
index the moment it first goes active, so one session maps the whole pad.

It also prints the name SDL's GameController layout assigns to that index.  A
mismatch there is worth knowing about: it means the pad is not being remapped to
the standard layout, and config written against SDL indices will not apply.

    ros2 run joy game_controller_node &
    python3 joymap.py [seconds]

Verified on 2026-07-28 against a 2.4G dongle pad that enumerates as an Xbox 360
controller: LB reported index 9 and RB index 10, matching SDL, while the left
stick's forward direction read +1.0 rather than SDL's raw -1.0, because
game_controller_node already flips it.  That sign is exactly what
config/teleop_joy.yaml had wrong.
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
SDL_BUTTONS = ['A', 'B', 'X', 'Y', 'BACK', 'GUIDE', 'START', 'LSTICK', 'RSTICK',
               'LB', 'RB', 'DPAD_UP', 'DPAD_DOWN', 'DPAD_LEFT', 'DPAD_RIGHT']
SDL_AXES = ['LEFT_X', 'LEFT_Y', 'RIGHT_X', 'RIGHT_Y', 'LT', 'RT']


class Mapper(Node):
    def __init__(self):
        super().__init__('joymap')
        self.seen_btn = {}
        self.seen_axis = {}
        self.t0 = self.now()
        self.create_subscription(Joy, '/joy', self.cb, 10)
        print('press things now...', flush=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb(self, msg):
        for i, v in enumerate(msg.buttons):
            if v and i not in self.seen_btn:
                name = SDL_BUTTONS[i] if i < len(SDL_BUTTONS) else '?'
                self.seen_btn[i] = name
                print(f'  BUTTON {i:2d}  (SDL says {name})', flush=True)
        for i, v in enumerate(msg.axes):
            if abs(v) > 0.5 and i not in self.seen_axis:
                name = SDL_AXES[i] if i < len(SDL_AXES) else '?'
                self.seen_axis[i] = (name, v)
                print(f'  AXIS   {i:2d}  (SDL says {name}) reached {v:+.2f}',
                      flush=True)
        if self.now() - self.t0 > SECONDS:
            print('done.', flush=True)
            raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(Mapper())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
