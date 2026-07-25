"""Dump the raw trace around the largest wheel-speed excursion."""
import sys

import numpy as np

from analyze_run import read_all, wheel_velocity


def main():
    path = sys.argv[1]
    offset = float(sys.argv[2])
    (imu_t, pitch), (cmd_t, pwm), (enc_t, enc_l, enc_r) = read_all(path)
    enc_v, enc_turn = wheel_velocity(enc_t, enc_l, enc_r)

    vel = np.interp(imu_t, enc_t, enc_v)
    turn = np.interp(imu_t, enc_t, enc_turn)
    upwm = np.interp(imu_t, cmd_t, pwm)
    err = pitch - offset

    rate = np.zeros(len(imu_t))
    for i in range(1, len(imu_t)):
        dt = imu_t[i] - imu_t[i - 1]
        rate[i] = (pitch[i] - pitch[i - 1]) / dt if 0.001 < dt < 0.05 else 0.0

    peak = int(np.argmax(np.abs(vel)))
    t0 = imu_t[peak]
    print(f'peak |vel|={vel[peak]:.0f} at t={t0 - imu_t[0]:.1f}s into the bag')

    win = (imu_t > t0 - 2.0) & (imu_t < t0 + 2.0)
    idx = np.where(win)[0]
    step = max(1, len(idx) // 60)
    print(f'{"t_rel":>7} {"pitch":>7} {"err":>7} {"rate":>8} {"pwm":>7} '
          f'{"vel":>7} {"turn":>7}')
    for i in idx[::step]:
        print(f'{imu_t[i]-t0:7.2f} {pitch[i]:7.2f} {err[i]:7.2f} {rate[i]:8.1f} '
              f'{upwm[i]:7.0f} {vel[i]:7.0f} {turn[i]:7.0f}')


if __name__ == '__main__':
    main()
