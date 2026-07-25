"""Measure loaded breakaway PWM per direction from closed-loop bags.

Only samples where the command has already been held near its current level are
used, so a still-rising PWM ramp cannot be mistaken for a low breakaway point.
"""
import sys

import numpy as np

from analyze_run import read_all, wheel_velocity


def main():
    paths = sys.argv[1:]
    all_pwm, all_vel = [], []
    for path in paths:
        (imu_t, _), (cmd_t, pwm), (enc_t, enc_l, enc_r) = read_all(path)
        enc_v, _ = wheel_velocity(enc_t, enc_l, enc_r)
        # Work on the encoder clock; that is where wheel motion is observed.
        p = np.interp(enc_t, cmd_t, pwm)
        all_pwm.append(p)
        all_vel.append(enc_v)
    p = np.concatenate(all_pwm)
    v = np.concatenate(all_vel)

    # Hold filter: command must be within 15 PWM of its value 100 ms earlier
    # (10 encoder samples), so transient ramps are excluded.
    lag = 10
    held = np.zeros(len(p), dtype=bool)
    held[lag:] = np.abs(p[lag:] - p[:-lag]) < 15.0

    print(f'{"pwm bin":>14} {"n_held":>8} {"moving>25":>10} {"|vel|med":>9}')
    for sign, name in ((1, 'FWD +'), (-1, 'BWD -')):
        print(f'--- {name} ---')
        for lo, hi in ((5, 15), (15, 25), (25, 35), (35, 45), (45, 55),
                       (55, 65), (65, 80), (80, 100), (100, 140)):
            sp = p * sign
            m = held & (sp >= lo) & (sp < hi)
            if m.sum() < 30:
                continue
            print(f'{lo:6d}-{hi:<7d} {m.sum():8d} '
                  f'{100.0*(np.abs(v[m]) > 25).mean():9.1f}% '
                  f'{np.median(np.abs(v[m])):9.0f}')


if __name__ == '__main__':
    main()
