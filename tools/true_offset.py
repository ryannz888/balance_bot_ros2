"""Recover the true balance point from the outer loop's steady-state trim.

While balancing, target = pitch_offset_deg + trim.  The mean trim over a long
engaged segment is therefore the standing error in pitch_offset_deg.
"""
import sys

import numpy as np

from analyze_run import read_all, wheel_velocity


def main():
    path = sys.argv[1]
    offset = float(sys.argv[2])
    kp_v = float(sys.argv[3])
    ki_v = float(sys.argv[4])
    alpha = float(sys.argv[5]) if len(sys.argv) > 5 else 0.06
    max_trim = 4.0

    (imu_t, pitch), _, (enc_t, enc_l, enc_r) = read_all(path)
    enc_v, _ = wheel_velocity(enc_t, enc_l, enc_r)

    filt = np.zeros(len(enc_t))
    integ = np.zeros(len(enc_t))
    for i in range(1, len(enc_t)):
        filt[i] = alpha * enc_v[i] + (1.0 - alpha) * filt[i - 1]
        dt = enc_t[i] - enc_t[i - 1]
        integ[i] = integ[i - 1] + (filt[i] * dt if 0.0 < dt < 0.05 else 0.0)
        integ[i] = max(-500.0, min(500.0, integ[i]))

    trim = np.clip(kp_v * filt + ki_v * integ, -max_trim, max_trim)

    # Only the long balancing stretches carry a meaningful steady state.
    err = pitch - offset
    ok = np.interp(enc_t, imu_t, np.abs(err)) < 3.0
    # Drop the first seconds so the integral has time to wind up.
    ok[:int(len(ok) * 0.15)] = False

    print(f'samples used: {ok.sum()} / {len(ok)}')
    print(f'mean trim   = {trim[ok].mean():+.3f} deg  (median {np.median(trim[ok]):+.3f})')
    print(f'mean pitch  = {np.interp(enc_t, imu_t, pitch)[ok].mean():.3f} deg')
    print(f'=> true balance point = {offset} + ({trim[ok].mean():+.3f}) '
          f'= {offset + trim[ok].mean():.2f} deg')
    print(f'integral: mean={integ[ok].mean():+.0f}  '
          f'clamped {100.0 * (np.abs(integ[ok]) >= 499).mean():.1f}%')


if __name__ == '__main__':
    main()
