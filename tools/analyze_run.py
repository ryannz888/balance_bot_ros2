"""Reconstruct engaged segments and report balance/wheel behaviour for one bag."""
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def pitch_from_quat(x, y, z, w):
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(sinp))


def read_all(path):
    imu_t, imu_pitch = [], []
    cmd_t, cmd_pwm = [], []
    enc_t, enc_l, enc_r = [], [], []
    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ('/imu/data', '/cmd_vel', '/wheel/encoders')]
        for connection, bag_ts, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == '/imu/data':
                q = msg.orientation
                imu_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                imu_pitch.append(pitch_from_quat(q.x, q.y, q.z, q.w))
            elif connection.topic == '/cmd_vel':
                cmd_t.append(bag_ts * 1e-9)
                cmd_pwm.append(msg.linear.x * 100.0)
            else:
                l_str, r_str = msg.data.split(',')
                enc_t.append(bag_ts * 1e-9)
                enc_l.append(int(l_str))
                enc_r.append(int(r_str))
    return (np.array(imu_t), np.array(imu_pitch)), \
           (np.array(cmd_t), np.array(cmd_pwm)), \
           (np.array(enc_t), np.array(enc_l), np.array(enc_r))


def wheel_velocity(enc_t, enc_l, enc_r):
    """Match the controller: left_v = dl/dt, right_v = -dr/dt.

    Returns (mean, turn) where turn = left_v - right_v drives yaw.
    """
    v = np.zeros(len(enc_t))
    turn = np.zeros(len(enc_t))
    for i in range(1, len(enc_t)):
        dt = enc_t[i] - enc_t[i - 1]
        if 0.005 < dt < 0.2:
            left = (enc_l[i] - enc_l[i - 1]) / dt
            right = -(enc_r[i] - enc_r[i - 1]) / dt
            v[i] = (left + right) / 2.0
            turn[i] = left - right
        else:
            v[i] = v[i - 1]
            turn[i] = turn[i - 1]
    return v, turn


def main():
    path = sys.argv[1]
    offset = float(sys.argv[2]) if len(sys.argv) > 2 else 7.20
    engage_below = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    max_tilt = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
    engage_rate_below = 15.0
    max_wheel_v = float(sys.argv[5]) if len(sys.argv) > 5 else 600.0
    min_report_s = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

    (imu_t, pitch), (cmd_t, pwm), (enc_t, enc_l, enc_r) = read_all(path)
    dur = imu_t[-1] - imu_t[0]
    print(f'bag: {dur:.1f}s  imu={len(imu_t)} ({len(imu_t)/dur:.0f}Hz)  '
          f'cmd={len(cmd_t)}  enc={len(enc_t)} ({len(enc_t)/dur:.0f}Hz)')

    enc_v, enc_turn = wheel_velocity(enc_t, enc_l, enc_r)
    # Resample wheel speed and PWM onto the IMU clock.
    vel = np.interp(imu_t, enc_t, enc_v) if len(enc_t) > 1 else np.zeros(len(imu_t))
    turn = np.interp(imu_t, enc_t, enc_turn) if len(enc_t) > 1 else np.zeros(len(imu_t))
    upwm = np.interp(imu_t, cmd_t, pwm) if len(cmd_t) > 1 else np.zeros(len(imu_t))

    # pitch rate as the controller derives it (raw sample difference)
    rate = np.zeros(len(imu_t))
    for i in range(1, len(imu_t)):
        dt = imu_t[i] - imu_t[i - 1]
        rate[i] = (pitch[i] - pitch[i - 1]) / dt if 0.001 < dt < 0.05 else 0.0

    err = pitch - offset
    segments = []
    engaged = False
    start = None
    reason = None
    for i in range(len(err)):
        if engaged:
            if abs(vel[i]) > max_wheel_v:
                segments.append((start, i, 'WHEEL'))
                engaged = False
            elif abs(err[i]) > max_tilt:
                segments.append((start, i, 'TILT'))
                engaged = False
        elif abs(err[i]) < engage_below and abs(rate[i]) < engage_rate_below:
            engaged = True
            start = i
    if engaged:
        segments.append((start, len(err) - 1, 'END'))

    print(f'\n{len(segments)} engaged segments (offset={offset}, '
          f'max_tilt={max_tilt}, max_wheel_v={max_wheel_v})')
    hdr = (f'{"#":>3} {"dur":>6} {"exit":>6} {"errRange":>14} {"fall":>5} '
           f'{"|pwm|avg":>9} {"sat%":>5} {"velMean":>8} {"velMax":>7} {"xings":>6} '
           f'{"turnAvg":>8} {"|turn|max":>10}')
    print(hdr)
    for si, (i0, i1, why) in enumerate(segments):
        d = imu_t[i1] - imu_t[i0]
        if d < min_report_s:
            continue
        e = err[i0:i1 + 1]
        p = upwm[i0:i1 + 1]
        v = vel[i0:i1 + 1]
        tw = turn[i0:i1 + 1]
        xings = int(np.sum(np.diff(np.sign(e)) != 0))
        sat = 100.0 * np.mean(np.abs(p) >= 254.0)
        fall = 'FWD' if err[i1] > 0 else 'BWD'
        print(f'{si:3d} {d:6.2f} {why:>6} [{e.min():6.2f},{e.max():6.2f}] {fall:>5} '
              f'{np.mean(np.abs(p)):9.1f} {sat:5.1f} {np.mean(v):8.0f} '
              f'{np.max(np.abs(v)):7.0f} {xings:6d} {np.mean(tw):8.0f} '
              f'{np.max(np.abs(tw)):10.0f}')

    if segments:
        durs = [imu_t[b] - imu_t[a] for a, b, _ in segments]
        print(f'\ntotal engaged {sum(durs):.1f}s  best {max(durs):.2f}s  '
              f'median {np.median(durs):.2f}s')

    # Surge diagnosis: integral of wheel velocity is position, so the slow
    # back-and-forth is the position loop's own oscillation.
    print(f'\n{"#":>3} {"dur":>6} {"posPP_rev":>10} {"surgeHz":>8} {"surge_s":>8} '
          f'{"velRMS":>7} {"errRMS":>7} {"posSD":>8} {"vSmMax":>8}')
    for si, (i0, i1, _) in enumerate(segments):
        d = imu_t[i1] - imu_t[i0]
        if d < max(min_report_s, 5.0):
            continue
        t = imu_t[i0:i1 + 1]
        v = vel[i0:i1 + 1]
        pos = np.concatenate(([0.0], np.cumsum(v[1:] * np.diff(t)))) / 277.5
        # Raw wheel speed is too noisy to count surges; smooth to ~0.4 s first.
        win = max(3, int(0.4 * len(t) / d))
        vs = np.convolve(v, np.ones(win) / win, mode='same')
        vx = int(np.sum(np.diff(np.sign(vs[win:-win])) != 0))
        hz = vx / 2.0 / d
        print(f'{si:3d} {d:6.2f} {pos.max()-pos.min():10.2f} {hz:8.3f} '
              f'{(1.0/hz if hz > 0 else 0):8.2f} {np.sqrt(np.mean(v**2)):7.0f} '
              f'{np.sqrt(np.mean(err[i0:i1+1]**2)):7.2f} {np.std(pos):8.2f} '
              f'{np.max(np.abs(vs)):8.0f}')

    # Static-friction evidence: commanded PWM present but wheels not turning.
    moving = np.abs(vel) > 15.0
    for lo, hi in ((1, 40), (40, 70), (70, 100), (100, 150), (150, 300)):
        m = (np.abs(upwm) >= lo) & (np.abs(upwm) < hi)
        if m.sum() > 20:
            print(f'|pwm| {lo:3d}-{hi:3d}: n={m.sum():5d}  wheels moving '
                  f'{100.0*moving[m].mean():5.1f}%  |vel|med={np.median(np.abs(vel[m])):6.0f}')


if __name__ == '__main__':
    main()
