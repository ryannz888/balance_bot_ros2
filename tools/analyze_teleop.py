"""Score a teleop run straight from /balance/state.

analyze_run.py has to replay the controller's state machine offline to work out
which stretches were engaged.  /balance/state records that decision, plus the
setpoints and the trim, so this script reads the controller's own view instead
of reconstructing it.

    python analyze_teleop.py <bag path>
"""
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from pathlib import Path

# Fallback only.  The controller writes the real field list into the message
# layout label, so a bag stays readable even when the field set changes.
FIELDS = ('pitch', 'err', 'rate', 'trim', 'offset', 'vel', 'vel_raw', 'turn',
          'v_ref', 'turn_ref', 'vel_int', 'u', 'sync', 'engaged')
IDX = {name: i for i, name in enumerate(FIELDS)}
TICKS_PER_M = 277.5 / (2 * np.pi * 0.0325)


def load(path):
    """Returns (times, rows, index map read from the bag itself)."""
    rows, times, idx = [], [], None
    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/balance/state']
        if not conns:
            sys.exit('bag has no /balance/state; record it with the current launch file')
        for conn, timestamp, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if idx is None and msg.layout.dim and ',' in msg.layout.dim[0].label:
                idx = {n: i for i, n in enumerate(msg.layout.dim[0].label.split(','))}
            rows.append(list(msg.data))
            times.append(timestamp * 1e-9)
    return np.array(times), np.array(rows), (idx or dict(IDX))


def segments(t, engaged):
    """Contiguous engaged stretches as (start_index, end_index) pairs."""
    out, start = [], None
    for i, e in enumerate(engaged):
        if e > 0.5 and start is None:
            start = i
        elif e <= 0.5 and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(engaged)))
    return out


def command_blocks(t, v_ref, turn_ref, deadzone=5.0):
    """Stretches where either setpoint is non-zero, i.e. a drive command."""
    active = (np.abs(v_ref) > deadzone) | (np.abs(turn_ref) > deadzone)
    out, start = [], None
    for i, a in enumerate(active):
        if a and start is None:
            start = i
        elif not a and start is not None:
            if t[i] - t[start] > 0.5:
                out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(active)))
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    t, d, idx = load(sys.argv[1])
    t = t - t[0]
    col = {name: d[:, i] for name, i in idx.items()}

    print(f'bag: {sys.argv[1]}')
    print(f'duration {t[-1]:.1f}s, {len(t)} samples, {len(t)/t[-1]:.1f} Hz\n')

    # A physical fall is a gap between real balancing stretches.  Do NOT count
    # log lines or raw engage events: with latch_on_fall false the robot
    # re-engages every time it crosses the engage window while being picked up,
    # so one fall produces several engage/trip cycles.  Counting those reported
    # 17 "falls" for a run the operator saw fall once.
    segs = [(s, e) for s, e in segments(t, col['engaged']) if t[e - 1] - t[s] >= 5.0]
    print(f'== engaged segments ==   real interruptions between them: {max(0, len(segs) - 1)}')
    total = 0.0
    for s, e in segments(t, col['engaged']):
        dur = t[e - 1] - t[s]
        total += dur
        if dur < 1.0:
            continue
        err = col['err'][s:e]
        print(f'  {t[s]:7.1f}s +{dur:6.1f}s  errRMS {np.sqrt((err**2).mean()):.2f}deg  '
              f'err[{err.min():+.2f},{err.max():+.2f}]  '
              f'|u|max {np.abs(col["u"][s:e]).max():5.1f}')
    print(f'  engaged {total:.1f}s of {t[-1]:.1f}s\n')

    print('== drive commands: setpoint tracking ==')
    print('  each block reports the mean over its last half, once the ramp has settled')
    for s, e in command_blocks(t, col['v_ref'], col['turn_ref']):
        mid = (s + e) // 2
        v_ref = col['v_ref'][mid:e].mean()
        vel = col['vel'][mid:e].mean()
        turn_ref = col['turn_ref'][mid:e].mean()
        turn = col['turn'][mid:e].mean()
        kind = 'drive' if abs(v_ref) > abs(turn_ref) else 'yaw  '
        print(f'  {t[s]:7.1f}s +{t[e-1]-t[s]:4.1f}s {kind} '
              f'v_ref {v_ref:+7.1f} -> vel {vel:+7.1f} ticks/s '
              f'({v_ref/TICKS_PER_M:+.3f} -> {vel/TICKS_PER_M:+.3f} m/s)  '
              f'turn_ref {turn_ref:+7.1f} -> turn {turn:+7.1f}  '
              f'trim {col["trim"][mid:e].mean():+.2f}deg')

    print('\n== station keeping (no command, engaged) ==')
    idle = (col['engaged'] > 0.5) & (np.abs(col['v_ref']) < 5) & (np.abs(col['turn_ref']) < 5)
    if idle.sum() > 10:
        err = col['err'][idle]
        print(f'  {idle.sum()/len(t)*t[-1]:.1f}s  errRMS {np.sqrt((err**2).mean()):.2f}deg  '
              f'velMean {col["vel"][idle].mean():+.1f}  '
              f'turnMean {col["turn"][idle].mean():+.1f}  '
              f'velInt mean {col["vel_int"][idle].mean():+.0f}')

    # Net displacement per command block: did releasing the stick pull it back?
    if 'bias' in col and np.abs(col['bias']).max() > 1e-6:
        print('\n== offset self-adaptation ==')
        print(f'  bias {col["bias"][0]:+.3f} -> {col["bias"][-1]:+.3f} deg  '
              f'(configured offset {col["offset"][-1] - col["bias"][-1]:.2f}, '
              f'effective {col["offset"][-1]:.2f})')

    print('\n== position drift (integrated wheel speed, wheel revs) ==')
    eng = col['engaged'] > 0.5
    dt = np.diff(t, prepend=t[0])
    pos = np.cumsum(np.where(eng, col['vel'] * dt, 0.0)) / 277.5
    print(f'  net {pos[-1]:+.2f} rev ({pos[-1]*2*np.pi*0.0325:+.2f} m), '
          f'range [{pos.min():+.2f}, {pos.max():+.2f}]')


if __name__ == '__main__':
    main()
