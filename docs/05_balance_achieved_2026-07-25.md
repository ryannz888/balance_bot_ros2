# Balance Achieved - 2026-07-25

## Result

The robot balances on its own indefinitely. The endurance run `tuneA10` held a single engaged
segment for **228.7 s** and was still balancing when the recording was stopped; it was engaged for
432.5 s of a 457.5 s recording. Pitch error stays inside about +/- 1 deg and the machine holds
position to within a quarter of a wheel revolution.

Best single-segment figures, from `tuneA9` and the first segment of `tuneA10`:

```text
pitch error RMS       0.30 deg
pitch error range     [-1.03, +1.21] deg
position wander       0.22 rev (std),  0.67 rev peak-to-peak
wheel speed RMS       61 ticks/s,  peak 267
yaw bias              0
peak PWM commanded    under 70 of 255
```

This is the first day where the body genuinely holds itself upright rather than being caught by
hand or by a wall.

## Root Cause of Every Earlier Failure

Before this run, all 29 engaged segments of `tuneA1` exited on the wheel-speed safety latch and
none on the tilt limit. The body was already stable inside +/- 2.5 deg for up to 7.8 s while the
whole robot accelerated across the floor at a steady 430 ticks/s. The angle loop was never the
problem; there was no working velocity feedback.

Two sign errors were responsible:

- `kp_v` had been used as a positive gain. It must be **negative**. To decelerate a forward-
  rolling balancing robot you must first drive the wheels forward harder so the base moves ahead
  of the centre of mass and the body pitches back; the angle loop then decelerates on its own.
  Derivation: `v > 0` needs a more positive `u`, so a more positive `err`, so
  `target = offset + trim` must shrink, so `trim < 0`, so `kp_v < 0`.
- `kd_wheel` applies `-kd_wheel * v` directly to the output, which is a naive brake. It makes the
  body pitch further forward and the angle loop immediately fights it. Leave it at 0.

`velocity_filter_alpha` also had to drop from 0.3 to 0.06. At the 100 Hz encoder rate, 0.3 puts
the outer loop cutoff near 4.8 Hz, which is faster than the inner angle loop. The outer loop must
be much slower than the inner loop.

## Best Parameter Set (run `tuneA6`)

```text
pitch_offset_deg = 7.53          capture_offset_on_engage = false
kp = 70,  ki = 0,  kd = 5
kp_v = -0.008,  ki_v = -0.005
kd_wheel = 0,  max_wheel_damping_pwm = 0
velocity_filter_alpha = 0.06     max_offset_trim_deg = 4.0
wheel_sync_kp = 0.4,  max_wheel_sync_pwm = 60
turn_velocity_filter_alpha = 0.15
pitch_rate_filter_alpha = 0.3 (default)
engage_below_deg = 3,  engage_rate_below_dps = 15
max_tilt_deg = 12,  max_wheel_velocity = 600
latch_on_fall = false            recovery_pwm = 0
deadband_pwm_fwd = 0,  deadband_pwm_bwd = 0,  deadband_ramp_deg = 0
max_pwm = 255,  motor_output_sign = 1.0
```

`latch_on_fall = false` lets the controller re-engage after the body is set upright again, so a
single recording can hold many attempts without a controller restart.

## Tuning Progression

| run | change | errRMS | velRMS | posSD (rev) | surge (s) | turnAvg | longest |
|-----|--------|--------|--------|-------------|-----------|---------|---------|
| A1 | `kd_wheel = 0.3`, no velocity loop | - | - | - | - | - | 7.8 s |
| A2 | `kp_v = -0.005` sign fixed | - | - | - | - | +78 | 367 s |
| A3 | `wheel_sync_kp = 0.25` | - | - | - | - | +1 | 210 s |
| A4 | `ki_v = -0.0015`, sync 0.4 | 0.56 | 102 | 0.84 | 8.8 | +1 | 115 s |
| A5 | `kp_v -0.006`, `ki_v -0.003` | 0.51 | 82 | 0.37 | 5.2 | +1 | 134 s |
| A6 | offset 7.53, `kp_v -0.008`, `ki_v -0.005` | 0.37 | 62 | 0.25 | 5.8 | -0 | 99 s* |
| A7 | static-friction feed-forward | 1.10 | 133 | 0.44 | 4.3 | +93 | 56 s |
| A8 | A6 parameters restored | 1.03 | 135 | 0.49 | 6.4 | +86 | 9.6 s |
| -- | **loose fastener found and tightened** | | | | | | |
| A9 | A6 parameters, unchanged | **0.31** | **59** | **0.23** | 5.4 | **+1** | **70 s**† |
| A10 | offset 7.60 (reverted, see below) | 0.30 | 61 | 0.22 | 4.2 | 0 | **229 s**‡ |

\* `tuneA6` ended because recording was stopped, not because the robot fell. 153.4 s of the
155 s recording was engaged, in only two segments.

† `tuneA9` was engaged for the entire 70.3 s recording in a single segment: it never fell and
never disengaged. Pitch error stayed inside [-0.95, +1.00] deg, peak wheel speed was 239 ticks/s
against 606 in `tuneA6`, and no sample in the whole run commanded more than 70 PWM.

‡ `tuneA10` is the endurance run: 228.7 s in one segment, still balancing when recording stopped,
432.5 s engaged out of 457.5 s. Its figures in the table are from the first 88 s segment. Two
short segments in the middle of that run show `turnAvg` 184 and errRMS 1.37; those are external
disturbances, not controller behaviour.

## What Was Never the Problem

Recorded so these are not re-tested:

- `pitch_offset_deg` beyond the 7.53 value. It was swept by trial and error across earlier
  sessions at great cost; the mean-pitch method settles it in one run.
- `motor_output_sign`. Confirmed 1.0.
- Motor or gearbox asymmetry. An open-loop free-spin test measured 1590-1600 ticks/s in both
  directions.
- The angle inner loop. `kp = 70`, `kd = 5` were inherited and never needed changing; every
  failure through `tuneA1` was a wheel-speed runaway, not an angle-loop failure.
- `kd_wheel` and `max_wheel_damping_pwm`. Structurally the wrong tool, see above. Leave at 0.
- Deadband feed-forward. Leave at 0.

## Next Stage

The angle loop, the velocity loop and the position loop are all closed and stable, so balance
tuning is finished. The next piece of work is accepting motion commands, which means feeding a
non-zero velocity target into the outer loop instead of the implicit zero, and adding a yaw-rate
target alongside `wheel_sync_kp` so the robot can be steered. Neither is a tuning task.

## How `pitch_offset_deg` Was Refined to 7.53

Do not sweep this parameter by trial and error. While the robot balances with a mean wheel speed
of zero, the mean pitch over a long engaged segment *is* the true balance point. Two independent
runs agreed to within 0.03 deg:

```text
tuneA4 mean pitch = 7.528 deg
tuneA5 mean pitch = 7.554 deg
```

Estimating instead from the outer-loop trim gave inconsistent answers (7.46 vs 8.21 deg) because
the velocity integral was clamped; the mean-pitch method is the reliable one.

**Stop refining at this resolution.** `tuneA10` tried 7.60 and it was slightly worse: the integral
mean rose from +38 to +184 and started clamping again (0.0 to 2.1 percent). Mean pitch between the
two runs differed by 0.27 deg, which is larger than the 0.07 deg change being tested, so anything
below about 0.2 deg is run-to-run noise. Use the integral mean, not mean pitch, to judge offset at
this scale: it should sit near zero and never clamp. 7.53 is the value to keep.

The correction was confirmed by its effect on the integral, which had been silently spending its
range compensating the offset error:

```text
velocity_integral mean:    -322 (A5, offset 7.20)  ->   -49 (A6, offset 7.53)
velocity_integral clamped:  8.5% (A5)              ->  0.0% (A6)
```

## A Loose Fastener Masqueraded as a Tuning Problem

Runs `tuneA7` and `tuneA8` both degraded badly, and the cause was mechanical, not a parameter.

`tuneA7` enabled static-friction feed-forward and every metric got worse, with yaw regressing
hardest: `turnAvg` went from about 0 to +93, `|turn|` peaks to 1088. The obvious reading was that
the feed-forward caused it. **That reading was wrong.** `tuneA8` reverted every deadband
parameter to zero and the yaw bias stayed (`turnAvg` +86, `|turn|max` 1207), while engaged time
collapsed to 11.3 s out of 84.4 s.

Three independent signals pointed at hardware rather than gains:

```text
mean pitch while balancing:  7.528 (A4)  7.554 (A5)  7.673 (A6)  8.176 (A8)
turnAvg:                     -0 (A6)     +93 (A7)    +86 (A8, deadband already reverted)
engaged fraction:            153/155 s (A6)          11.3/84.4 s (A8)
```

A single loose fastener explains all three at once: a shifted balance point, a left/right
asymmetry, and total loss of performance. This project has a documented history of the IMU
mounting screw working loose. Tightening the hardware restored everything.

`tuneA9` re-ran the `tuneA6` parameters unchanged after the fix and beat `tuneA6` on every
metric, which means the hardware had already been degrading during `tuneA6` itself.

**Rule going forward:** when several unrelated metrics degrade at once and a parameter revert does
not restore them, stop tuning and inspect the hardware. Do not attribute it to the most recent
parameter change.

## Static-Friction Feed-Forward: Verdict Deferred

The `tuneA7` feed-forward test is inconclusive because it overlapped the loose fastener. What the
data does show is that it worked mechanically: the wheels-moving fraction at `|pwm|` 100-150 rose
from 92.9 to 97.0 percent, and samples in the sticky 1-40 band fell from 11033 to 2924.

It is probably not needed at all. After the hardware fix, `tuneA9` never commanded more than
70 PWM for the entire recording and peak wheel speed dropped from 606 to 239 ticks/s, so the
stick-slip lurch largely disappeared on its own.

If it is ever revisited, note the structural defect: the assist is added to the common-mode `u`,
while the bridge maps `left = u - sync` and `right = u + sync`. Since the two wheels have
different breakaway thresholds, a common-mode assist can free one wheel and not the other, which
is a yaw disturbance that `wheel_sync_kp` cannot correct because its own differential command is
subject to the same stiction. Compensation must be applied **per wheel after the differential
split**. The `deadband_dither_hz` path has the same defect as written.

## Breakaway Measurement Note

Loaded breakaway measured from closed-loop bags is roughly symmetric and has no sharp threshold:
about 60 percent of held commands move the wheels at 25-35 PWM, only about 50 percent at 45-55,
and 80-85 percent at 65-80.

An earlier reading of this data claimed stiction was not a limiting factor. That was wrong: the
test counted samples where PWM was still ramping, which made breakaway look far lower than it is.
Any breakaway test must exclude samples where the command is still changing.

## Confirmed Facts

- `pitch_offset_deg = 7.20` is correct. Error ranges in `tuneA2` are symmetric about zero and
  falls occur in both directions, so it needs no further sweeping.
- `motor_output_sign = 1.0` is correct. Do not flip it.
- Static friction is not a limiting factor. At commanded `|pwm|` of 1-40 the wheels were already
  turning 72 percent of the time, so the deadband feed-forward can stay at 0.
- IMU now publishes at about **147 Hz**, not the 303 Hz recorded on 2026-07-24. Encoders are at
  about 99 Hz. A one-pole filter alpha therefore maps to roughly half the cutoff frequency it did
  before, so `pitch_rate_filter_alpha` and `velocity_filter_alpha` are not comparable across the
  two dates.
- Arduino firmware now has a command watchdog (`CMD_TIMEOUT_MS`), so an undelivered stop command
  no longer leaves the wheels latched on.

## Open Issue

The robot balances but rotates in place. Yaw is unregulated: `turn_velocity = left_v - right_v`
sits at a persistent bias near +100 ticks/s with peaks to 730, meaning the left wheel consistently
outruns the right. `wheel_sync_kp` exists for this and was still 0 during `tuneA2`.

Sign check against `serial_bridge`: `left_pwm = (linear - angular) * 100` and
`right_pwm = (linear + angular) * 100`, with `angular * 100 = sync_correction`. A positive
`turn_velocity` therefore needs a positive `sync_correction`, so **`wheel_sync_kp` is positive**.
The correction is equal and opposite on the two sides, so it does not change the mean forward
torque and does not disturb the balance loop.
