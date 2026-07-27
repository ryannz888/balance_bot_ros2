# Balance Tuning Pause - 2026-07-24

## Safe Pause State

- The remote `balance_controller` was stopped deliberately.
- A zero `/cmd_vel` command was sent after stopping it.
- `/hfi_imu` and `/serial_bridge` remain available on the Raspberry Pi.
- Do not resume a controller test until the robot is hand-supported near upright.

## Facts Confirmed During This Session

- IMU rate is about 303 Hz and encoder rate is about 100 Hz after the Pi reboot.
- Both motor channels and both encoder channels were verified earlier with synchronized PWM tests.
- Positive PWM moves the wheels forward. A single-direction IMU check confirmed that leaning
  the body toward its physical front raises the controller pitch value.
- Therefore `motor_output_sign:=1.0` is the correct polarity. Do not flip it as a tuning step.
- The apparent recoveries in some earlier runs were caused by external contact or hand input,
  not autonomous recovery. Treat those runs as failed balance attempts.

## Reliable Fixed Pitch Reference

The most recent deliberate upright calibration is the reference to use on resume:

```text
pitch_offset_deg = 7.20
standard deviation = 0.11 deg
samples = 2374 over 8 s
```

The earlier `12.10 deg` sample was explicitly marked inaccurate and must be discarded.
Automatic capture produced inconsistent values between roughly 5 and 13 deg, so resume with
`capture_offset_on_engage:=false` and the fixed 7.20 deg offset until a better calibration
method is implemented.

## Controller Changes Already Made

`balance_bot/balance_controller.py` now includes:

- Independent positive and negative PWM limits: `max_pwm_fwd`, `max_pwm_bwd`.
- A configurable wheel-runaway safety latch: `max_wheel_velocity`.
- Stable-pose capture parameters: `capture_settle_time_sec`, `capture_rate_below_dps`.
- Smooth static-friction and recovery parameters already exposed by the controller.

The wheel-speed latch is useful while hand-tuning because a supported body can otherwise let
the wheels run into a wall without exceeding the pitch limit.

## Latest Test and Result

The last protected test used the reliable fixed offset but was not stable:

```text
capture_offset_on_engage=false
pitch_offset_deg=7.20
kp=70, kd=5
max_pwm_fwd=255, max_pwm_bwd=255
deadband_ramp_deg=0.5
recovery_pwm=110, recovery_err_deg=0.5, recovery_ramp_deg=1.5
max_tilt_deg=5, max_wheel_velocity=250
```

It reached a filtered wheel speed of about +271 ticks/s and the wheel-speed protection cut
output. This parameter set must not be described as balanced or reused unchanged.

## Resume Plan

1. Start from the fixed 7.20 deg reference, with auto-capture disabled and the wheel-speed
   safety latch enabled.
2. Keep the smooth static-friction ramp; avoid the earlier abrupt recovery boost.
3. Tune a bounded velocity/position braking term so wheel motion is opposed before it reaches
   the safety limit, then retune pitch P/D around that behavior.
4. Keep each run hand-supported and use the safety latch as the stopping condition; do not use
   a wall or a manual push as evidence of autonomous recovery.
