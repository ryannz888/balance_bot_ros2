# balance_bot_ros2

Two-wheeled self-balancing robot. Raspberry Pi 5 + Arduino Mega + IMU + depth camera, ROS 2 Jazzy.

Stands on its own indefinitely, drives under gamepad or keyboard control while balancing,
stops itself when the depth camera sees something in the way, and will drive around
a room on its own, turning away from whatever it finds.

| | measured |
| --- | --- |
| Continuous balance | **361 s** in one unbroken segment, still upright when recording stopped |
| Driving | **755 s** of gamepad driving with zero falls |
| Pitch error, station keeping | **0.34 deg RMS**, inside ±1.3 deg |
| Braking distance | **0.10 m** from 0.174 m/s |
| Obstacle scan | 1.98 m baseline with 0.01 m scatter; detects at 0.53–0.71 m |
| Autonomous | **486.9 s** of unbroken balance while driving, turning and reversing on its own |

## What was actually hard

A balancing robot is open-loop unstable and **non-minimum phase**: to slow a forward roll the
wheels must first drive *forward*, so the base moves ahead of the centre of mass and the body
pitches back. Get that backwards and the outer loop becomes positive feedback.

That is exactly what happened. `kp_v` was positive for three days and the robot could not hold
more than 8 seconds. The fix came from deriving the sign twice — once from the physical
transient, once from the characteristic equation `s² − K'·kp_v·s − K'·ki_v = 0`, which requires
both gains negative for stability. Setting `kp_v = −0.005` took the longest run from **7.8 s to
367 s in a single step**.

The rest of the tuning was data-driven rather than trial and error:

- **Damping vs stiffness.** A slow 0.45 Hz surge looked like too little damping. Estimating
  ζ ≈ 1.31 from the observed period showed the loop was already overdamped, so the answer was
  more position stiffness, not more damping.
- **The balance point was measured, not guessed.** While balancing with zero mean wheel speed,
  the mean pitch *is* the mechanical balance point: 7.53°. Judged by the velocity integral
  collapsing from −322 to −49 and no longer clamping.
- **Judder was identified as loop self-excitation, not mechanical resonance** — the oscillation
  frequency moved from 9.18 Hz to 7.07 Hz when a software filter coefficient changed, which a
  structural resonance cannot do. Lowering `kd` removed it at no cost to station keeping.
- **A regression was traced to a loose fastener, not to parameters.** Several unrelated metrics
  degraded at once and reverting the parameter did not restore them. That pattern is now a
  documented rule: stop tuning, inspect hardware.

Full reasoning, including the negative results and the conclusions that were later overturned,
is in [`docs/`](docs/).

## Architecture

```text
HFI-A9 IMU ──USB──► hfi_imu_node ──► /imu/data (~145 Hz)
                                         │
                                    level_frame_publisher ──► base_link_level (gravity-aligned)
                                         │
Astra depth ──► openni2 ──► /depth/image ──► obstacle_scan ──► /obstacle/scan (LaserScan, 30 Hz)
                                                                    │
explorer ──┐                                                        ▼
           ├──► /cmd_vel_raw ──► avoidance_guard ──► /cmd_vel ──► balance_controller
teleop ────┘                     (limits forward only)                 │
                                                                /motor_cmd (PWM/100)
                                                                       ▼
                                    serial_bridge ──"M,<pwm>,<pwm>"──► Arduino ──► motors
                                          ▲                               │
                                          └───"E,<left>,<right>" 100 Hz───┘
```

Each boundary in that chain exists for a reason found by breaking it:

- **`/cmd_vel` vs `/motor_cmd`.** Both teleop and the controller once wrote `/cmd_vel` and
  overwrote each other, so driving while balancing was impossible. `/cmd_vel` is now human
  intent in SI units; `/motor_cmd` is the PWM setpoint.
- **`/cmd_vel_raw` vs `/cmd_vel`.** The guard owns `/cmd_vel`; command sources publish
  `/cmd_vel_raw`. The topic name *is* the enforcement — a source that writes `/cmd_vel`
  directly bypasses the guard.
- **`base_link_level`.** The TF tree roots at `base_link`, so everything downstream implicitly
  treats the body as level. It never is. At 1 m, four degrees of tilt is 7 cm of apparent
  height — the whole difference between floor and obstacle.

Driving needed **no new control loop**. The velocity and yaw loops already regulate wheel speed
and left/right difference to zero; teleop only moves those setpoints off zero.

## Failing safe

Losing the link on a balancing robot means falling over, not stopping. Every stage fails closed:

| | |
| --- | --- |
| Command timeout | setpoint ramps to zero, robot keeps balancing in place |
| Setpoint slew limit | a step command would demand an instant lean the body cannot produce |
| Input clamp | `teleop_twist_keyboard` defaults to 0.5 m/s, past what this machine can hold |
| Deadman button | releasing stops the robot |
| No attitude → no scan | `obstacle_scan` publishes nothing rather than a scan that reads clear |
| No scan → no forward | the guard reports `STALE` and allows 0% forward |
| Unknown ≠ clear | invalid depth pixels are absence of evidence, published as `NaN`, never as free space |

That last one is the one that matters most and the one most easily got wrong — see
[`docs/08`](docs/08_MVP2深度相机接入_2026-07-28.md) for how it was got wrong here first.

## Running it

```bash
# on the robot
ros2 launch balance_bot balance.launch.py                    # stands and drives
ros2 launch balance_bot balance.launch.py record:=true bag_name:=run1
./run_camera.sh                                              # depth driver
ros2 launch balance_bot avoid.launch.py                      # perception + guard
ros2 launch balance_bot teleop_joy.launch.py cmd_topic:=/cmd_vel_raw
ros2 launch balance_bot explore.launch.py                    # autonomous
ros2 param set /explorer enabled true
```

Set the robot upright and let go; it engages itself. Analysis of a recorded run:

```bash
scp -r pi:~/ros2_ws/bags/run1_<stamp> results/rosbag/
python tools/analyze_teleop.py results/rosbag/run1_<stamp>
```

## Hardware

| Component | Role |
| --------- | ---- |
| Two-wheeled base with encoders | 277.5 counts/rev, 65 mm wheels, 220 mm track |
| Arduino Mega 2560 | TB6612 motor driver, encoder ISR, serial protocol, command watchdog |
| Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) | control, perception, logging |
| HFI-A9 IMU | fused quaternion over USB serial, driver written from the protocol |
| Orbbec Astra depth camera | 640×480 depth at 24 Hz, discontinued — see below |

The Astra is discontinued and its official OpenNI2 repository is gone. The current Orbbec SDK
does not recognise it, and Debian's `libopenni2` ships only the PrimeSense driver. Depth works
here by taking the prebuilt `liborbbec.so` out of `orbbec/ros2_astra_camera`, installing it as
`liborbbec.so.0` because Debian's loader only scans for versioned names, and running the camera
node against Orbbec's own `libOpenNI2` — Debian's enumerates the device but segfaults on stream.

## Tooling

Everything under [`tools/`](tools/) exists because something needed measuring:

| | |
| --- | --- |
| `analyze_teleop.py` | scores a run from the bag, using the controller's own recorded state |
| `analyze_run.py` | replays the controller state machine offline, for bags without it |
| `teleop_sequence.py` | scripted command sequences, repeatable across runs |
| `check_level_frame.py` | verifies the gravity frame by rotating measured gravity through it |
| `depth_geometry.py` | checks a depth frame's 3-D geometry for self-consistency |
| `loadtest_camera.sh` | measures whether streaming depth costs the control loops anything |
| `joymap.py` | maps a gamepad's real axis and button indices in one pass |

Braking distance was never measured deliberately — it was computed from bags recorded days
earlier for tuning, because every teleop command is followed by a release.

## Status

- [x] Week 1–4: IMU driver, motor and encoder verification, serial bridge, URDF and TF
- [x] **MVP0** — keyboard teleop with encoder-driven live RViz
- [x] **MVP1** — balance PID (four-stage cascade), driving under balance, gamepad, recording and analysis
- [x] **MVP2** — depth perception, gravity-aligned obstacle scan, avoidance guard, autonomous wandering
- [ ] Odometry (encoders on a pitching body do not measure ground distance directly), then SLAM

Autonomy's limit is the sensor, not the logic: with a 58° horizontal field of view the
robot cannot see where it is about to turn until it has turned, so escaping an enclosed
space is search rather than planning. A wider sensor, or remembering what was seen a
moment ago, is the next real improvement.

Known open items are recorded with reproduction steps rather than omitted — the most
significant is that driving forward can lock up below the static-friction threshold, and why
one attempt to fix it made things worse. See [`docs/07`](docs/07_遥控与录制_2026-07-26.md) §11.

## Engineering notes

- Serial devices open via `/dev/serial/by-id/` — `ttyACM*` numbering drifts across boots
- The left/right harness (motors **and** encoders) was cross-wired at assembly; normalised in
  `serial_bridge` and verified wheel by wheel
- `/joint_states` is decimated to 20 Hz; 100 Hz TF saturates a hotspot link
- The camera driver stamps from its own clock by default, which drifted 17 s and silently broke
  attitude pairing — `use_device_time:=false`
- Depth is never streamed off-board: 640×480 float32 at 24 Hz is ~29 MB/s. The robot computes
  and publishes a 64-range scan instead
