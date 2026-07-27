# balance_bot_ros2

Two-wheeled self-balancing robot based on ROS2 Jazzy | Raspberry Pi 5 + Arduino + IMU + PID balance control

**MVP0 demo:** keyboard teleop with encoder-driven live RViz visualization → [videos/mvp0_teleop_rviz_demo.mp4](videos/mvp0_teleop_rviz_demo.mp4)

## Hardware

| Component | Role |
| --------- | ---- |
| Two-wheeled self-balancing base (with encoders) | Mobile platform |
| Arduino Mega 2560 | Motor driver (TB6612) + encoder ISR + serial protocol |
| Raspberry Pi 5 (Ubuntu, ROS2 Jazzy) | ROS2 main compute |
| HFI-A9 IMU | Orientation via USB serial, custom driver, ~285Hz |
| Orbbec Astra depth camera | Obstacle detection (MVP2) |

## System Architecture

```text
HFI-A9 IMU ──USB──► hfi_imu_node ──► /imu/data (sensor_msgs/Imu, ~145Hz)
                                          │
teleop ──/cmd_vel──► balance_controller ◄─┘
       (SI units,           │  └──► /balance/state (controller's own view, 48Hz)
        human intent)       │
                    /motor_cmd (PWM/100)
                            ▼
                      serial_bridge ──"M,<pwm>,<pwm>"──► Arduino Mega ──► TB6612 ──► motors
                            ▲                                  │
                            └────"E,<left>,<right>" (100Hz)────┘  encoder counts
                            │
                            ├──► /wheel/encoders (raw counts, left/right normalized)
                            └──► /joint_states (wheel angles, 20Hz) ──► robot_state_publisher ──► /tf

URDF (SolidWorks-exported meshes + hand-written sensor frames) ──► /robot_description
```

`/cmd_vel` carries **velocity intent in SI units**; `/motor_cmd` carries the PWM
setpoint.  Splitting them is what lets teleop and the balance controller coexist —
before, both wrote `/cmd_vel` and overwrote each other.  `bringup.launch.py` still
defaults `serial_bridge` to `/cmd_vel`, so MVP0 keyboard-direct driving is unchanged.

Driving adds no new control loop: the velocity and yaw loops already regulate wheel
speed and left/right difference to **zero**, so teleop just moves those setpoints.

Frame convention: REP-103 (X forward, Y left, Z up). `base_link` origin at wheel-axle midpoint.
Tilt convention for balance control: **forward tilt = positive pitch**.

## Current Status

- [x] Week 1: HFI-A9 IMU driver written from scratch (protocol reverse-engineered), `/imu/data` publishing
- [x] Week 2: Dual motor + encoder verified, `E,`/`M,` serial protocol working
- [x] Week 3: `serial_bridge` node — `/cmd_vel` → PWM downlink, encoder uplink
- [x] Week 4: URDF (SW2URDF export + fixes), RViz visualization, TF tree with sensor frames
- [x] **MVP0 (2026-07-19): keyboard teleop + encoder-driven RViz live sync** ✅
- [x] **MVP1a (2026-07-25): balance PID — four-stage cascade, 228s continuous** ✅
- [x] **MVP1b (2026-07-27): driving under balance + integrated rosbag recording** ✅
      — 361s continuous balance, drive/reverse/yaw commands tracked, one-command capture
- [ ] MVP2: Depth-camera obstacle avoidance + state machine (stretch goal)

## Running it

```bash
ros2 launch balance_bot balance.launch.py                          # balance only
ros2 launch balance_bot balance.launch.py record:=true bag_name:=run1   # + rosbag
ros2 run teleop_twist_keyboard teleop_twist_keyboard               # drive it
ros2 launch balance_bot teleop_joy.launch.py                       # or a gamepad
```

Set the robot upright and let go; it engages itself.  Analysis of a recorded run:

```bash
scp -r pi:~/ros2_ws/bags/run1_<stamp> results/rosbag/
python tools/analyze_teleop.py results/rosbag/run1_<stamp>
```

## Quick Start (on the robot)

```bash
cd ~/ros2_ws && colcon build --packages-select balance_bot balance_bot_description
source install/setup.bash
ros2 launch balance_bot bringup.launch.py     # IMU + serial bridge + robot_state_publisher

# In a second terminal: keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# On a remote machine (CycloneDDS static-peer config): live model view
rviz2    # Fixed Frame: base_link, add RobotModel on /robot_description
```

URDF-only preview (no hardware):

```bash
ros2 launch balance_bot_description display.launch.py
```

## Engineering Notes (hard-won)

- Serial devices are opened via `/dev/serial/by-id/` stable paths — `ttyACM*` numbering drifts across boots
- The left/right harness (motors **and** encoders) was found cross-wired at assembly; mapping is
  normalized in `serial_bridge` against the URDF convention, verified wheel-by-wheel
- `/joint_states` is decimated to 20Hz — 100Hz TF over Wi-Fi saturates the link and causes replay lag
- Cross-machine DDS uses CycloneDDS with explicit peer IPs (multicast unreliable on hotspot networks)
- Full incident logs: [docs/](docs/)

## Project Structure

```text
balance_bot_ros2/           # ROS2 package: balance_bot (python)
  balance_bot/              #   hfi_imu_node / serial_bridge / imu_reader (debug)
  launch/bringup.launch.py  #   one-command bringup
  arduino/firmware/         # Arduino Mega firmware (step1-4)
  src/balance_bot_description/  # URDF (mesh + primitive), meshes, display launch
  docs/                     # Issue/fix logs per milestone
  results/                  # Screenshots, TF tree, rosbags
  videos/                   # Demo recordings
```
