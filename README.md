# balance_bot_ros2

Two-wheeled self-balancing robot based on ROS2 Jazzy | Raspberry Pi 5 + Arduino + IMU + PID balance control

## Hardware

| Component | Role |
| --------- | ---- |
| Two-wheeled self-balancing base (with encoders) | Mobile platform |
| Arduino Mega 2560 | Motor driver + encoder reading + serial communication |
| Raspberry Pi 5 (Ubuntu 22.04) | ROS2 main compute |
| HFI-A9 ROS IMU | Tilt angle / angular velocity via USB |
| Orbbec Astra depth camera | Obstacle detection |

## System Architecture

```text
HFI-A9 IMU ──USB──┐
                   ├──► Raspberry Pi 5 (ROS2 Jazzy)
Orbbec Astra ──USB─┘         │
                         ┌───┴──────────────────────┐
                         │  /imu/data               │  ← HFI-A9
                         │  balance_controller      │  ← PID node
                         │  serial_bridge           │  ← serial bridge node
                         │  depth_obstacle          │  ← obstacle node
                         │  task_coordinator        │  ← state machine node
                         └──────────┬───────────────┘
                                    │ USB Serial
                           Arduino Mega 2560
                           Encoder reading + Motor PWM
```

## Environment

- ROS2 Jazzy
- Ubuntu 22.04
- Raspberry Pi 5

## Current Status

- [x] Project scaffold
- [ ] MVP0: Communication links verified (target: 2026-04-13)
- [ ] MVP1: Balance PID running (target: 2026-05-11)
- [ ] MVP2: Full system with obstacle avoidance (target: 2026-06-15)

## Next Steps

1. Flash Ubuntu 22.04 to SD card, boot Raspberry Pi 5
2. Install ROS2 Jazzy
3. Connect HFI-A9 IMU, verify `/imu/data` topic
4. Write `imu_reader` practice node

## Project Structure

```text
balance_bot_ros2/
  docs/               # Architecture diagram, wiring diagram, PID tuning notes
  src/
    balance_bot/      # Main ROS2 package
    balance_bot_description/  # URDF + RViz
  arduino/firmware/   # Arduino firmware
  results/            # rosbag recordings, plots
  videos/             # Demo videos
```
