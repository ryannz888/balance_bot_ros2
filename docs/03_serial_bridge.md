# Stage 3: ROS2 Serial Bridge Node

> Date: 2026-04-02
> Environment: Raspberry Pi 5 + Ubuntu 24.04 + ROS2 Jazzy + Arduino Mega 2560

## Result

- `serial_bridge` node running, `/cmd_vel` controls motors via serial
- `/wheel/encoders` topic publishing encoder data at ~100Hz
- Keyboard teleoperation working — instant response
- `bringup.launch.py` launches IMU + serial_bridge with one command

---

## Architecture

```
teleop_twist_keyboard
        ↓ /cmd_vel (geometry_msgs/Twist)
  serial_bridge node
        ↓ "M,left_pwm,right_pwm\n"  (USB serial, 115200 baud)
    Arduino Mega
        ↓ PWM
      TB6612FNG → MG310P20 motors

    Arduino Mega
        ↑ "E,left_count,right_count\n"  (every 10 ms)
  serial_bridge node
        ↑ /wheel/encoders (std_msgs/String)
```

---

## Node: serial_bridge

**File:** `src/balance_bot/balance_bot/serial_bridge.py`

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist) — velocity command input

**Publications:**
- `/wheel/encoders` (std_msgs/String) — encoder counts, format `"left,right"`

**Key logic:**

```python
# cmd_vel → PWM conversion
left_pwm  = int((linear.x - angular.z) * 100)
right_pwm = int((linear.x + angular.z) * 100)

# Serial read thread — runs independently to avoid blocking rclpy.spin()
threading.Thread(target=self._read_serial, daemon=True).start()
```

---

## Launch File

**File:** `src/balance_bot/launch/bringup.launch.py`

```bash
ros2 launch balance_bot bringup.launch.py
```

Starts both nodes simultaneously:
- `hfi_imu` → publishes `/imu/data`
- `serial_bridge` → publishes `/wheel/encoders`, listens on `/cmd_vel`

---

## Issues & Fixes

**1. `pip3 install pyserial` blocked by system package manager**

Debian 12+ protects the system Python environment. Fix:
```bash
sudo apt install python3-serial
```

**2. TabError: inconsistent use of tabs and spaces**

nano mixed tabs and spaces when pasting code. Fix: rewrite the file entirely using only spaces (4-space indent throughout).

**3. ROS2 topic discovery latency (~2 minutes on first publish)**

Symptom: `ros2 topic pub --once` command returns immediately but motor takes minutes to respond.

Root cause: FastDDS multicast discovery is slow on Raspberry Pi with multiple network interfaces. Tried `ROS_LOCALHOST_ONLY=1`, CycloneDDS with `lo` interface, static peers — none fully resolved the issue on this setup.

Workaround: Use `-r 10` (continuous publish at 10 Hz) instead of `--once`. Once connection is established, response is instant. In practice this is not an issue because `teleop_twist_keyboard` maintains a persistent connection.

---

## Lessons Learned

| # | Lesson |
|---|--------|
| 1 | Serial read must run in a dedicated thread — `readline()` blocks, cannot run inside `rclpy.spin()` |
| 2 | Use `sudo apt install python3-serial` on Debian-based systems, not `pip3 install pyserial` |
| 3 | `ros2 topic pub --once` is unreliable for testing when DDS discovery is slow; use `-r 10` instead |
| 4 | `/cmd_vel` (geometry_msgs/Twist) is the standard ROS2 velocity interface — always prefer it over custom topics for motion commands |
| 5 | `data_files` in `setup.py` must explicitly include the `launch/` directory, and requires `import os` + `from glob import glob` |
