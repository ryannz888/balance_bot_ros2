# Week 1 Troubleshooting: HFI-A9 IMU Integration

> Date: 2026-03-24
> Environment: Raspberry Pi 5 + Ubuntu 24.04 + ROS2 Jazzy

---

## Final Result

- HFI-A9 IMU successfully integrated, `/imu/data` topic publishing at ~**295Hz**
- `imu_reader` node running, Pitch angle changes in real time when IMU is tilted
- All Week 1 requirements completed

---

## Issue 1: `ros2 --version` error

**Symptom:** `ros2 --version` reports `unrecognized arguments`

**Cause:** ROS2 CLI does not support the `--version` flag

**Fix:**
```bash
printenv ROS_DISTRO   # correct way to check ROS2 version
```

---

## Issue 2: `cp` command caused nested directory mess

**Symptom:** Copying driver package from `products` repo into `src/` while already inside `src/` created a `src/src/` nested structure; colcon reported duplicate package error

**Cause:** `cp` executed inside `src/`, with `products` repo also inside `src/`, causing recursive path collision

**Fix:**
```bash
cd ~/ros2_ws/src
rm -rf products src   # delete products repo and the mistaken src/src/
# keep only hipnuc_imu and hipnuc_lib_package
```

---

## Issue 3: System bzip2 dependency conflict, cannot install build-essential

**Symptom:**
```
libbz2-dev: Depends: libbz2-1.0 (= 1.0.8-5.1) but 1.0.8-5.1build0.1 is to be installed
```

**Cause:** Installed `libbz2-1.0` is the arm64 rebuild version (`build0.1` suffix), exact version mismatch with `bzip2` requirement

**Fix:**
```bash
sudo apt install libbz2-1.0=1.0.8-5.1 --allow-downgrades
sudo apt install build-essential -y
```

---

## Issue 4: hipnuc driver switches all default to FALSE, no data published

**Symptom:** After launch, `ros2 topic hz /IMU_data` shows "does not appear to be published yet"

**Cause:** Three switches in `hipnuc_config.yaml` default to `FALSE`:
```yaml
imu_switch: FALSE
euler_switch: FALSE
magnetic_switch: FALSE
```

**Fix:**
```bash
sed -i 's/: FALSE/: TRUE/g' ~/ros2_ws/src/hipnuc_imu/config/hipnuc_config.yaml
colcon build --packages-select hipnuc_imu
```

---

## Issue 5: Internal QoS incompatibility warnings

**Symptom:** launch logs continuously show `RELIABILITY_QOS_POLICY` incompatibility warnings

**Cause:** hipnuc package's internal publisher uses `BEST_EFFORT`, built-in listener uses `RELIABLE`

**Impact:** Does not affect external subscribers — can be ignored. Add `--qos-reliability best_effort` when subscribing externally

---

## Issue 6: No data even after enabling switches (root cause)

**Symptom:** `ros2 topic echo --qos-reliability best_effort /IMU_data` produces no output

**Investigation:**
1. Read source `serial_port.cpp` — sync header is `0x5A 0xA5`
2. Used Python to read raw serial data — no `5A A5` found in 500 bytes
3. Tried different baud rates (9600, 115200, 460800) — still no `5A A5`

**Root causes (two):**
1. **Wrong driver entirely:** hipnuc driver sync header is `5A A5`, but HFI-A9 actual protocol sync header is `AA 55`
2. **Wrong baud rate:** config says 115200, HFI-A9 actual default baud rate is **921600**

**Conclusion:** HFI-A9 is a HandsFree/TaoBotic product, not a genuine HiPNUC device — hipnuc driver is completely incompatible

---

## Issue 7: Official driver is ROS1, cannot be used with ROS2

**Symptom:** `handsfree_ros_imu` repo is a ROS1 catkin package using `rospy`, cannot compile under ROS2 Jazzy

**Fix:** Referenced the protocol parsing logic from the official ROS1 script `hfi_a9_ros.py` and wrote a ROS2 Python node from scratch

Key protocol parameters (reverse-engineered from ROS1 source):
- Sync header: `0xAA 0x55`
- Baud rate: 921600
- Packet type `0x2C` (44 bytes): gyro + accelerometer + magnetometer raw data
- Packet type `0x14` (20 bytes): Roll/Pitch/Yaw Euler angles
- Checksum: CRC16

---

## Final Solution

**File:** `src/balance_bot/balance_bot/hfi_imu_node.py` (standalone ROS2 Python node)

**How to run:**
```bash
# Terminal 1: start IMU node
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/balance_bot/balance_bot/hfi_imu_node.py

# Terminal 2: run imu_reader
source ~/ros2_ws/install/setup.bash
ros2 run balance_bot imu_reader
```

**Verified results:**
- Topic: `/imu/data` (sensor_msgs/Imu)
- Frequency: ~295Hz
- Data: quaternion, angular velocity, linear acceleration all normal

---

## Issue 8: PC (VM) cannot discover ROS2 topics from Raspberry Pi

**Symptom:** Node running normally on Pi, `ros2 topic list` visible on Pi itself, but completely invisible from PC VM

**Investigation:**
1. ping succeeds → basic network is fine
2. domain ID matches (both default 0) → not a domain ID issue
3. Setting `CYCLONEDDS_URI` had no effect → because default RMW is FastDDS, not CycloneDDS

**Root cause:** VM network adapters typically do not forward DDS multicast packets; FastDDS relies on multicast for node discovery by default

**Fix:** Switch both sides to CycloneDDS and specify peer IP for unicast discovery

```bash
# Install and switch RMW on both machines
sudo apt install ros-jazzy-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# On Pi (pointing to PC IP)
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><Peers><Peer address="192.168.1.1"/></Peers></Discovery></Domain></CycloneDDS>'

# On PC (pointing to Pi IP)
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><Peers><Peer address="192.168.1.16"/></Peers></Discovery></Domain></CycloneDDS>'
```

**Note:** Must re-export in every new terminal. Add to `~/.bashrc` for permanent effect.

---

## Lessons Learned

| # | Lesson |
| - | ------ |
| 1 | Always verify manufacturer and protocol before using a driver — never assume compatibility |
| 2 | `ros2 topic list` showing a topic ≠ topic has data — always verify with `hz` and `echo` |
| 3 | When official driver is ROS1, read the protocol parsing logic and write your own ROS2 version |
| 4 | Resolve system package dependency conflicts with `--allow-downgrades` |
| 5 | For serial debugging, use `xxd` to inspect raw bytes first — confirm sync header and baud rate |
| 6 | For ROS2 cross-machine comms in VM: switch to CycloneDDS + manually specify peer IP |
