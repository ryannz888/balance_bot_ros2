#!/bin/bash
# Measure whether streaming depth degrades the control loops.
#
# `ros2 topic hz` is not trustworthy for this: it is a python node whose own
# scheduling perturbs what it measures, and short windows gave contradictory
# answers (IMU appeared *faster* with the camera on).  Record the topics
# instead and compute inter-message intervals offline, where the measurement
# costs nothing.
#
# The subscriber matters as much as the driver.  image_transport publishers are
# lazy: with nobody subscribed the driver sits at ~2% CPU and produces no
# frames, so measuring that condition says nothing about the real cost.  Phase B
# therefore attaches a subscriber to force actual streaming.
#
#   ./loadtest_camera.sh [seconds per phase]
# No `set -u` here: ROS's setup.bash references unbound variables internally
# and aborts under it.
DUR="${1:-45}"

source /opt/ros/jazzy/setup.bash
source /home/ryannz/ros2_ws/install/setup.bash
cd /home/ryannz/ros2_ws
rm -rf bags/load_camoff bags/load_camon

echo "--- phase A: camera OFF (${DUR}s) ---"
pkill -f '[o]penni2_camera_driver' || true
sleep 4
ros2 bag record --storage mcap -o bags/load_camoff /imu/data /wheel/encoders >/dev/null 2>&1 &
sleep "$DUR"
pkill -TERM -f '[r]os2 bag' || true
sleep 4

echo "--- phase B: camera ON and streaming (${DUR}s) ---"
setsid nohup /home/ryannz/ros2_ws/run_camera.sh >/tmp/cam.log 2>&1 </dev/null &
sleep 12
setsid nohup ros2 topic hz /depth/image >/tmp/depthhz.log 2>&1 </dev/null &
sleep 5
ros2 bag record --storage mcap -o bags/load_camon /imu/data /wheel/encoders >/dev/null 2>&1 &
sleep "$DUR"
pkill -TERM -f '[r]os2 bag' || true
sleep 4
pkill -f 'topic hz' || true

echo "depth rate during phase B:"
grep average /tmp/depthhz.log | tail -1
echo "cpu during phase B:"
ps -eo pcpu,comm --sort=-pcpu | head -6
echo LOADTEST_DONE
