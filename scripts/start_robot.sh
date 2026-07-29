#!/bin/bash
# Bring the whole robot up, in order, for an unattended boot.
#
# Ordering is not cosmetic.  The balance controller will not engage until it has
# IMU data and a subscriber on its output, obstacle_scan refuses to publish
# without attitude, and the guard reports STALE and allows no forward motion
# until a scan arrives.  Starting these out of order is safe -- every stage
# fails closed -- but it produces a minute of confusing log lines, so wait.
#
# USB enumeration is slower than systemd.  The serial devices and the camera
# appear a few seconds after userspace starts, so poll for them rather than
# assuming.
set -o pipefail

WS=/home/ryannz/ros2_ws
LOGDIR=/home/ryannz/robot_logs
KEEP=20                      # boots of history to retain
STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$LOGDIR/boot_$STAMP.log"

# Refuse to stack a second copy on top of a running one.  Every node here
# opens an exclusive resource -- two serial ports and a USB camera -- and a
# second set does not fail loudly, it produces 67 serial errors and three
# controllers fighting over one Arduino.  Seen exactly that after a manual run
# and a systemctl start overlapped.
for n in openni2_camera_driver hfi_imu_node serial_bridge          balance_bot/balance_controller obstacle_scan avoidance_guard          cmd_mux balance_bot/explorer; do
    pkill -f "$n" 2>/dev/null
done
pkill -f 'ros2 launch balance_bot' 2>/dev/null
pkill -TERM -f 'ros2 bag record' 2>/dev/null
sleep 4

mkdir -p "$LOGDIR"
# Prune oldest first so a long-running robot cannot fill the card with logs.
ls -1t "$LOGDIR"/boot_*.log 2>/dev/null | tail -n +$((KEEP + 1)) | xargs -r rm -f

exec > >(tee -a "$LOG") 2>&1
echo "=== boot $STAMP ==="

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"

wait_for () {   # wait_for <path> <seconds> <label>
    local i=0
    while [ ! -e "$1" ] && [ $i -lt "$2" ]; do sleep 1; i=$((i + 1)); done
    if [ -e "$1" ]; then
        echo "$3 present after ${i}s"
    else
        echo "$3 MISSING after ${2}s -- continuing, the stack fails closed without it"
    fi
}

wait_for /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 30 "IMU"
wait_for /dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85431303636351613122-if00 30 "Arduino"

echo "--- camera ---"
setsid "$WS/run_camera.sh" >"$LOGDIR/camera_$STAMP.log" 2>&1 </dev/null &
sleep 12

echo "--- balance ---"
setsid ros2 launch balance_bot balance.launch.py \
    >"$LOGDIR/balance_$STAMP.log" 2>&1 </dev/null &
sleep 14

echo "--- autonomy ---"
setsid ros2 launch balance_bot autonomy.launch.py \
    >"$LOGDIR/autonomy_$STAMP.log" 2>&1 </dev/null &
sleep 8

if [ "${RECORD:-1}" = "1" ]; then
    # Deliberately excludes /depth/image.  It is about 29 MB/s and once filled
    # a fifth of the card in six minutes; everything needed to reconstruct what
    # the robot decided is in the scan and the command topics.
    echo "--- recording ---"
    setsid ros2 bag record --storage mcap \
        --max-bag-size 268435456 --max-cache-size 10485760 \
        -o "$LOGDIR/bag_$STAMP" \
        /imu/data /wheel/encoders /cmd_vel /cmd_vel_raw /cmd_vel_auto \
        /cmd_vel_manual /motor_cmd /balance/state /obstacle/scan \
        /obstacle/guard_state /explorer/state /cmd_mux/mode \
        >"$LOGDIR/bagrec_$STAMP.log" 2>&1 </dev/null &
    sleep 4
fi

echo "--- running ---"
for n in openni2_camera_driver hfi_imu_node serial_bridge \
         balance_bot/balance_controller obstacle_scan avoidance_guard \
         cmd_mux balance_bot/explorer; do
    pgrep -f "$n" >/dev/null && echo "  up   $n" || echo "  DOWN $n"
done

echo "=== boot complete, log: $LOG ==="
# systemd keeps the unit active while this lives; the children outlive a exit.
while true; do sleep 3600; done
