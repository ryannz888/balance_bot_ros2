#!/bin/bash
# Start the OpenNI2 camera driver against Orbbec's own libOpenNI2.
#
# The Astra Pro needs liborbbec.so, which Debian's libopenni2 does not ship.
# Dropping that driver into the system directory is enough to ENUMERATE the
# device, but opening a stream then segfaults: Debian's libOpenNI2 2.2.0.33 and
# Orbbec's driver are not fully ABI compatible.  Orbbec ships and tests its own
# libOpenNI2 alongside the driver, so point the loader at that pair instead of
# replacing the system library, which other packages link against.
ORBBEC_HOME=/home/ryannz/orbbec_openni2
export LD_LIBRARY_PATH=$ORBBEC_HOME:/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/ryannz/ros2_ws/install/setup.bash
# use_device_time defaults to true, which stamps frames from the camera's own
# clock.  That clock is not disciplined to system time: measured delay between
# stamp and arrival drifted to -17.5 s with a 1.9 s standard deviation, then
# settled near 41 ms, then drifted again.  Frames arriving with stamps seconds
# in the future silently destroy any time-based fusion -- pairing depth with
# body attitude read 15.6 s stale, then correct, then 9.5 s stale, with nothing
# in the logs to suggest why.  Publish-time stamps carry a consistent ~41 ms of
# pipeline latency instead, which is a known constant rather than a moving one.
# The parameter is not settable at runtime, so it has to go here.
exec /opt/ros/jazzy/lib/openni2_camera/openni2_camera_driver     --ros-args -p use_device_time:=false "$@"
