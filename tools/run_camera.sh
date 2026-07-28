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
exec /opt/ros/jazzy/lib/openni2_camera/openni2_camera_driver "$@"
