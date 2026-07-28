"""Depth perception and the obstacle guard, on top of a running balance stack.

Kept separate from balance.launch.py so the robot can be driven with the guard
absent, and so a camera fault cannot take the balance loop down with it.

    ros2 launch balance_bot balance.launch.py          # robot stands and drives
    ros2 launch balance_bot avoid.launch.py            # add perception + guard
    ros2 launch balance_bot teleop_joy.launch.py cmd_topic:=/cmd_vel_raw

The guard inserts itself by owning /cmd_vel: command sources publish
/cmd_vel_raw instead, and the guard forwards a limited version.  Launching this
without redirecting the command source leaves teleop writing straight to
/cmd_vel, which bypasses the guard entirely -- the topic name is the enforcement
mechanism, so it has to be got right.

The camera driver is not launched here.  It needs Orbbec's own OpenNI2 build on
LD_LIBRARY_PATH (see tools/run_camera.sh for why), which is a shell concern
rather than a launch-file one.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Gravity-aligned frame.  Nothing downstream can measure height against
        # the floor without it, because the body is never level.
        Node(
            package='balance_bot',
            executable='level_frame_publisher',
            name='level_frame_publisher',
            output='screen',
        ),
        Node(
            package='balance_bot',
            executable='obstacle_scan',
            name='obstacle_scan',
            output='screen',
        ),
        Node(
            package='balance_bot',
            executable='avoidance_guard',
            name='avoidance_guard',
            output='screen',
        ),
    ])
