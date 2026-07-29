"""Autonomous wandering, on top of a running balance stack and obstacle guard.

Separate from avoid.launch.py on purpose.  The guard is a safety layer and
should be running whenever the robot can move, including under manual control.
Autonomy is optional and is what you turn on when you want a demonstration.
Bundling them would mean you could not have the safety without the autonomy.

    ros2 launch balance_bot balance.launch.py
    ros2 launch balance_bot avoid.launch.py
    ros2 launch balance_bot explore.launch.py
    ros2 param set /explorer enabled true

The explorer starts disabled.  Something that drives itself should require a
deliberate act to begin, not merely a launch file that ran.

Manual and autonomous command sources both publish /cmd_vel_raw and will fight
if both are live, so run teleop or the explorer, not both.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='balance_bot',
            executable='explorer',
            name='explorer',
            output='screen',
        ),
    ])
