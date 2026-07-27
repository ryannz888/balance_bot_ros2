"""Gamepad teleop: /joy -> /cmd_vel.

Run alongside balance.launch.py, which owns the robot.  This only produces the
velocity intent, so it is safe to start and stop at will:

    ros2 launch balance_bot balance.launch.py
    ros2 launch balance_bot teleop_joy.launch.py

game_controller_node is preferred over joy_node because it maps every pad brand
onto one SDL layout, so swapping controllers does not mean re-deriving button
indices.

Needs: sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('balance_bot')
    params = os.path.join(share, 'config', 'teleop_joy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='Joystick index, /dev/input/js<N>'),
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                # Ignore tiny stick offsets so a centred stick commands nothing.
                'deadzone': 0.08,
                # Republish while a stick is held; the controller's command
                # timeout would otherwise zero the setpoint during a steady hold.
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[params],
        ),
    ])
