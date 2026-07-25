"""Bring up the sensors and drivers, then the tuned balance controller.

Kept separate from bringup.launch.py so the hardware can be started without
anything driving the motors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('balance_bot')
    params = os.path.join(share, 'config', 'balance_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'bringup.launch.py')),
        ),
        Node(
            package='balance_bot',
            executable='balance_controller',
            name='balance_controller',
            parameters=[params],
            output='screen',
        ),
    ])
