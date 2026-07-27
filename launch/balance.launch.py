"""Bring up the sensors and drivers, then the tuned balance controller.

Kept separate from bringup.launch.py so the hardware can be started without
anything driving the motors.

Topic layering (established 2026-07-26 when teleop was added):
    /cmd_vel    human velocity intent in SI units, published by a teleop node
    /motor_cmd  this controller's PWM output, consumed by serial_bridge
bringup.launch.py defaults serial_bridge to /cmd_vel so the MVP0 keyboard-direct
workflow still works; here it is remapped so teleop and the controller do not
fight over the same topic.

Usage:
    ros2 launch balance_bot balance.launch.py
    ros2 launch balance_bot balance.launch.py record:=true bag_name:=teleop1
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Everything an offline replay needs.  /balance/state carries the controller's
# own view (engaged flag, trim, setpoints) so analysis no longer has to
# reconstruct the state machine from raw signals.
RECORD_TOPICS = [
    '/imu/data',
    '/wheel/encoders',
    '/joint_states',
    '/cmd_vel',
    '/motor_cmd',
    '/balance/state',
]


def generate_launch_description():
    share = get_package_share_directory('balance_bot')
    params = os.path.join(share, 'config', 'balance_params.yaml')
    bags_dir = os.path.expanduser('~/ros2_ws/bags')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false',
                              description='Record a rosbag of the whole run'),
        DeclareLaunchArgument('bag_name', default_value='run',
                              description='Bag name prefix; a timestamp is appended'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'motor_cmd_topic': '/motor_cmd'}.items(),
        ),
        Node(
            package='balance_bot',
            executable='balance_controller',
            name='balance_controller',
            parameters=[params],
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=['ros2', 'bag', 'record', '--storage', 'mcap',
                 '-o', [os.path.join(bags_dir, ''),
                        LaunchConfiguration('bag_name'), '_', stamp]] + RECORD_TOPICS,
            output='screen',
        ),
    ])
