"""Perception, arbitration, guard, autonomy and gamepad, in one command.

    ros2 launch balance_bot balance.launch.py     # first: the robot must stand
    ./run_camera.sh                               # and see
    ros2 launch balance_bot autonomy.launch.py    # then this

The robot explores on its own and hands control to the gamepad the moment
someone holds the deadman button, returning to exploring shortly after they let
go.  Nothing needs switching by hand.

Command flow:

    explorer  -> /cmd_vel_auto   ─┐
                                  ├─ cmd_mux -> /cmd_vel_raw -> guard -> /cmd_vel
    teleop    -> /cmd_vel_manual ─┘

Every source passes through the guard, so neither autonomy nor a human can
drive further forward than the obstacle scan allows.

`explore:=false` brings up perception, the guard and the gamepad without
autonomy, which is manual driving with obstacle limiting.  The explorer node
itself still defaults to disabled when run bare; it is enabled here because
launching this file is the deliberate act.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('balance_bot')
    joy_params = os.path.join(share, 'config', 'teleop_joy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('explore', default_value='true',
                              description='false leaves manual driving with the guard'),
        DeclareLaunchArgument('device_id', default_value='0'),

        # Gravity-aligned frame: heights mean nothing without it.
        Node(package='balance_bot', executable='level_frame_publisher',
             name='level_frame_publisher', output='screen'),
        Node(package='balance_bot', executable='obstacle_scan',
             name='obstacle_scan', output='screen'),

        Node(package='balance_bot', executable='cmd_mux',
             name='cmd_mux', output='screen'),
        Node(package='balance_bot', executable='avoidance_guard',
             name='avoidance_guard', output='screen'),

        Node(package='balance_bot', executable='explorer', name='explorer',
             output='screen',
             parameters=[{'enabled': True}],
             condition=IfCondition(LaunchConfiguration('explore'))),

        # Gamepad.  Publishing /cmd_vel_manual is what makes the mux hand over.
        Node(package='joy', executable='game_controller_node', name='joy_node',
             parameters=[{'device_id': LaunchConfiguration('device_id'),
                          'deadzone': 0.08,
                          'autorepeat_rate': 20.0}]),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node',
             parameters=[joy_params],
             remappings=[('/cmd_vel', '/cmd_vel_manual')]),
    ])
