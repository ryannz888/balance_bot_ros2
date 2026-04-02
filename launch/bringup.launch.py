from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='balance_bot',
            executable='imu_reader',
            name='hfi_imu',
        ),
        Node(
            package='balance_bot',
            executable='serial_bridge',
            name='serial_bridge',
        ),
    ])
