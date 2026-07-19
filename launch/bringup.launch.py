import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('balance_bot_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'balance_bot_mesh.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # IMU驱动（发布/imu/data）。注意imu_reader只是订阅端调试工具，不是驱动
        Node(
            package='balance_bot',
            executable='hfi_imu_node',
            name='hfi_imu',
            parameters=[{
                'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            }],
        ),
        # serial_bridge现在也从编码器发布/joint_states（标定后修改ticks_per_rev）
        Node(
            package='balance_bot',
            executable='serial_bridge',
            name='serial_bridge',
            # 2026-07-19手转标定：约277.5计数/圈（精标定留给MVP1里程计）
            # 字段对调归一后实测：左轮前滚计数为正(不翻转)、右轮为负(翻转)
            parameters=[{
                'ticks_per_rev': 277.5,
                'invert_left': False,
                'invert_right': True,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
