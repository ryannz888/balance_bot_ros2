"""在 RViz 中显示 balance_bot 的 URDF 模型。

用法:
    ros2 launch balance_bot_description display.launch.py

启动三个节点:
    robot_state_publisher      读 URDF, 发布 /robot_description 和 TF
    joint_state_publisher_gui  滑块窗口, 手动拖动轮子关节验证方向
    rviz2                      可视化
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('balance_bot_description')
    # mesh 版（SW导出）；调试嫌卡可换回简版 'balance_bot.urdf'
    urdf_path = os.path.join(pkg_share, 'urdf', 'balance_bot_mesh.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])
