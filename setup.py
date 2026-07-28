from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'balance_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
	(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryannz',
    maintainer_email='ryannz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'imu_reader = balance_bot.imu_reader:main',
		'serial_bridge = balance_bot.serial_bridge:main',
		'hfi_imu_node = balance_bot.hfi_imu_node:main',
		'balance_controller = balance_bot.balance_controller:main',
		'level_frame_publisher = balance_bot.level_frame_publisher:main',
		'obstacle_scan = balance_bot.obstacle_scan:main',
		'avoidance_guard = balance_bot.avoidance_guard:main',
        ],
    },
)
