import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manas Arumalla',
    maintainer_email='manasreddyarumalla@gmail.com',
    description='Bridge nodes for Unitree Go2 on ROS2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf_bridge = go2_bridge.odom_tf_bridge:main',
            'joint_state_bridge = go2_bridge.joint_state_bridge:main',
            'cmd_vel_bridge = go2_bridge.cmd_vel_bridge:main',
            'topic_monitor = go2_bridge.topic_monitor:main',
            'sensor_bridge = go2_bridge.sensor_bridge:main',
            'camera_bridge = go2_bridge.camera_bridge:main',
        ],
    },
)
