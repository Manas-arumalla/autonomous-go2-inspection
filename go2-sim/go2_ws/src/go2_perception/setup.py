from setuptools import find_packages, setup

package_name = 'go2_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EE26 team',
    maintainer_email='manasreddyarumalla@gmail.com',
    description='Go2 inspection perception: RGB+LiDAR colorization (and later SAM3/Claude reports).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_colorize = go2_perception.lidar_colorize:main',
        ],
    },
)
