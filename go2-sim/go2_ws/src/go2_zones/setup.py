from setuptools import find_packages, setup

package_name = 'go2_zones'

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
    description='Automatic topological zone (room) segmentation of the occupancy grid for the Go2 sweep.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zone_segmenter = go2_zones.zone_segmenter:main',
        ],
    },
)
