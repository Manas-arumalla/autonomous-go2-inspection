from setuptools import find_packages, setup
package_name = 'go2_inspection'
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
    description='Go2 autonomous facility inspection: viewpoint+spin YOLOE engine (zone_inspector) '
                '+ map-driven wall-follower + Claude gauge reading + ROS service layer + MCP.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        # --- inspection engine (converged from -main; ADR-016) ---
        'zone_inspector = go2_inspection.zone_inspector:main',
        # --- existing path (kept until the convergence is validated; ADR-016 M6 retires the legacy) ---
        'zone_sweeper = go2_inspection.zone_sweeper:main',
        'zone_wall_follower = go2_inspection.zone_wall_follower:main',
        'panorama_segmenter = go2_inspection.panorama_segmenter:main',
        'yoloe_segmenter = go2_inspection.yoloe_segmenter:main',
        'inspection_mission = go2_inspection.inspection_mission:main',
        'mission_control_server = go2_inspection.mission_control_server:main',
    ]},
)
