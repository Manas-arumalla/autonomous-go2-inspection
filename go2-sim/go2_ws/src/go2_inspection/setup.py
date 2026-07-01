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
    maintainer='Manas Arumalla',
    maintainer_email='manasreddyarumalla@gmail.com',
    description='Go2 autonomous facility inspection: viewpoint+spin YOLOE engine (zone_inspector) '
                '+ map-driven wall-follower + LLM gauge reading + ROS service layer + MCP.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        # --- inspection engine (converged from -main; ADR-016) ---
        'zone_inspector = go2_inspection.zone_inspector:main',
        # (ADR-016 M6: the legacy wall-follower nodes — zone_sweeper / zone_wall_follower /
        #  panorama_segmenter / yoloe_segmenter — were retired; zone_inspector supersedes them.)
        'inspection_mission = go2_inspection.inspection_mission:main',
        'mission_control_server = go2_inspection.mission_control_server:main',
        # --- benchmarking: score a run vs world ground truth (ADR-016 M7b) ---
        'benchmark = go2_inspection.benchmark:_cli',
        # --- visualization: publish detected gauges as RViz markers (ADR-018 demo viz) ---
        'inspection_markers = go2_inspection.inspection_markers:main',
    ]},
)
