"""mission.launch.py -- ONE COMMAND: the full autonomous gauge-inspection mission.

Brings up the navigation foundation (inspection_nav: sim + rtabmap localization + static map_server +
Nav2) and then runs the inspection_mission orchestrator, which from HOME autonomously visits each gauge
room -> sweeps -> segments -> (reads, if ANTHROPIC_API_KEY) -> returns HOME -> writes one facility report.

  ros2 launch go2_bringup mission.launch.py                      # headless, all gauge rooms
  ros2 launch go2_bringup mission.launch.py headless:=false      # watch in gz
  ros2 launch go2_bringup mission.launch.py zones:=NE,SC         # a subset (testing)

Each phase is ALSO runnable standalone: zone_sweeper / panorama_segmenter / gauge_inspector, and the nav
foundation alone via inspection_nav.launch.py. Requires FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (CP39) and the
maps at ~/.go2_maps.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "inspection_nav.launch.py")),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )
    mission = Node(
        package="go2_inspection", executable="inspection_mission", output="screen",
        parameters=[{"use_sim_time": True, "zones": LaunchConfiguration("zones")}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("zones", default_value=""),     # "" = all gauge rooms
        nav,
        # start the mission after localization (DB load) + Nav2 are fully up
        TimerAction(period=100.0, actions=[mission]),
    ])
