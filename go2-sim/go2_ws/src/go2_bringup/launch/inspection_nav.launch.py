"""inspection_nav.launch.py -- Phase 5b: facility-wide localization + Nav2 for the gauge-inspection mission.

The robot LOCALIZES on the pre-built map and Nav2 can plan across the WHOLE facility:
  - RTAB-Map in LOCALIZATION mode loads the saved DB -> provides map->odom (+ its LOCAL grid on
    /rtabmap/grid_map, kept off /map).
  - a static `map_server` serves the FULL saved facility grid on /map (CP39: rtabmap's own loc /map is
    only a local window, so Nav2's global costmap needs this static full map instead).
  - Nav2 (the proven nav2_params_rtab tuning) plans + drives.
Robot starts at HOME (0,0). This is the navigation foundation the mission orchestrator (5c) drives.

  ros2 launch go2_bringup inspection_nav.launch.py                       # headless
  ros2 launch go2_bringup inspection_nav.launch.py headless:=false       # watch in gz
Requires the maps at ~/.go2_maps (symlink to go2-sim/maps) and FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (CP39).
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
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map_yaml")

    # sim + RTAB-Map LOCALIZATION (map->odom); its grid goes to /rtabmap/grid_map, NOT /map.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "rtabmap_slam.launch.py")),
        launch_arguments={"world": world, "headless": headless, "localization": "true",
                          "grid_topic": "/rtabmap/grid_map",
                          "spawn_x": LaunchConfiguration("spawn_x"),
                          "spawn_y": LaunchConfiguration("spawn_y"),
                          "spawn_yaw": LaunchConfiguration("spawn_yaw")}.items(),
    )
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": map_yaml,
                     "topic_name": "map", "frame_id": "map"}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
        output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": "true",
                          "params_file": os.path.join(pkg, "config", "nav2_params_rtab.yaml"),
                          "use_safety": LaunchConfiguration("use_safety")}.items(),  # opt-in safety chain
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("world", default_value="facility_inspection.sdf"),
        DeclareLaunchArgument("map_yaml",
                              default_value=os.path.expanduser("~/.go2_maps/facility_inspection_map.yaml")),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_safety", default_value="false"),  # forward to nav2: twist_mux + collision_monitor
        slam,
        TimerAction(period=5.0, actions=[map_server, map_lifecycle]),   # static /map up early
        TimerAction(period=85.0, actions=[nav2]),                       # after rtabmap loc map->odom
    ])
