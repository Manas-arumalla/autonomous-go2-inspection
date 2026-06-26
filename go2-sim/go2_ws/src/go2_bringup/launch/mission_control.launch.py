"""mission_control -- start ONLY the service trigger-layer node, beside whatever base stack is up.

This is additive and non-invasive: it does NOT bring up the sim / SLAM / Nav2 (run those as usual).
It advertises the 12 mission-control services (the future MCP tool surface). See docs/04-SERVICE-LAYER.md.

  # mapping mode  (rtabmap SLAM + Nav2 already up):
  ros2 launch go2_bringup mission_control.launch.py
  ros2 service call /start_exploration go2_inspection_interfaces/srv/ZoneTask "{}"

  # inspection mode (inspection_nav stack already up):
  ros2 launch go2_bringup mission_control.launch.py
  ros2 service call /inspect_zone go2_inspection_interfaces/srv/ZoneTask "{zone_id: zone_3, read: false}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("zones_file", default_value="~/.go2_maps/facility_inspection_zones.yaml"),
        DeclareLaunchArgument("map_name", default_value="facility_inspection"),
        Node(
            package="go2_inspection", executable="mission_control_server", name="mission_control",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "zones_file": LaunchConfiguration("zones_file"),
                "map_name": LaunchConfiguration("map_name"),
            }],
        ),
    ])
