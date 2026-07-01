"""demo.launch.py — ONE-COMMAND flagship demo (ADR-018).

Brings up the whole autonomous-inspection environment so a viewer sees BOTH the physical simulation
(Gazebo) AND the robot's internal planning / perception / state (RViz) side by side:

    Gazebo (sim) + RTAB-Map localization + Nav2 + RViz (inspection view) + the 14-service control layer
    + the detected-gauge RViz markers.

  ros2 launch go2_bringup demo.launch.py                          # maze, GUI + RViz, ready for a mission
  ros2 launch go2_bringup demo.launch.py mission:=true           # ALSO auto-run the full inspection mission
  ros2 launch go2_bringup demo.launch.py rviz:=false             # Gazebo only (no RViz)
  ros2 launch go2_bringup demo.launch.py headless:=true          # RViz only (no Gazebo window)
  ros2 launch go2_bringup demo.launch.py use_safety:=true        # + twist_mux/collision_monitor safety chain
  ros2 launch go2_bringup demo.launch.py world:=facility_inspection.sdf map_yaml:=~/.go2_maps/facility_inspection_map.yaml zones_file:=~/.go2_maps/facility_inspection_zones.yaml

This is purely a composition of the existing, unchanged launches (inspection_nav + mission_control) plus
RViz + the marker node, so every component still launches on its own. Nothing working is modified.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map_yaml")
    zones_file = LaunchConfiguration("zones_file")
    use_rviz = LaunchConfiguration("rviz")

    # 1) sim + RTAB-Map localization + Nav2  (the proven inspection stack — included unchanged)
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "inspection_nav.launch.py")),
        launch_arguments={
            "headless": headless,
            "world": world,
            "map_yaml": map_yaml,
            "use_safety": LaunchConfiguration("use_safety"),
        }.items(),
    )

    # 2) the 14-service control layer (after the sim core has started)
    mc = TimerAction(period=8.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "mission_control.launch.py")),
        launch_arguments={"zones_file": zones_file}.items(),
    )])

    # 3) RViz — the inspection view (map, robot, scan, costmaps, plan, frontiers, zones, detections, camera)
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", os.path.join(pkg, "rviz", "inspection.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # 4) detected-gauge markers for RViz (read-only; publishes /inspection/objects)
    markers = Node(
        package="go2_inspection", executable="inspection_markers", name="inspection_markers", output="log",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # 5) OPTIONAL: auto-run the full inspection mission once the stack is up (~95 s for Nav2 + localization)
    mission = TimerAction(period=98.0, actions=[ExecuteProcess(
        cmd=["ros2", "run", "go2_inspection", "inspection_mission", "--ros-args",
             "-p", "use_sim_time:=true",
             "-p", ["zones_file:=", zones_file],
             "-p", ["map_yaml:=", map_yaml],
             "-p", "inspect:=true", "-p", "return_home:=true", "-p", "read_approach:=true"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mission")),
    )])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),   # demo: show the Gazebo window
        DeclareLaunchArgument("world", default_value="maze.sdf"),
        DeclareLaunchArgument("map_yaml",
                              default_value=os.path.expanduser("~/.go2_maps/maze_map.yaml")),
        DeclareLaunchArgument("zones_file",
                              default_value=os.path.expanduser("~/.go2_maps/maze_zones.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_safety", default_value="false"),
        DeclareLaunchArgument("mission", default_value="false"),
        nav, mc, rviz, markers, mission,
    ])
