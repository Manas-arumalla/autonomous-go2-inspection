"""explore_lite (m-explore-ros2) autonomous frontier exploration for the Go2 (ADR-014).

The PROVEN community explorer, replacing our custom frontier_explorer. It reads frontiers from the
Nav2 GLOBAL COSTMAP (so goals are always planner-reachable) and drives Nav2's NavigateToPose.

  ros2 launch go2_bringup explore_lite.launch.py        # run after sim+slam + nav2 are up

Sim-agnostic: identical on the real Go2 (same Nav2 costmap + NavigateToPose action).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="explore_lite", executable="explore", name="explore_node", output="screen",
            parameters=[os.path.join(pkg, "config", "explore_lite.yaml"),
                        {"use_sim_time": use_sim_time}],
        ),
    ])
