"""Stage-1 FULL autonomy: Gazebo Go2 + SLAM + Nav2 + frontier exploration -> autonomous map.

  ros2 launch go2_bringup explore.launch.py                 # GUI
  ros2 launch go2_bringup explore.launch.py headless:=true  # headless

Brings up sim + slam_toolbox, then Nav2, then the frontier_explorer (autostart) which drives
the Go2 to map the unknown world. Layers are staggered so each is ready before the next.
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
    rviz = LaunchConfiguration("rviz")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "slam.launch.py")),
        launch_arguments={"headless": headless, "with_sim": "true", "rviz": rviz}.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    frontier = Node(
        package="go2_exploration", executable="frontier_explorer", name="frontier_explorer",
        output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "robot_base_frame": "base_link"}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        slam,
        TimerAction(period=18.0, actions=[nav2]),       # after CHAMP sim (~13s) + slam are up
        TimerAction(period=28.0, actions=[frontier]),   # after Nav2 lifecycle is active
    ])
