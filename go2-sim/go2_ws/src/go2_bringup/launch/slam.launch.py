"""SLAM bringup: (optionally) the sim + pointcloud_to_laserscan + slam_toolbox (2D).

  ros2 launch go2_bringup slam.launch.py                 # sim + SLAM (GUI)
  ros2 launch go2_bringup slam.launch.py headless:=true  # headless
  ros2 launch go2_bringup slam.launch.py with_sim:=false # SLAM only (sim already up)

Produces /scan, /map, and the map->odom transform (slam_toolbox owns it — no ground truth).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")
    with_sim = LaunchConfiguration("with_sim")
    rviz = LaunchConfiguration("rviz")

    # The walking Go2 (CHAMP) is the sim provider — gz + real Go2 + gait + leg/IMU odometry
    # (odom->base_footprint->base_link TF, /odom) + L1 LiDAR bridge (/utlidar/cloud_deskewed).
    # (The old box sim.launch.py is kept for reference; superseded by go2_champ per ADR-010/011.)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "go2_champ.launch.py")),
        launch_arguments={"headless": headless, "champ": "true"}.items(),
        condition=IfCondition(with_sim),
    )

    # self_filter removes the robot's OWN body from the L1 cloud (ADR-013) so the LiDAR dead zone
    # can be tiny -> walls get seen + cleared next to the robot (fixes wall-trap + wall-climbing).
    self_filter = Node(
        package="go2_exploration", executable="self_filter", name="self_filter", output="screen",
        parameters=[{"use_sim_time": True}],
    )

    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", "/utlidar/cloud_filtered"), ("scan", "/scan")],  # self-filtered cloud
        parameters=[os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml"),
                    {"use_sim_time": True}],
    )

    # slam_toolbox is a LIFECYCLE node — it must be configured+activated. Reuse its stock
    # launch (which performs the lifecycle transitions) with our params, instead of a plain
    # Node (which leaves it UNCONFIGURED so it never subscribes to /scan).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": os.path.join(pkg, "config", "slam_toolbox.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("with_sim", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        sim, self_filter, p2l, slam,
    ])
