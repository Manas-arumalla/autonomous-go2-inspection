"""Stage-1 sim bringup: Gazebo Harmonic + Go2 (velocity base + L1 LiDAR) + ros_gz bridge.

Brings the robot up publishing the mirrored real-Go2 topics (ADR-007):
  /utlidar/cloud_deskewed (PointCloud2), /utlidar/robot_odom (Odometry), /cmd_vel (in),
  /imu, /clock, TF odom->base_link->utlidar_lidar.

  ros2 launch go2_bringup sim.launch.py            # GUI
  ros2 launch go2_bringup sim.launch.py headless:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory("go2_bringup")
    pkg_worlds = get_package_share_directory("go2_worlds")
    pkg_desc = get_package_share_directory("go2_description")

    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    world = LaunchConfiguration("world").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)

    world_path = os.path.join(pkg_worlds, "worlds", world)
    urdf_path = os.path.join(pkg_desc, "urdf", "go2.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()
    bridge_cfg = os.path.join(pkg_bringup, "config", "ros_gz_bridge.yaml")

    gz_flags = "-s --headless-rendering -r -v3" if headless else "-r -v3"
    from launch.actions import ExecuteProcess
    gz = ExecuteProcess(
        cmd=["gz", "sim", *gz_flags.split(), world_path],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": f"{pkg_worlds}:{pkg_desc}"},
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "go2", "-z", spawn_z],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(pkg_bringup, "rviz", "slam.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return [gz, rsp, spawn, bridge, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="lab.sdf"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("spawn_z", default_value="0.40"),
        OpaqueFunction(function=_setup),
    ])
