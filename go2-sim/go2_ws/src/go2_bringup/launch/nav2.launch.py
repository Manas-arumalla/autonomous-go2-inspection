"""Lean Nav2 bringup for the Go2 (no AMCL — slam_toolbox owns map->odom).

  ros2 launch go2_bringup nav2.launch.py            # nav2 only (sim+slam already up)

Brings up ONLY the nodes we need for NavigateToPose + frontier exploration:
controller_server, smoother_server, planner_server, behavior_server, bt_navigator + a
lifecycle_manager. We deliberately do NOT use nav2_bringup/navigation_launch.py because in Jazzy
it also force-starts docking_server + waypoint_follower + collision_monitor, which abort the whole
lifecycle bringup unless fully configured. controller_server publishes /cmd_vel directly (Twist),
which CHAMP's quadruped_controller consumes — no velocity_smoother/collision_monitor chain.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    params = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    common = [params, {"use_sim_time": use_sim_time}]

    managed = ["controller_server", "smoother_server", "planner_server",
               "behavior_server", "bt_navigator"]

    nodes = [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=common, remappings=remaps),                       # publishes /cmd_vel (Twist)
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server",
             output="screen", parameters=common, remappings=remaps),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=common, remappings=remaps),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=common, remappings=remaps),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=common, remappings=remaps),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": managed}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Default = the proven slam stack params. The RTAB-Map stack passes
        # params_file:=.../nav2_params_rtab.yaml (fixed facility-sized global costmap, no static layer,
        # so the frontier sees the whole world's unknown space instead of rtabmap's tiny projected grid).
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg, "config", "nav2_params.yaml")),
    ] + nodes)
