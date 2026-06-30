"""Lean Nav2 bringup for the Go2 (no AMCL — slam_toolbox / rtabmap owns map->odom).

  ros2 launch go2_bringup nav2.launch.py                     # default: controller -> /cmd_vel directly
  ros2 launch go2_bringup nav2.launch.py use_safety:=true    # opt-in safety chain (twist_mux + smoother +
                                                             #   collision_monitor)

Brings up the nodes for NavigateToPose + frontier exploration: controller_server, smoother_server,
planner_server, behavior_server, bt_navigator + a lifecycle_manager. We deliberately do NOT use
nav2_bringup/navigation_launch.py because in Jazzy it force-starts docking_server + waypoint_follower +
collision_monitor, which abort the whole lifecycle bringup unless fully configured.

DEFAULT (use_safety:=false): controller_server publishes /cmd_vel directly (Twist), which CHAMP's
quadruped_controller consumes — the proven, minimal path. **This file's default behaviour is unchanged.**

OPT-IN SAFETY (use_safety:=true, ADR-016 M7b): insert the velocity-arbitration + obstacle-stop chain
    controller -> cmd_vel_nav -> twist_mux -> cmd_vel_mux -> velocity_smoother -> cmd_vel_smoothed
        -> collision_monitor -> cmd_vel (to CHAMP)
- twist_mux (config/twist_mux.yaml) priority-muxes nav (10) < teleop (50) < e-stop (100), so a zero-Twist
  on /cmd_vel_estop overrides everything; teleop overrides autonomy.
- velocity_smoother + collision_monitor (already configured in nav2_params*.yaml) are added to the managed
  lifecycle set; collision_monitor is ALWAYS the last hop, so nav AND teleop are obstacle-guarded.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("go2_bringup")
    params = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_safety = LaunchConfiguration("use_safety").perform(context).lower() in ("true", "1", "yes")
    remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    common = [params, {"use_sim_time": use_sim_time}]

    managed = ["controller_server", "smoother_server", "planner_server", "behavior_server", "bt_navigator"]
    ctrl_remaps = list(remaps)
    safety_nodes = []
    if use_safety:
        ctrl_remaps = remaps + [("cmd_vel", "cmd_vel_nav")]  # controller feeds the safety chain, not /cmd_vel
        tmux = os.path.join(pkg, "config", "twist_mux.yaml")
        safety_nodes = [
            Node(package="twist_mux", executable="twist_mux", name="twist_mux", output="screen",
                 parameters=[tmux, {"use_sim_time": use_sim_time}],
                 remappings=remaps + [("cmd_vel_out", "cmd_vel_mux")]),
            Node(package="nav2_velocity_smoother", executable="velocity_smoother",
                 name="velocity_smoother", output="screen", parameters=common,
                 remappings=remaps + [("cmd_vel", "cmd_vel_mux")]),  # in: cmd_vel_mux -> out: cmd_vel_smoothed
            Node(package="nav2_collision_monitor", executable="collision_monitor",
                 name="collision_monitor", output="screen", parameters=common,
                 remappings=remaps),  # cmd_vel_smoothed -> cmd_vel (config in nav2_params*.yaml)
        ]
        managed = managed + ["velocity_smoother", "collision_monitor"]

    nodes = [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=common, remappings=ctrl_remaps),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server",
             output="screen", parameters=common, remappings=remaps),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=common, remappings=remaps),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=common, remappings=remaps),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=common, remappings=remaps),
    ] + safety_nodes + [
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": managed}]),
    ]
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Default = the proven slam stack params. The RTAB-Map stack passes
        # params_file:=.../nav2_params_rtab.yaml (fixed facility-sized global costmap, no static layer,
        # so the frontier sees the whole world's unknown space instead of rtabmap's tiny projected grid).
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg, "config", "nav2_params.yaml")),
        # Opt-in obstacle-stop + velocity-arbitration chain (default OFF -> behaviour unchanged).
        DeclareLaunchArgument("use_safety", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
