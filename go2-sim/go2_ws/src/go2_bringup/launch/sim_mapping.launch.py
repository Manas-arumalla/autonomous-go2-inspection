"""sim_mapping.launch.py -- ONE COMMAND for MODE A (mapping / frontier exploration).

Brings up the sim + RTAB-Map SLAM (T1) and then STAGES Nav2 (T2) behind a timer, so you no longer
have to hand-time two terminals. The old failure was launching nav2.launch.py before RTAB-Map had
published map->odom -> Nav2's global_costmap logs "Invalid frame ID 'map' ... frame does not exist"
(and can abort the lifecycle bringup). Here Nav2 only starts AFTER RTAB-Map is publishing map->odom.

Timeline (all timers are relative to launch t=0; rtabmap_slam's own internal timers run inside it):
  t=0   gz + CHAMP gait spin up (inside rtabmap_slam -> go2_champ)
  t=14  RTAB-Map starts (fresh SLAM); publishes map->odom within ~2s once odom+cloud flow
  t=20  octomap_server (inside rtabmap_slam)
  t=24  Nav2 (controller/planner/behavior/bt + lifecycle) -- map frame now exists

  ros2 launch go2_bringup sim_mapping.launch.py headless:=false          # watch in gz (mapping)
  ros2 launch go2_bringup sim_mapping.launch.py                          # headless
  ros2 launch go2_bringup sim_mapping.launch.py continue_map:=true       # resume + extend a saved DB
  ros2 launch go2_bringup sim_mapping.launch.py nav2_delay:=30.0         # slower laptop -> wait longer
  ros2 launch go2_bringup sim_mapping.launch.py with_nav2:=false         # sim+rtabmap only (no nav2)

After this is up, start MODE A's driver/services in their own terminals (frontier_explorer or
mission_control), exactly as before -- this launcher only replaces the T1+T2 pair.

NOTE: this mirrors inspection_nav.launch.py for the LOCALIZATION/inspection (MODE B) path -- use
mission.launch.py / inspection_nav.launch.py for that. This file is the fresh-mapping counterpart.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # noqa: F401  (kept for parity / future inline nodes)


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")

    # T1: sim + RTAB-Map SLAM (fresh map). Same include the user ran by hand as terminal 1.
    # grid_topic defaults to /map (mapping/exploration). localization/continue_map are passed through
    # so this one launcher also covers "resume a saved DB" (continue_map:=true).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "rtabmap_slam.launch.py")),
        launch_arguments={
            "headless": headless,
            "world": LaunchConfiguration("world"),
            "localization": LaunchConfiguration("localization"),
            "continue_map": LaunchConfiguration("continue_map"),
            "grid_topic": LaunchConfiguration("grid_topic"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    # T2: Nav2 with the RTAB-Map facility-sized costmap tuning. Same include the user ran by hand as
    # terminal 2 -- only difference is it is gated behind a timer so map->odom exists first.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(pkg, "config", "nav2_params_rtab.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("world", default_value="lab.sdf"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("continue_map", default_value="false"),
        DeclareLaunchArgument("grid_topic", default_value="/map"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("with_nav2", default_value="true"),
        # RTAB-Map starts at t=14 and publishes map->odom within ~2s on a fresh DB; 24s leaves margin
        # for a loaded laptop running the gz GUI. Bump this if Nav2 still logs "frame 'map' does not
        # exist" (it retries, so a late start self-heals; this just avoids the noise).
        DeclareLaunchArgument("nav2_delay", default_value="24.0"),
        slam,
        TimerAction(period=LaunchConfiguration("nav2_delay"),
                    actions=[nav2], condition=IfCondition(LaunchConfiguration("with_nav2"))),
    ])
