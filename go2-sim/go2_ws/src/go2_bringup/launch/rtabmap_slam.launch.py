"""RTAB-Map graph SLAM (localization + 3D mapping) for the Go2 -- ADR-015.

A SEPARATE, parallel stack to the slam_toolbox + frontier 2D stack (does NOT touch it). Fuses the
4D LiDAR (/utlidar/cloud_filtered) + RGB camera (/camera) + accurate odom (/odom) into one
memory-managed graph-SLAM node that publishes:
  - map->odom TF          (localization; replaces slam_toolbox here)
  - /map                  (2D occupancy grid, projected from the LiDAR -- for Nav2 + frontier_explorer)
  - /rtabmap/cloud_map    (assembled 3D point cloud, RGB-coloured)
  - /rtabmap/mapGraph     (pose graph: nodes + odometry links + loop-closure links)
  - /rtabmap/mapData      (full graph data)

  ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility.sdf                 # mapping (GUI sim)
  ros2 launch go2_bringup rtabmap_slam.launch.py world:=facility.sdf headless:=true  # headless gz
  ros2 launch go2_bringup rtabmap_slam.launch.py localization:=true                  # localize on saved DB

Why this works where the CP11 LiDAR-only attempt drifted: now we ALSO use the RGB camera (visual loop
closure) + the accurate fused /odom (not RTAB-Map's own ICP odom) + Reg/Force3DoF (ground-plane lock).
Sim-agnostic: identical topics on the real Go2 (ADR-002/007); Orin-friendly (DetectionRate 2Hz, lean ICP).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    headless = LaunchConfiguration("headless")
    with_sim = LaunchConfiguration("with_sim")
    localization = LaunchConfiguration("localization")
    continue_map = LaunchConfiguration("continue_map")   # resume + EXTEND a saved ~/.ros/rtabmap.db

    # The walking Go2 + gz sensors (4D LiDAR, RGB camera, IMU) + accurate odom (CHAMP+gz EKF).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "go2_champ.launch.py")),
        # NOTE: 'world' MUST be forwarded -- without it go2_champ falls back to its default lab.sdf, so
        # inspection_nav/mission/sim_mapping passing world:=maze.sdf were silently loading lab instead
        # (robot spawned in lab but localized against the maze/facility DB+map -> Nav2 goals unreachable
        # -> robot never moved). Default stays lab.sdf, so callers that don't pass world are unaffected.
        launch_arguments={"headless": headless, "champ": "true",
                          "world": LaunchConfiguration("world"),
                          "spawn_x": LaunchConfiguration("spawn_x"),
                          "spawn_y": LaunchConfiguration("spawn_y"),
                          "spawn_yaw": LaunchConfiguration("spawn_yaw")}.items(),
        condition=IfCondition(with_sim),
    )

    # Remove the dog's own body from the L1 cloud (so RTAB-Map's ICP + grid aren't polluted).
    self_filter = Node(
        package="go2_exploration", executable="self_filter", name="self_filter", output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # 2D /scan for Nav2's local obstacle layer (Phase 2). RTAB-Map builds the global grid itself.
    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", "/utlidar/cloud_filtered"), ("scan", "/scan")],
        parameters=[os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml"), {"use_sim_time": True}],
    )
    rtabmap_params = {
        "use_sim_time": True,
        "frame_id": "base_link",
        "map_frame_id": "map",
        # PURE-LIDAR SLAM (per user: the RGB-colour-on-3D-points requirement is dropped -- focus on the
        # frontier-exploration rtab approach). rtabmap localizes + builds the 2D grid + pose graph from the
        # 3D LiDAR + the accurate fused /odom; NO camera into SLAM, which also avoids the repetitive-facility
        # false visual loop closure that teleported the graph. The RGB camera stays available for the
        # inspection report (the end goal), just not for SLAM/3D colour.
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_scan_cloud": True,        # the 4D LiDAR (3D registration + 2D grid projection)
        "approx_sync": True,                 # LiDAR 10Hz / cam 15Hz / odom 50Hz aren't time-synced
        "approx_sync_max_interval": 0.0,     # 0 = no interval cap (sync the nearest in time)
        "sync_queue_size": 30,
        "wait_for_transform": 0.5,           # patience for the odom->base_link TF (sensor leads TF by ms)
        "qos_image": 1, "qos_camera_info": 1, "qos_scan": 1, "qos_odom": 1,
        # --- registration + loop closure ---
        "Reg/Strategy": "1",                 # ICP (strong geometric registration from the 3D LiDAR)
        "Reg/Force3DoF": "true",             # ground robot: lock z/roll/pitch (was the CP11 drift fix)
        "Icp/VoxelSize": "0.05",
        "Icp/PointToPlane": "true",
        "Icp/MaxCorrespondenceDistance": "0.3",
        "Icp/Epsilon": "0.001",
        "RGBD/ProximityBySpace": "true",     # LiDAR-geometry loop closure (sim walls are low-texture)
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/AngularUpdate": "0.05",        # add a node every 0.05 rad / 0.05 m of motion
        "RGBD/LinearUpdate": "0.05",
        "Vis/MinInliers": "12",              # (moot in pure-lidar mode; no visual loop closure)
        # --- 2D occupancy grid from the 3D LiDAR cloud (config from the Go2-Inspector reference) ---
        # THE fix for "rtabmap /map has no free cells": Grid/Sensor "0" builds the grid from the LiDAR
        # SCAN_CLOUD. Our earlier "1" meant "build from the DEPTH camera" -- but subscribe_depth=false, so
        # rtabmap had NO grid source -> /map was all-unknown/wall-only. "0" + RayTracing + map_empty_ray_tracing
        # carve real FREE space from each lidar ray -> /map gets free/occupied/unknown -> the frontier can run
        # DIRECTLY on /map (the reference architecture), no Nav2-costmap workaround needed.
        "Grid/Sensor": "0",
        "Grid/RayTracing": "true",           # carve free space: sensor->obstacle rays mark cells FREE
        "Grid/3D": "false",                  # rtabmap does the LIGHT 2D grid + graph; octomap_server does the 3D
        "Grid/NormalsSegmentation": "false", # flat-height ground split (robust on sparse lidar) -- matches ref
        "RGBD/CreateOccupancyGrid": "true",
        "RGBD/OptimizeMaxError": "3.0",      # tolerate odom-only links (few loop closures in low-texture sim)
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "5.0",              # match the reference (L1 reliable range for grid projection)
        "Grid/MaxGroundHeight": "0.10",      # z below this = floor = FREE; above = obstacle
        "Grid/MaxObstacleHeight": "1.8",
        # --- occupancy-grid node behaviour (Go2-Inspector reference) ---
        "map_always_update": True,           # refresh the grid every cycle, not only on new graph nodes
        "map_empty_ray_tracing": False,      # do NOT carve free PAST the last return: our sparse 32-ring
                                             # lidar has gaps, and free-past-walls created phantom frontiers
                                             # beyond the facility walls. Grid/RayTracing still carves free
                                             # up to each obstacle (safe). Walls stay solid in the grid.
        # --- memory (Orin-friendly) ---
        "Mem/STMSize": "30",
        "Rtabmap/DetectionRate": "2.0",      # process at 2 Hz, not every frame -> lighter
    }
    rtabmap_remap = [
        ("scan_cloud", "/utlidar/cloud_filtered"),
        ("odom", "/odom"),
        # RTAB-Map's 2D grid topic IS 'map'. Default -> /map (mapping/exploration). For the MISSION we set
        # grid_topic:=/rtabmap/grid_map so a STATIC map_server owns /map (full facility), while RTAB-Map
        # localization only provides map->odom + its local grid elsewhere (CP39: rtabmap loc /map is partial).
        ("map", LaunchConfiguration("grid_topic")),
    ]
    map_params = dict(rtabmap_params); map_params["Mem/IncrementalMemory"] = "true"   # SLAM
    loc_params = dict(rtabmap_params)
    loc_params["Mem/IncrementalMemory"] = "false"; loc_params["Mem/InitWMWithAllNodes"] = "true"  # localize
    # The robot ALWAYS (re)spawns at HOME = the map origin (mapping started there), so ASSUME the start pose
    # is the map origin instead of doing a GLOBAL relocalization -- which, in a near-symmetric world (the
    # maze), snapped to a ROTATED match so the static /map and the live localized frame appeared misaligned
    # (CP45). Localization-mode ONLY: SLAM (-d) + continue modes never set this, so mapping is unaffected.
    loc_params["RGBD/StartAtOrigin"] = "true"
    # CONTINUE: keep mapping (IncrementalMemory stays true) but LOAD all nodes of the existing DB into
    # working memory so the FULL prior map is active + republished, then extend it. No -d (don't wipe).
    cont_params = dict(map_params); cont_params["Mem/InitWMWithAllNodes"] = "true"

    # fresh mapping fires only when NOT localizing AND NOT continuing (so -d never wipes a resumed DB)
    fresh_cond = IfCondition(PythonExpression(
        ["'", localization, "' == 'false' and '", continue_map, "' == 'false'"]))
    rtabmap_map = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
        output="screen", parameters=[map_params], remappings=rtabmap_remap,
        arguments=["-d"],                    # -d: start a fresh database (mapping)
        condition=fresh_cond,
    )
    rtabmap_cont = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
        output="screen", parameters=[cont_params], remappings=rtabmap_remap,
        arguments=[],                        # NO -d: load + EXTEND the existing ~/.ros/rtabmap.db
        condition=IfCondition(continue_map),
    )
    rtabmap_loc = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
        output="screen", parameters=[loc_params], remappings=rtabmap_remap,
        condition=IfCondition(localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("with_sim", default_value="true"),
        DeclareLaunchArgument("world", default_value="lab.sdf"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("continue_map", default_value="false"),
        DeclareLaunchArgument("grid_topic", default_value="/map"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        sim, self_filter, p2l,
        # start RTAB-Map AFTER the sim + CHAMP EKF are up, so its first frames have a stable
        # odom->base_link TF (else it loses odometry at startup and stops adding nodes).
        TimerAction(period=14.0, actions=[rtabmap_map, rtabmap_cont, rtabmap_loc]),
        # 3D map: octomap_server on the SAME LiDAR cloud, built in RTAB-Map's loop-closure-corrected
        # 'map' frame (rtabmap's internal dense cloud_map assembly was unreliable here). Starts after
        # rtabmap publishes map->odom. -> /occupied_cells_vis_array (colored 3D voxels).
        TimerAction(period=20.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "octomap.launch.py")),
                launch_arguments={"use_sim_time": "true"}.items()),
        ]),
    ])
