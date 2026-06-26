"""RTAB-Map 3D LiDAR SLAM for the Go2 (ADR-004 "3D later") — the expert 3D map.

Consumes the L1 3D cloud (/utlidar/cloud_deskewed) + CHAMP leg/IMU odometry (/odom) and produces:
  - map->odom TF (replaces slam_toolbox; loop-closure corrected)
  - /map         2D occupancy grid (height-segmented from the 3D cloud) -> Nav2 / frontier
  - /rtabmap/cloud_map  assembled 3D point-cloud map -> RViz (the dense expert 3D map)
Also runs pointcloud_to_laserscan so Nav2's local costmap keeps a /scan obstacle source.

  ros2 launch go2_bringup rtabmap.launch.py            # rtabmap + p2l (sim+nav launched separately)

Sim-agnostic (ADR-002): on the real Go2 the same /utlidar/cloud_deskewed + /odom feed this 1:1.
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

    # 3D cloud -> /scan, still needed for Nav2's local-costmap obstacle layer
    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", "/utlidar/cloud_deskewed"), ("scan", "/scan")],
        parameters=[os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml"),
                    {"use_sim_time": use_sim_time}],
    )

    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",         # external odom from CHAMP EKF (TF odom->base_footprint)
        "map_frame_id": "map",
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_scan_cloud": True,    # 3D LiDAR mode
        "approx_sync": True,
        "wait_for_transform": 0.3,
        "qos_scan": 1,
        "publish_tf": True,              # owns map->odom
        # --- registration: ICP for LiDAR; constrain to ground plane (stable) but keep 3D cloud ---
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "true",
        "Mem/NotLinkedNodesKept": "false",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "Icp/PointToPlane": "true",
        "Icp/Iterations": "10",
        "Icp/VoxelSize": "0.1",
        "Icp/MaxCorrespondenceDistance": "0.3",
        "Icp/PointToPlaneK": "20",
        "Icp/Epsilon": "0.001",
        "Icp/MaxTranslation": "2.0",
        "Icp/CorrespondenceRatio": "0.2",
        # --- 2D occupancy grid (for Nav2) from the 3D cloud, height-segmented ---
        "Grid/FromDepth": "false",
        "Grid/3D": "true",              # keep vertical structure (walls!) — false projects flat to xy
        "Grid/CellSize": "0.06",
        "Grid/RangeMax": "8.0",
        "Grid/RayTracing": "true",
        "Grid/NormalsSegmentation": "false",
        "Grid/MaxGroundHeight": "0.12",  # floor vs walls
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/ClusterRadius": "0.1",
        "GridGlobal/UpdateError": "0.02",
    }

    rtabmap = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
        parameters=[rtabmap_params],
        remappings=[("scan_cloud", "/utlidar/cloud_deskewed"),
                    ("odom", "/odom"),
                    ("grid_map", "/map")],         # feed Nav2 / frontier the same /map contract
        arguments=["--delete_db_on_start"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        p2l,
        rtabmap,
    ])
