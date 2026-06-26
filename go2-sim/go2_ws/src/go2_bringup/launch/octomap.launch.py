"""3D occupancy map (OctoMap) for the Go2 — the expert colored-voxel 3D map (like the reference repos).

octomap_server consumes the L1 3D cloud (/utlidar/cloud_deskewed) and the EXISTING TF tree
(map->odom->base_link->utlidar_lidar provided by slam_toolbox + CHAMP), and assembles a global 3D
occupancy octomap. It does NOT do localization (slam_toolbox owns map->odom), so navigation stays
reliable — this is purely the 3D visualization/mapping layer.

Publishes: /octomap_full, /octomap_point_cloud_centers, /occupied_cells_vis_array (colored voxels).

  ros2 launch go2_bringup octomap.launch.py        # run alongside slam_toolbox + sim

Sim-agnostic (ADR-002/004): identical on the real Go2 (same /utlidar/cloud_deskewed + TF).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="octomap_server", executable="octomap_server_node", name="octomap_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": "map",                # global frame; cloud transformed via TF
                "base_frame_id": "base_link",
                "resolution": 0.10,               # voxel size (the colored cubes)
                "sensor_model.max_range": 4.5,    # only insert nearby accurate points -> no phantom
                                                  # walls from far/sparse returns smearing on pose shifts
                "sensor_model.hit": 0.7,
                "sensor_model.miss": 0.45,        # stronger free-space clearing of spurious voxels
                "sensor_model.min": 0.12,
                "sensor_model.max": 0.97,
                "occupancy_thres": 0.5,
                "pointcloud_min_z": 0.08,         # DROP the floor: the L1 is pitched ~8.6 deg down (utlidar
                                                  # rpy 0 0.15 0) so its rings rake the ground a few m out.
                                                  # Filtering below 8cm (map frame, floor=0) stops the whole
                                                  # floor being voxelised into the blue ground-blanket; only
                                                  # walls/objects (>=0.08m) remain. Octomap is NOT a costmap
                                                  # source here, so this is purely the 3D viz (the /scan band
                                                  # below is the costmap's ground rejection).
                "pointcloud_max_z": 2.5,
                "filter_ground_plane": False,     # z-threshold (above) is more reliable than PCL plane-seg on
                                                  # sparse 4D-lidar; flat sim/facility floor is at z~0.
                "height_map": True,               # color voxels by height (the rainbow look)
                "colored_map": False,
                "latch": False,
            }],
            remappings=[("cloud_in", "/utlidar/cloud_filtered")],  # self-filtered (no robot body) — ADR-013
        ),
    ])
