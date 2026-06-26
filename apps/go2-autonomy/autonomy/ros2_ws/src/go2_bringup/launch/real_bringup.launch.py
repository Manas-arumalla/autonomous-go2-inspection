"""Real-robot bringup for the Go2 autonomy stack on WendyOS (the sim->real port of rtabmap_slam +
inspection_nav + nav2, with NO Gazebo/CHAMP).

This app runs ON TOP of the `go2-ros2-bridge-only` app, which publishes the clean robot contract on
ROS domain 30:
    /odom (+ odom->base_link TF), /joint_states, /pointcloud, /imu, /cmd_vel->Sport API.

What this launch adds (everything use_sim_time:=FALSE -- the real Go2 has no /clock):
  - base_link->utlidar_lidar / camera_link / camera_link_optical STATIC TF (the bridge gives only
    odom->base_link; these are the sensor mounts lifted from the sim URDF -- VERIFY/calibrate on the
    real robot). base_link == trunk on the Go2 (identity floating_base), so we publish from base_link.
  - self_filter: drop the dog's body from the bridge's /pointcloud -> /utlidar/cloud_filtered.
  - pointcloud_to_laserscan: flatten /utlidar/cloud_filtered -> /scan (Nav2 obstacle layer + frontier).
  - (optional) v4l2 USB camera -> /camera/image_raw + /camera/camera_info (the inspection pipeline is
    camera-frame + odometry only, so the camera MOUNT is not safety-critical -- intrinsics matter, not TF).
  - RTAB-Map (mapping OR localization) -> map->odom + /map (or /rtabmap/grid_map in inspection mode).
  - Nav2 (controller publishes /cmd_vel, which the bridge forwards to the Sport API).
  - (inspection mode) static nav2_map_server serving the saved facility /map.
  - (optional) octomap 3D voxel map.

  mode:=mapping     RTAB-Map SLAM (-d) + Nav2; drive frontier/save via mission_control services.
  mode:=inspection  RTAB-Map localization on ~/.ros/rtabmap.db + static map_server + Nav2; run inspect_zone.

  ros2 launch go2_bringup real_bringup.launch.py mode:=mapping
  ros2 launch go2_bringup real_bringup.launch.py mode:=inspection map_yaml:=/maps/facility.yaml enable_camera:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    grid_topic = LaunchConfiguration("grid_topic")
    map_yaml = LaunchConfiguration("map_yaml")
    nav2_params = LaunchConfiguration("nav2_params")
    continue_map = LaunchConfiguration("continue_map")

    is_inspection = IfCondition(PythonExpression(["'", mode, "' == 'inspection'"]))
    is_mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    want_map_server = IfCondition(PythonExpression(
        ["'", mode, "' == 'inspection' and '", map_yaml, "' != ''"]))

    # ---- sensor-mount static TF (bridge publishes only odom->base_link). VERIFY on the real robot. ----
    # Named args (Jazzy form). rpy lifted from the sim URDF (robot_gz.xacro): utlidar_joint rpy "0 0.15 0"
    # (the ~8.6deg L1 down-tilt), camera_optical_joint rpy "-1.5708 0 -1.5708" (ROS z-forward optical frame).
    lidar_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_utlidar",
        arguments=["--x", "0.30", "--y", "0", "--z", "-0.02", "--roll", "0", "--pitch", "0.15", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "utlidar_lidar"],
        parameters=[{"use_sim_time": use_sim_time}], output="screen")
    cam_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_camera",
        arguments=["--x", "0.30", "--y", "0", "--z", "0.06", "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}], output="screen")
    cam_opt_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="camera_to_optical",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
                   "--frame-id", "camera_link", "--child-frame-id", "camera_link_optical"],
        parameters=[{"use_sim_time": use_sim_time}], output="screen")

    # ---- cloud cleanup + 2D scan ----
    self_filter = Node(
        package="go2_exploration", executable="self_filter", name="self_filter", output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "input_topic": cloud_topic,            # bridge /pointcloud (frame utlidar_lidar)
                     "output_topic": "/utlidar/cloud_filtered"}])
    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", "/utlidar/cloud_filtered"), ("scan", "/scan")],
        parameters=[os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml"),
                    {"use_sim_time": use_sim_time}])

    # ---- EXTERNAL USB RGB camera, OWNED BY THIS APP -> /camera/image_raw + /camera/camera_info (what the
    # inspection nodes subscribe to). Keep the BRIDGE's enable_usb_camera OFF so two v4l2 nodes don't fight
    # over /dev/video*. camera_info_url defaults to a generic 720p calibration so CameraInfo isn't all-zeros
    # (the sweeper's fx>0 guard would otherwise refuse to stitch) -- recalibrate for accuracy. ----
    camera = Node(
        package="v4l2_camera", executable="v4l2_camera_node", name="camera", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[{"use_sim_time": use_sim_time,
                     "video_device": LaunchConfiguration("camera_device"),
                     "camera_info_url": LaunchConfiguration("camera_info_url"),
                     "camera_frame_id": "camera_link_optical",
                     "image_size": [1280, 720]}],   # matches camera_generic_720p.yaml; edit both together
        remappings=[("/image_raw", "/camera/image_raw"), ("/camera_info", "/camera/camera_info")])

    # ---- RTAB-Map (ported from rtabmap_slam.launch.py; PURE-LIDAR, Reg/Force3DoF, Grid/Sensor=0) ----
    rtab = {
        "use_sim_time": use_sim_time, "frame_id": "base_link", "map_frame_id": "map",
        "database_path": LaunchConfiguration("rtabmap_db"),   # PERSISTED /maps -> survives redeploy (mapping
        #   -d wipes only this file; inspection/localization loads it). Default /maps/rtabmap.db.
        "subscribe_depth": False, "subscribe_rgb": False, "subscribe_scan_cloud": True,
        "approx_sync": True, "approx_sync_max_interval": 0.0, "sync_queue_size": 30,
        "wait_for_transform": 0.5, "qos_scan": 1, "qos_odom": 1,
        "Reg/Strategy": "1", "Reg/Force3DoF": "true",
        "Icp/VoxelSize": "0.05", "Icp/PointToPlane": "true",
        "Icp/MaxCorrespondenceDistance": "0.3", "Icp/Epsilon": "0.001",
        "RGBD/ProximityBySpace": "true", "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/NeighborLinkRefining": "true", "RGBD/AngularUpdate": "0.05", "RGBD/LinearUpdate": "0.05",
        "Grid/Sensor": "0", "Grid/RayTracing": "true", "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false", "RGBD/CreateOccupancyGrid": "true",
        "RGBD/OptimizeMaxError": "3.0", "Grid/CellSize": "0.05", "Grid/RangeMax": "5.0",
        "Grid/MaxGroundHeight": "0.10", "Grid/MaxObstacleHeight": "1.8",
        "map_always_update": True, "map_empty_ray_tracing": False,
        "Mem/STMSize": "30", "Rtabmap/DetectionRate": "2.0",
    }
    rtab_remap = [("scan_cloud", "/utlidar/cloud_filtered"), ("odom", odom_topic), ("map", grid_topic)]
    map_p = dict(rtab); map_p["Mem/IncrementalMemory"] = "true"
    cont_p = dict(map_p); cont_p["Mem/InitWMWithAllNodes"] = "true"
    loc_p = dict(rtab); loc_p["Mem/IncrementalMemory"] = "false"
    loc_p["Mem/InitWMWithAllNodes"] = "true"; loc_p["RGBD/StartAtOrigin"] = "true"

    fresh_cond = IfCondition(PythonExpression(
        ["'", mode, "' == 'mapping' and '", continue_map, "' == 'false'"]))
    rtab_map = Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
                    output="screen", parameters=[map_p], remappings=rtab_remap, arguments=["-d"],
                    condition=fresh_cond)
    rtab_cont = Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
                     output="screen", parameters=[cont_p], remappings=rtab_remap, arguments=[],
                     condition=IfCondition(continue_map))
    rtab_loc = Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap",
                    output="screen", parameters=[loc_p], remappings=rtab_remap, condition=is_inspection)

    # ---- inspection mode: static map_server serving the saved facility /map ----
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server", output="screen",
        condition=want_map_server,
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml,
                     "topic_name": "/map", "frame_id": "map"}])
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
        output="screen", condition=want_map_server,
        parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": ["map_server"]}])

    # ---- Nav2 (controller publishes /cmd_vel -> bridge -> Sport API) ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "params_file": nav2_params}.items(),
        condition=IfCondition(LaunchConfiguration("enable_nav2")))

    # ---- optional 3D octomap ----
    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "octomap.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration("enable_octomap")))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),       # REAL robot: no /clock
        DeclareLaunchArgument("mode", default_value="mapping"),             # mapping | inspection
        DeclareLaunchArgument("cloud_topic", default_value="/pointcloud"),  # bridge clean cloud
        DeclareLaunchArgument("odom_topic", default_value="/odom"),         # bridge clean odom
        DeclareLaunchArgument("grid_topic", default_value="/map"),          # inspection: /rtabmap/grid_map
        DeclareLaunchArgument("map_yaml", default_value=""),                # inspection static map
        DeclareLaunchArgument("rtabmap_db", default_value="/maps/rtabmap.db"),  # persisted localization graph
        DeclareLaunchArgument("continue_map", default_value="false"),
        DeclareLaunchArgument("enable_camera", default_value="false"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_info_url",   # generic 720p; override with a real calibration for accuracy
                              default_value="package://go2_bringup/config/camera_generic_720p.yaml"),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        DeclareLaunchArgument("enable_octomap", default_value="false"),
        DeclareLaunchArgument("nav2_params",
                              default_value=os.path.join(pkg, "config", "nav2_params_rtab.yaml")),
        lidar_tf, cam_tf, cam_opt_tf, self_filter, p2l, camera,
        map_server, map_lifecycle, nav2,
        # start RTAB-Map a few seconds in, after the static TF + self_filter + the bridge's odom->base_link
        # are flowing (else it loses odometry at startup and stops adding nodes).
        TimerAction(period=5.0, actions=[rtab_map, rtab_cont, rtab_loc]),
        TimerAction(period=8.0, actions=[octomap]),
    ])
