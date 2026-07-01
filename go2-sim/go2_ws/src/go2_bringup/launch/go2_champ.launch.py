"""Real walking Go2 in Gazebo Harmonic via CHAMP + gz_ros2_control (ADR-010).

  ros2 launch go2_bringup go2_champ.launch.py                 # GUI, lab world
  ros2 launch go2_bringup go2_champ.launch.py headless:=true

Layers (staggered): gz sim + RSP -> spawn + bridge -> controllers -> champ gait.
The Go2 then walks from /cmd_vel; sensors publish the mirrored real-Go2 topics (ADR-007).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *a, **k):
    desc = get_package_share_directory("go2_description")
    cfg = get_package_share_directory("go2_config")
    worlds = get_package_share_directory("go2_worlds")
    bringup = get_package_share_directory("go2_bringup")
    champ_bringup = get_package_share_directory("champ_bringup")

    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    world = LaunchConfiguration("world").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)   # spawn pose (warehouse clear aisle = -8,0)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)   # spawn heading (rad); 0 = +X (default)
    world_path = os.path.join(worlds, "worlds", world)
    xacro_file = os.path.join(desc, "xacro", "robot_gz.xacro")
    # process xacro via subprocess (file as a single argv element) — the workspace path
    # contains a space, which breaks the Command substitution's shlex split.
    import subprocess
    _proc = subprocess.run(["xacro", xacro_file], capture_output=True, text=True)
    robot_description = _proc.stdout
    # write a no-space URDF copy for champ_bringup (it also xacro-processes its path)
    urdf_out = "/tmp/go2_gz.urdf"
    with open(urdf_out, "w") as _f:
        _f.write(robot_description)

    gz_flags = "-s --headless-rendering -r -v3" if headless else "-r -v3"
    # gz must find the gz_ros2_control system plugin (libgz_ros2_control-system.so) — it lives
    # in the ROS lib dir, which is NOT on gz's default system-plugin search path.
    ros_lib = os.path.join(os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/jazzy").split(":")[0], "lib")
    sys_plugin_path = os.pathsep.join(p for p in ["/opt/ros/jazzy/lib", ros_lib,
                                                  os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")] if p)
    gz = ExecuteProcess(cmd=["gz", "sim", *gz_flags.split(), world_path], output="screen",
                        additional_env={"GZ_SIM_RESOURCE_PATH": f"{worlds}:{desc}",
                                        "GZ_SIM_SYSTEM_PLUGIN_PATH": sys_plugin_path})

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "go2",
                            "-x", spawn_x, "-y", spawn_y, "-z", "0.30", "-Y", spawn_yaw])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": os.path.join(bringup, "config", "ros_gz_bridge_champ.yaml"),
                               "use_sim_time": True}])

    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_states_controller", "-c", "/controller_manager"])
    jtc = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_group_effort_controller", "-c", "/controller_manager"])

    do_champ = LaunchConfiguration("champ").perform(context).lower() in ("1", "true", "yes")
    champ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(champ_bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "description_path": urdf_out,
            "joints_map_path": os.path.join(cfg, "config/joints/joints.yaml"),
            "links_map_path": os.path.join(cfg, "config/links/links.yaml"),
            "gait_config_path": os.path.join(cfg, "config/gait/gait.yaml"),
            "use_sim_time": "true",
            "robot_name": "go2",
            "gazebo": "true",
            "lite": "false",
            "rviz": "false",
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "true",   # gait-phase contact estimate -> feeds state_estimation odom/raw
            "close_loop_odom": "true",
        }.items(),
    )

    # Tight staggering minimises the window where the dog is spawned but no controller is holding
    # it (limp legs -> collapse). gz up -> spawn -> controllers active -> gait, ~6.5s total.
    actions = [gz, rsp,
               TimerAction(period=3.0, actions=[spawn, bridge]),
               TimerAction(period=5.0, actions=[jsb]),
               TimerAction(period=6.0, actions=[jtc])]
    if do_champ:
        actions.append(TimerAction(period=8.0, actions=[champ]))  # gait controller holds stand, then walks
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world", default_value="lab.sdf"),
        DeclareLaunchArgument("champ", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        OpaqueFunction(function=_setup),
    ])
