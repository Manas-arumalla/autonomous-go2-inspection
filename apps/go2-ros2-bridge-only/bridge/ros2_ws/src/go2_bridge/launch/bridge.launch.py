#!/usr/bin/env python3
"""Launch file for Go2 bridge nodes.

Starts all bridge nodes that convert Unitree Go2 topics to standard
ROS2 interfaces for use with Nav2, SLAM, and other ROS2 stacks.

Usage:
  ros2 launch go2_bridge bridge.launch.py
  ros2 launch go2_bridge bridge.launch.py enable_cmd_vel:=false
  ros2 launch go2_bridge bridge.launch.py odom_source:=/uslam/frontend/odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # --- Launch arguments ---
    odom_source_arg = DeclareLaunchArgument(
        'odom_source',
        default_value='/utlidar/robot_odom',
        description='Source odometry topic from Go2',
    )
    odom_output_arg = DeclareLaunchArgument(
        'odom_output',
        default_value='/odom',
        description='Output odometry topic (Nav2 compatible)',
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID',
    )
    enable_tf_arg = DeclareLaunchArgument(
        'enable_tf',
        default_value='true',
        description='Publish odom->base_link TF',
    )
    restamp_arg = DeclareLaunchArgument(
        'restamp_with_ros_time',
        default_value='true',
        description='Restamp raw Go2 messages with local ROS time for Nav2/TF compatibility',
    )
    enable_cmd_vel_arg = DeclareLaunchArgument(
        'enable_cmd_vel',
        default_value='true',
        description='Enable cmd_vel -> sport API bridge',
    )
    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable topic monitor node',
    )
    enable_usb_camera_arg = DeclareLaunchArgument(
        'enable_usb_camera',
        default_value='false',
        description='Enable external USB v4l2 camera bridge',
    )
    max_vx_arg = DeclareLaunchArgument(
        'max_vx', default_value='0.6',
        description='Max forward velocity (m/s)',
    )
    max_vy_arg = DeclareLaunchArgument(
        'max_vy', default_value='0.4',
        description='Max lateral velocity (m/s)',
    )
    max_vyaw_arg = DeclareLaunchArgument(
        'max_vyaw', default_value='1.0',
        description='Max yaw rate (rad/s)',
    )
    lowstate_topic_arg = DeclareLaunchArgument(
        'lowstate_topic',
        default_value='/lowstate',
        description='LowState source topic',
    )
    monitor_interval_arg = DeclareLaunchArgument(
        'monitor_interval',
        default_value='10.0',
        description='Topic monitor check interval (seconds)',
    )

    # --- Nodes ---
    odom_tf_node = Node(
        package='go2_bridge',
        executable='odom_tf_bridge',
        output='screen',
        parameters=[{
            'odom_source_topic': LaunchConfiguration('odom_source'),
            'odom_output_topic': LaunchConfiguration('odom_output'),
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'base_frame_id': LaunchConfiguration('base_frame'),
            'publish_tf': LaunchConfiguration('enable_tf'),
            'restamp_with_ros_time': LaunchConfiguration('restamp_with_ros_time'),
        }],
    )

    joint_state_node = Node(
        package='go2_bridge',
        executable='joint_state_bridge',
        output='screen',
        parameters=[{
            'lowstate_topic': LaunchConfiguration('lowstate_topic'),
            'joint_states_topic': '/joint_states',
            'base_frame_id': LaunchConfiguration('base_frame'),
        }],
    )

    cmd_vel_node = Node(
        package='go2_bridge',
        executable='cmd_vel_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cmd_vel')),
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'sport_request_topic': '/api/sport/request',
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_vyaw': LaunchConfiguration('max_vyaw'),
            'watchdog_timeout': 0.5,
            'watchdog_rate': 10.0,
        }],
    )

    monitor_node = Node(
        package='go2_bridge',
        executable='topic_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
        parameters=[{
            'check_interval': LaunchConfiguration('monitor_interval'),
            'log_all': False,
        }],
    )

    sensor_node = Node(
        package='go2_bridge',
        executable='sensor_bridge',
        output='screen',
        parameters=[{
            'restamp_with_ros_time': LaunchConfiguration('restamp_with_ros_time'),
        }],
    )

    # External USB Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_usb_camera')),
        remappings=[
            ('/image_raw', '/camera/external/image_raw'),
            ('/image_raw/compressed', '/camera/external/compressed'),
        ],
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link'
        }]
    )

    return LaunchDescription([
        # Arguments
        odom_source_arg,
        odom_output_arg,
        odom_frame_arg,
        base_frame_arg,
        enable_tf_arg,
        restamp_arg,
        enable_cmd_vel_arg,
        enable_monitor_arg,
        enable_usb_camera_arg,
        max_vx_arg,
        max_vy_arg,
        max_vyaw_arg,
        lowstate_topic_arg,
        monitor_interval_arg,
        # Info
        LogInfo(msg='=== Go2 Bridge: starting all bridge nodes ==='),
        # Nodes
        odom_tf_node,
        joint_state_node,
        cmd_vel_node,
        monitor_node,
        sensor_node,
        camera_node,
    ])
