#!/usr/bin/env python3
"""Periodically checks which expected Go2 topics are alive.

Logs a summary of discovered vs expected topics. Useful for verifying
that the DDS/CycloneDDS connection to the Go2 is working and which
topics are actually publishing.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes

# All expected Go2 topics with their types, grouped by priority.
# This list matches the full topic inventory from the Go2.
EXPECTED_TOPICS: dict[str, str] = {
    # === Priority 1: Standard ROS2 sensor topics (Nav2/SLAM ready) ===
    '/utlidar/cloud_deskewed': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/cloud': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/cloud_base': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/grid_map': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/height_map': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/range_map': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/voxel_map': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/imu': 'sensor_msgs/msg/Imu',
    '/utlidar/robot_odom': 'nav_msgs/msg/Odometry',
    '/utlidar/robot_pose': 'geometry_msgs/msg/PoseStamped',
    '/utlidar/range_info': 'geometry_msgs/msg/PointStamped',
    '/uslam/cloud_map': 'sensor_msgs/msg/PointCloud2',
    '/uslam/frontend/cloud_world_ds': 'sensor_msgs/msg/PointCloud2',
    '/uslam/frontend/odom': 'nav_msgs/msg/Odometry',
    '/uslam/localization/cloud_world': 'sensor_msgs/msg/PointCloud2',
    '/uslam/localization/odom': 'nav_msgs/msg/Odometry',
    '/uslam/map_file_pub': 'sensor_msgs/msg/PointCloud2',
    '/uslam/map_file_sub': 'sensor_msgs/msg/PointCloud2',
    '/uslam/navigation/global_path': 'sensor_msgs/msg/PointCloud2',
    '/lio_sam_ros2/mapping/odometry': 'nav_msgs/msg/Odometry',
    # === Priority 2: Unitree custom topics (unitree_go) ===
    '/lowstate': 'unitree_go/msg/LowState',
    '/lf/lowstate': 'unitree_go/msg/LowState',
    '/sportmodestate': 'unitree_go/msg/SportModeState',
    '/lf/sportmodestate': 'unitree_go/msg/SportModeState',
    '/lowcmd': 'unitree_go/msg/LowCmd',
    '/wirelesscontroller': 'unitree_go/msg/WirelessController',
    '/wirelesscontroller_unprocessed': 'unitree_go/msg/WirelessController',
    '/uwbstate': 'unitree_go/msg/UwbState',
    '/uwbswitch': 'unitree_go/msg/UwbSwitch',
    '/utlidar/lidar_state': 'unitree_go/msg/LidarState',
    '/utlidar/height_map_array': 'unitree_go/msg/HeightMap',
    '/utlidar/voxel_map_compressed': 'unitree_go/msg/VoxelMapCompressed',
    '/audioreceiver': 'unitree_go/msg/AudioData',
    '/audiosender': 'unitree_go/msg/AudioData',
    '/frontvideostream': 'unitree_go/msg/Go2FrontVideoData',
    '/config_change_status': 'unitree_go/msg/ConfigChangeStatus',
    # === Priority 3: Unitree API topics (unitree_api) ===
    '/api/sport/request': 'unitree_api/msg/Request',
    '/api/sport/response': 'unitree_api/msg/Response',
    '/api/motion_switcher/request': 'unitree_api/msg/Request',
    '/api/motion_switcher/response': 'unitree_api/msg/Response',
    '/api/robot_state/request': 'unitree_api/msg/Request',
    '/api/robot_state/response': 'unitree_api/msg/Response',
    '/api/slam_operate/request': 'unitree_api/msg/Request',
    '/api/slam_operate/response': 'unitree_api/msg/Response',
    '/api/obstacles_avoid/request': 'unitree_api/msg/Request',
    '/api/obstacles_avoid/response': 'unitree_api/msg/Response',
    '/api/config/request': 'unitree_api/msg/Request',
    '/api/config/response': 'unitree_api/msg/Response',
    '/api/sport_lease/request': 'unitree_api/msg/Request',
    '/api/sport_lease/response': 'unitree_api/msg/Response',
    '/api/uwbswitch/request': 'unitree_api/msg/Request',
    '/api/uwbswitch/response': 'unitree_api/msg/Response',
    '/api/voice/request': 'unitree_api/msg/Request',
    '/api/voice/response': 'unitree_api/msg/Response',
    '/api/vui/request': 'unitree_api/msg/Request',
    '/api/vui/response': 'unitree_api/msg/Response',
    '/api/audiohub/request': 'unitree_api/msg/Request',
    '/api/audiohub/response': 'unitree_api/msg/Response',
    '/api/videohub/request': 'unitree_api/msg/Request',
    '/api/videohub/response': 'unitree_api/msg/Response',
    '/api/gpt/request': 'unitree_api/msg/Request',
    '/api/gpt/response': 'unitree_api/msg/Response',
    '/api/pet/request': 'unitree_api/msg/Request',
    '/api/pet/response': 'unitree_api/msg/Response',
    '/api/bashrunner/request': 'unitree_api/msg/Request',
    '/api/bashrunner/response': 'unitree_api/msg/Response',
    '/api/assistant_recorder/request': 'unitree_api/msg/Request',
    '/api/assistant_recorder/response': 'unitree_api/msg/Response',
    '/api/fourg_agent/request': 'unitree_api/msg/Request',
    '/api/fourg_agent/response': 'unitree_api/msg/Response',
    '/api/gas_sensor/request': 'unitree_api/msg/Request',
    '/api/gas_sensor/response': 'unitree_api/msg/Response',
    '/api/gesture/request': 'unitree_api/msg/Request',
    '/api/programming_actuator/request': 'unitree_api/msg/Request',
    '/api/programming_actuator/response': 'unitree_api/msg/Response',
    '/api/arm/request': 'unitree_api/msg/Request',
    '/api/rm_con/request': 'unitree_api/msg/Request',
    # === Priority 4: std_msgs/String informational topics ===
    '/audio_msg': 'std_msgs/msg/String',
    '/audiohub/player/state': 'std_msgs/msg/String',
    '/gas_sensor': 'std_msgs/msg/String',
    '/gesture/result': 'std_msgs/msg/String',
    '/gnss': 'std_msgs/msg/String',
    '/gpt_cmd': 'std_msgs/msg/String',
    '/gpt_state': 'std_msgs/msg/String',
    '/gptflowfeedback': 'std_msgs/msg/String',
    '/lf/battery_alarm': 'std_msgs/msg/String',
    '/multiplestate': 'std_msgs/msg/String',
    '/pet/flowfeedback': 'std_msgs/msg/String',
    '/programming_actuator/command': 'std_msgs/msg/String',
    '/programming_actuator/feedback': 'std_msgs/msg/String',
    '/public_network_status': 'std_msgs/msg/String',
    '/qt_notice': 'std_msgs/msg/String',
    '/rtc/state': 'std_msgs/msg/String',
    '/rtc_status': 'std_msgs/msg/String',
    '/selftest': 'std_msgs/msg/String',
    '/servicestate': 'std_msgs/msg/String',
    '/servicestateactivate': 'std_msgs/msg/String',
    '/sit_stand/heartbeat': 'std_msgs/msg/String',
    '/slam_info': 'std_msgs/msg/String',
    '/slam_key_info': 'std_msgs/msg/String',
    '/uslam/client_command': 'std_msgs/msg/String',
    '/uslam/server_log': 'std_msgs/msg/String',
    '/utlidar/client_command': 'std_msgs/msg/String',
    '/utlidar/mapping_cmd': 'std_msgs/msg/String',
    '/utlidar/server_log': 'std_msgs/msg/String',
    '/utlidar/switch': 'std_msgs/msg/String',
    '/videohub/inner': 'std_msgs/msg/String',
    '/webrtcreq': 'std_msgs/msg/String',
    '/webrtcres': 'std_msgs/msg/String',
    '/xfk_webrtcreq': 'std_msgs/msg/String',
    '/xfk_webrtcres': 'std_msgs/msg/String',
    '/arm/action/state': 'std_msgs/msg/String',
    # === Priority 5: System topics ===
    '/rosout': 'rcl_interfaces/msg/Log',
    '/parameter_events': 'rcl_interfaces/msg/ParameterEvent',
}


class TopicMonitor(Node):
    """Monitors Go2 topic availability and logs status."""

    def __init__(self, source_context, status_context) -> None:
        super().__init__('topic_monitor', context=source_context)
        self._status_node = Node('topic_monitor_status', context=status_context)

        self.declare_parameter('check_interval', 10.0)
        self.declare_parameter('log_all', False)

        interval = self.get_parameter('check_interval').value
        self._log_all = self.get_parameter('log_all').value

        # Status publisher for external monitoring
        self._status_pub = self._status_node.create_publisher(
            String, '/go2_bridge/topic_status', 10
        )

        self._timer = self.create_timer(interval, self._check_topics)
        self._check_count = 0

        self.get_logger().info(
            f'topic_monitor: checking {len(EXPECTED_TOPICS)} expected topics '
            f'every {interval}s'
        )

    @property
    def status_node(self) -> Node:
        return self._status_node

    def _check_topics(self) -> None:
        self._check_count += 1
        discovered = dict(self.get_topic_names_and_types())

        found = []
        missing = []
        for topic, expected_type in EXPECTED_TOPICS.items():
            if topic in discovered:
                found.append(topic)
            else:
                missing.append(topic)

        # Extra topics not in our expected list
        extra = [
            t for t in discovered
            if t not in EXPECTED_TOPICS
            and not t.startswith('/go2_bridge/')
        ]

        summary = (
            f'Topic check #{self._check_count}: '
            f'{len(found)}/{len(EXPECTED_TOPICS)} expected topics found, '
            f'{len(missing)} missing, {len(extra)} unexpected'
        )

        # First check or log_all: show details
        if self._check_count <= 2 or self._log_all:
            self.get_logger().info(summary)
            if missing:
                # Group missing by priority category
                self.get_logger().warn(
                    f'Missing topics ({len(missing)}): '
                    + ', '.join(missing[:20])
                    + ('...' if len(missing) > 20 else '')
                )
            if extra and self._check_count == 1:
                self.get_logger().info(
                    f'Unexpected topics ({len(extra)}): '
                    + ', '.join(extra[:10])
                    + ('...' if len(extra) > 10 else '')
                )
        else:
            self.get_logger().info(summary)

        # Publish JSON status
        status = json.dumps({
            'check': self._check_count,
            'found': len(found),
            'expected': len(EXPECTED_TOPICS),
            'missing_count': len(missing),
            'missing_topics': missing[:30],
            'extra_count': len(extra),
        })
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._status_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    source_domain = go2_domain_id()
    status_domain = bridge_domain_id()
    source_context = init_context(source_domain)
    status_context = init_context(status_domain)
    node = TopicMonitor(source_context, status_context)
    node.get_logger().info(
        f'topic_monitor domains: Go2 source={source_domain}, '
        f'status output={status_domain}'
    )
    try:
        spin_nodes([
            (node, source_context),
            (node.status_node, status_context),
        ])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=source_context)
        rclpy.try_shutdown(context=status_context)


if __name__ == '__main__':
    main()
