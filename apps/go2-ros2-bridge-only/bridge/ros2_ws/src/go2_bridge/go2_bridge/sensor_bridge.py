#!/usr/bin/env python3
"""Bridge: Go2 Native Sensors -> Standard ROS 2 Sensors.

Bridges:
- /utlidar/cloud_deskewed -> /pointcloud (PointCloud2)
- /utlidar/imu -> /imu (Imu)
- /frontvideostream -> /camera/front/compressed (CompressedImage)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes


class SensorBridge(Node):
    def __init__(self, source_context, output_context) -> None:
        super().__init__('sensor_bridge', context=source_context)
        self._out_node = Node('sensor_bridge_output', context=output_context)

        self.declare_parameter('restamp_with_ros_time', True)
        self._restamp = bool(self.get_parameter('restamp_with_ros_time').value)

        go2_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        standard_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Lidar Bridge
        self._lidar_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud_deskewed', self._lidar_cb, go2_qos
        )
        self._lidar_pub = self._out_node.create_publisher(
            PointCloud2, '/pointcloud', standard_qos
        )

        # IMU Bridge
        self._imu_sub = self.create_subscription(
            Imu, '/utlidar/imu', self._imu_cb, go2_qos
        )
        self._imu_pub = self._out_node.create_publisher(
            Imu, '/imu', standard_qos
        )

        self._lidar_count = 0
        self._imu_count = 0

        self.get_logger().info(
            'sensor_bridge initialized: Lidar -> /pointcloud, IMU -> /imu '
            f'(restamp_with_ros_time={self._restamp})'
        )

    @property
    def output_node(self) -> Node:
        return self._out_node

    def _lidar_cb(self, msg: PointCloud2) -> None:
        self._lidar_count += 1
        if self._lidar_count % 100 == 1:
            self.get_logger().info(f'Received Lidar pointcloud (count={self._lidar_count})')
        if self._restamp:
            msg.header.stamp = self._out_node.get_clock().now().to_msg()
        self._lidar_pub.publish(msg)

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_count += 1
        if self._imu_count % 500 == 1:
            self.get_logger().info(f'Received IMU data (count={self._imu_count})')
        if self._restamp:
            msg.header.stamp = self._out_node.get_clock().now().to_msg()
        self._imu_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._out_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    source_domain = go2_domain_id()
    output_domain = bridge_domain_id()
    source_context = init_context(source_domain)
    output_context = init_context(output_domain)
    node = SensorBridge(source_context, output_context)

    try:
        spin_nodes([
            (node, source_context),
            (node.output_node, output_context),
        ])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=source_context)
        rclpy.try_shutdown(context=output_context)


if __name__ == '__main__':
    main()
