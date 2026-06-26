#!/usr/bin/env python3
"""Bridge: Go2 Native Front Camera -> ROS 2 CompressedImage.

This node is isolated because the Go2FrontVideoData message fails to deserialize
in CycloneDDS. It must be run with RMW_IMPLEMENTATION=rmw_fastrtps_cpp.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    from unitree_go.msg import Go2FrontVideoData
    _HAS_UNITREE = True
except ImportError:
    _HAS_UNITREE = False

from go2_bridge.domains import bridge_domain_id, go2_domain_id, init_context, spin_nodes


class CameraBridge(Node):
    def __init__(self, source_context, output_context) -> None:
        super().__init__('camera_bridge', context=source_context)
        self._out_node = Node('camera_bridge_output', context=output_context)

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

        if not _HAS_UNITREE:
            self.get_logger().error("unitree_go not found! Cannot bridge camera.")
            return

        self._camera_sub = self.create_subscription(
            Go2FrontVideoData, '/frontvideostream', self._camera_cb, go2_qos
        )
        self._camera_pub = self._out_node.create_publisher(
            CompressedImage, '/camera/front/compressed', standard_qos
        )

        self._camera_count = 0
        self.get_logger().info('camera_bridge initialized (FastDDS mode): /frontvideostream -> /camera/front/compressed')

    @property
    def output_node(self) -> Node:
        return self._out_node

    def _camera_cb(self, msg: Go2FrontVideoData) -> None:
        self._camera_count += 1
        if self._camera_count % 30 == 1:
            self.get_logger().info(f'Received Camera frame (count={self._camera_count})')
        
        if not msg.video720p:
            return

        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_link'
        out.format = 'h264'
        try:
            out.data = bytes(msg.video720p)
            self._camera_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera frame: {e}')

    def destroy_node(self) -> bool:
        self._out_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    del args
    # This node must run with rmw_fastrtps_cpp
    source_domain = go2_domain_id()
    output_domain = bridge_domain_id()
    source_context = init_context(source_domain)
    output_context = init_context(output_domain)
    node = CameraBridge(source_context, output_context)

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
